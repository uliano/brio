# One-time programmable memory, read side (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), chapter 13
(13.1 the address map and the four read windows with 13.1.1 the guarded
ones, 13.2 and 13.3 the IP and the hardware around it with 13.3.1 the
lock shim, 13.3.2 the two interfaces and the interrupt, 13.3.3 the boot
oscillator, 13.3.4 the power-up state machine, 13.4 the critical flags,
13.5 the page locks with 13.5.1 the progression, 13.5.2 the access keys,
13.5.3 the encoding, 13.5.5 what a blank device carries, 13.6 the error
correction with 13.6.1 bit repair by polarity and 13.6.2 the modified
Hamming code, 13.7 decommissioning, 13.9 the registers, 13.10 the
predefined data locations), 4.5 (what the boot path reads out of it) and
10.8 (its place in the security architecture); Appendix E, RP2350-E16 and
RP2350-E28. The driver: `brio/rp2350/otp.hpp`. The reference suite:
`test_rp2350_blocks`, letters a, h and j, which runs on both of this
chip's architectures from one source.

## There is no write path here, and that is a decision

Every bit of this array goes from zero to one exactly once and never
back. Among those bits are CRIT1's `SECURE_BOOT_ENABLE`, `DEBUG_DISABLE`
and `SECURE_DEBUG_DISABLE` and CRIT0's two architecture disables (13.4) -
each of which, once set, permanently removes a debug port, a processor
architecture or the ability to run unsigned code on the part that carries
it. A driver that can program OTP is a driver that can destroy the board
it is being developed on, with no way back and no second try.

So this file has no SBPI bridge, no wrapper for the bootrom's
`otp_access` (whose argument carries an IS_WRITE bit - see
[bootrom.md](bootrom.md)), and no soft lock either: SW_LOCKn is read and
decoded, never written, because a lock only ever advances and the one
that costs nothing to set costs a reset to undo. The programming side of
chapter 13 belongs to whatever tool provisions a device, and that tool is
not a brio program. The family check asserts the absence: the verbs a
programming driver would carry are checked to be missing on every sweep,
on both architectures and both packages.

## What the silicon does

**4096 rows of 24 bits**, logically 64 pages of 64 rows. Sixteen of each
row's bits are data and eight are protection, which is where "8 kB with
ECC" comes from. A blank device reads all zeroes except for what the
factory wrote at manufacturing test.

**Four read windows** (13.1), in a 64 kB region above the control
registers:

| Base | What a read gives | On a refused read |
|------|-------------------|-------------------|
| `0x40130000` | 16 bits per row, hardware-corrected, addressed as a HALFWORD at 2 x row | all-ones |
| `0x40134000` | the row's 24 raw bits as a word at 4 x row, correction bypassed | all-ones |
| `0x40138000` | the ECC data again, with hardware consistency checks | a BUS FAULT |
| `0x4013c000` | the raw data again, with the same checks | a BUS FAULT |

Bit 14 of the address picks raw over ECC and bit 15 guarded over
unguarded. An unguarded refusal is indistinguishable from a row that
genuinely reads all-ones, which is exactly why the guarded windows exist
and why the bootrom uses them for boot flags; but a bus fault is not a
value a function can return. So this driver offers both, and offers a
lock-checked `read()` above them: the page's permission is read FIRST and
only then is the array touched, so a caller gets an answer or an empty
optional and never a fault.

The windows answer only while USR.DCTRL stands (13.1's own note). It is
set at reset and clear only while the programming bridge owns the array,
so a program that finds it clear would take a bus error from every read
above.

**The locks are a thermometer that only climbs** (13.5.1). Read/write,
then read-only, then inaccessible; Secure and Non-secure advance
independently; the registers are preloaded from the array at reset, are
world-readable, and IGNORE a write of a lower value. Reads and writes are
not orthogonal here - the IP performs both in the course of programming a
row, so a page whose reads are blocked has its writes blocked too
(13.3.1). On a device that has only been through manufacturing test
(13.5.5): page 0 read-only to everyone, pages 1 and 2 read-only to
Non-secure, page 62 and page 63 with their own rules, and everything else
open.

**ECC, and the sentence that has to be read carefully** (13.6). An ECC
row carries 16 bits of data, a 6-bit modified Hamming code in bits 21:16
and 2 bits of bit-repair-by-polarity in 23:22. The hardware corrects
transparently and tells nobody, so the only way for a program to learn
that a row needed correcting - or is beyond correcting - is to read the
RAW row and do the arithmetic. 13.6.2 publishes that arithmetic as a
six-entry parity table.

Its decode sentence is "ECC recalculates the six parity bits based on the
value read from the OTP row" and XORs them with the stored six. For the
five Hamming bits that is unambiguous: their masks cover data bits only,
so recomputing them from the sixteen data bits is right. **The sixth mask
covers bits 0..20 - the data AND the five Hamming bits** - so
recalculating IT from the data alone and the recomputed Hamming bits
gives a check that is blind to a whole class of single-bit errors: a flip
in data bit 3, or in any stored parity bit, comes out with the syndrome's
top bit CLEAR and would be filed as uncorrectable. The sixth check must
be the parity of the WHOLE RECEIVED row, which is what makes the code
single-error-correcting and double-error-detecting: any odd number of
flips sets it. This driver's `otp_ecc_checks()` is that definition, and
the family check sweeps all twenty-two single flips (each located and
repaired) and all two hundred and thirty-one double flips (each detected
and refused) against it at COMPILE time, with no silicon involved.

**Bit repair by polarity** (13.6.1) compensates for one bit that was
already set before a row was programmed: the row is stored inverted and
both of bits 23:22 are set to say so. A decoder checks for that pair and
inverts the row before the Hamming stage.

**Redundancy without ECC** (13.10). A row whose bits are programmed at
different times cannot carry ECC, because the code covers all sixteen
data bits at once. Those rows are replicated: best-of-three for most of
them, and **three-of-eight for the critical flags** (13.4) - a rule
deliberately biased towards reading a flag as SET.

**ARCHSEL_STATUS lives in this block**, at `OTP_BASE + 0x15c`. The
critical flags that force an architecture are read by this block's own
power-up state machine, so the register that reports the result sits
beside them rather than in the power manager. It is the one register of
this chapter a program on either half reads every day.

**The block has no init and is not the reset controller's.** Its power-up
state machine ran before the first instruction of this image existed
(13.3.4), from a ring oscillator of its own that randomises its
frequency; the rest of the system is held in reset until it finishes, so
that no software runs before the array's contents are known. 13.3 also
states a rate limit that a clock chapter must respect: **clk_ref may not
exceed 25 MHz while the OTP is accessed.**

**The decommissioning flag** (13.7) is a spare bit of the page 63 lock
word. Set, it re-enables the factory test port and makes pages 3..61
inaccessible. This driver reads it (`DBG.CUSTOMER_RMA_FLAG`) and nothing
more.

## Types and verbs

- The geometry: `otp_row_count`, `otp_page_rows`, `otp_page_count`,
  `otp_ecc_bytes`, `otp_clk_ref_max_hz`, the four window bases and the
  two "refused" patterns.
- `Otp::read(row)` - a row with its ECC verdict and no fault whatever the
  locks say: the page's lock, then the raw window, then 13.6's
  arithmetic. `Otp::read_run(first, rows)` reads several rows as one
  unsigned value, least significant row first.
- `Otp::ecc(row)`, `raw(row)`, `ecc_guarded(row)`, `raw_guarded(row)` -
  the four windows themselves. The guarded pair FAULTS where the others
  return a pattern of ones.
- `OtpRow` - the raw 24 bits, the datum, `OtpEcc::ok|corrected|
  uncorrectable`, which bit was repaired and whether the row was stored
  inverted.
- The arithmetic, all constexpr: `otp_ecc_parity(data)` (13.6.2's own
  table and loop), `otp_ecc_checks(codeword)` (the five Hamming checks
  and the overall parity), `otp_ecc_decode(raw)`, `otp_even_parity()`,
  `otp_vote3()` and `otp_vote8()`.
- `Otp::page_of(row)`, `page_lock(page)` giving an `OtpPageLock` of two
  `OtpLock` states, `readable(page)`.
- This die's record: `chip_id()` (CHIPID0..3, a 64-bit public
  identifier), `random_id()` (RANDID0..7, 128 bits), `rosc_calib_khz()`,
  `lposc_calib_hz()`, `num_gpios()`, `info_crc()`, `flash_devinfo()`.
- The flag words, voted: `crit0()` and `crit1()` (three-of-eight),
  `boot_flags0()`, `boot_flags1()` and `usb_boot_flags()` (best of
  three), with the bit names in `OtpCrit0`, `OtpCrit1` and
  `OtpBootFlags0`.
- The block's own registers: `critical()` (what hardware latched, in its
  own bit positions, with the names in `OtpCritical`), `key_valid()`,
  `debugen()`, `debugen_lock()`, `archsel()`, `archsel_status()` giving
  an `OtpArchSel`, `bootdis()`, `data_window_enabled()`, `rma_flag()`,
  `debug_status()`, `interrupt_raw()`, `interrupt_status()`,
  `clear_interrupts()`.

## How to use it

```cpp
// a row, with its verdict, and never a fault
if (auto row = brio::Otp::read(brio::OtpRowId::num_gpios)) {
    if (row->ecc != brio::OtpEcc::uncorrectable) { use(row->data); }
}

// this die's name, and which architecture is running
const auto id = brio::Otp::chip_id();
const brio::OtpArchSel arch = brio::Otp::archsel_status();

// what a program must never do, read back so it can say so
const auto crit1 = brio::Otp::crit1();
const bool secure_boot = crit1 && (*crit1 & brio::OtpCrit1::secure_boot_enable);

// the guarded window, for a program that would rather fault than be lied to -
// after checking the page is readable
if (brio::Otp::readable(brio::Otp::page_of(row))) {
    const uint16_t v = brio::Otp::ecc_guarded(row);
}
```

## Not covered yet

Driver gaps, each with its reason:

- **The whole programming side**: the SBPI bridge, the bootrom's
  `otp_access`, and any soft lock. Declined, permanently, for the reason
  this document opens with. The absence is asserted by the family check
  rather than promised here.
- **The OTP access keys** (13.5.2): a page that needs a key is opened by
  writing the key into a write-only register, which is a provisioning
  act. `key_valid()` reports which keys are enrolled, and no board here
  has one.
- **The interrupt** (13.3.2's five sources: a Secure read refused, a
  Non-secure read refused, a write refused, the programming bridge's
  completion, and a data-port access made while the bridge owns the
  array). The flags are read and cleared; nothing arms the line, because
  every read this driver makes is lock-checked first and the other four
  sources belong to a programming path that does not exist here.
- **Verifying INFO_CRC.** The value is reported. Checking it would mean
  reading a hundred rows and implementing a SECOND CRC-32 variant -
  13.10 names the reflected form with a final inversion, which is the
  zlib polynomial's usual spelling and not the CRC-32/MPEG-2 that
  `util/crc.hpp` carries for the hardware blocks that compute it. Born
  with a program that has a reason to distrust page 0.
- **Decoding FLASH_DEVINFO's size fields**: the flash chapter's, because
  it is the only thing that can check the answer against a chip.
- **The OTP boot path** (OTPBOOT_SRC/LEN/DST, an image executed out of
  the array), the USB white-label descriptors, the boot key fingerprints
  and the boot version counters: all of them are provisioning data, read
  as ordinary rows by `read()` and given no named verbs because nothing
  here writes or boots from them.
- **RP2350-E16 and RP2350-E28.** The first is a set of correctness checks
  the A3 stepping adds to the power-up state machine; the second is about
  the lock word of page 62 and is worked around by the factory's default
  permissions. Neither is a thing a read-only driver acts on, and the
  bench part is an A2.

Implemented but not bench-verified, each with the letter of
`test_rp2350_blocks` that will measure it:

- The identity rows read and cross-checked against SYSINFO: the chip id,
  and NUM_GPIOS against the package bit (letter a).
- ARCHSEL_STATUS, whose verdict DIFFERS between the two halves - 0 under
  an Arm image, 3 under a RISC-V one, because the bootrom switches both
  cores into the architecture the image named (letter a).
- This board's whole record printed: the random id, the two calibration
  rows, INFO_CRC, FLASH_DEVINFO, CRIT0 and CRIT1 decoded bit by bit with
  the hardware's own CRITICAL latch beside them, the boot flags, the
  enrolled keys, the debug enables, the decommissioning flag, and the
  lock state of all sixty-four pages against 13.5.5's blank-device
  expectation (letter h).
- The three read paths against each other over page 0: the ECC window
  against the raw window decoded in software, the guarded window on a row
  the first two have just agreed about, and the count of rows that needed
  a bit repaired or were beyond repair (letter j).
- That an unprogrammed user row reads ZERO - an OTP cell starts at zero
  and goes to one, the opposite of every flash array in this tree
  (letter j).
- `data_window_enabled()`, `rma_flag()` and the interrupt registers: read
  and printed, never seen in any state but the quiet one.
