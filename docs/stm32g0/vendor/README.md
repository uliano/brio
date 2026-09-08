# Vendor documents

The reference manual, datasheet and errata the `stm32g0/` stratum is
written against (every target folder has its own `vendor/`). They are
NOT in the repository (`docs/*/vendor/*.pdf` is git-ignored): keep the
files listed below here (downloaded or symlinked), and cite them from
code and docs by document number and SECTION, never by page ("RM0444
5.4.4"): sections survive revisions, pages do not.

Check the revision before trusting a local copy: ST republishes all
three, and the errata sheet is PER PART NUMBER - the G0B1's is ES0548,
the G071's is ES0418, and the number of a part's errata sheet is never
guessable from a neighbour's (verify on st.com before citing).

## STM32G0x1 (`brio/stm32g0/`)

| Document | Number / revision of record | File name here | Notes |
|----------|-----------------------------|----------------|-------|
| STM32G0x1 reference manual | **RM0444 Rev 6** (December 2024) | symlink to `~/Documenti/Elettronica/STM32/STM32G0/rm0444-*.pdf` | the whole x1 line (G031/041/051/061/071/081/0B1/0C1); the x0 value line is RM0454 (same peripherals minus a few) |
| STM32G0B1xB/xC/xE datasheet | **DS13560 Rev 5** (June 2024) | symlink to `.../stm32g0b1ce.pdf` | pinout, alternate-function tables 13..24 (the AF numbers no header carries), electrical characteristics |
| STM32G0B1xB/xC/xE device errata | **ES0548 Rev 3** (October 2022) | symlink to `.../es0548-*.pdf` | silicon revisions A (REV_ID 0x1000) and Z (0x1001) in one document with a per-item column each |
| STM32G071x8/xB device errata | **ES0418 Rev 5** (November 2023) | symlink to `.../es0418-*.pdf` | the SECOND SILICON's (the Nucleo-G071RB at desk position F, revision B); READ AT THE BENCH - see "The second silicon's errata" below |
| STM32G031x4/x6/x8 datasheet | **DS12992 Rev 4** | not fetched as a PDF; the TEXT was read at the bench | the THIRD SILICON's: pin assignment (table 12 - the LQFP32 column is what says which pads exist), alternate-function tables 13..17, the low-power mode paragraphs of 3.7 |
| STM32G031x4/x6/x8 device errata | **ES0487** (number read off st.com's own listing, es0487-stm32g031x4x6x8-device-errata) | NOT OBTAINED | the third silicon's, and every route to it timed out at the bench: the erratum letters run on that board and their outcomes are recorded as MEASURED with this sheet's verdict PENDING - see "The third silicon" below |
| Getting started with STM32G0 hardware development | AN5096 Rev 4 (December 2025) | symlink to `.../an5096-*.pdf` | decoupling, clocks, boot pins |

Canonical URLs (redirect to the current revision):
`https://www.st.com/resource/en/reference_manual/rm0444-stm32g0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf`,
`https://www.st.com/resource/en/errata_sheet/es0548-stm32g0b1xbxcxe-device-errata-stmicroelectronics.pdf`.

The Nucleo-64 user manual (UM2324, MB1360 schematic) is NOT in the
folder: the board facts this stratum states (LD4 on PA5, B1 on PC13,
the ST-LINK virtual COM port on USART2 PA2/PA3, no HSE crystal fitted)
were each VERIFIED AT THE BENCH before being written down, and the
one still unverified (the LSE crystal) is said to be so in
[../README.md](../README.md).

## Device headers + SVD: vendored in the repo

arm-none-eabi-gcc ships no device headers, so the CMSIS device headers
ARE in the repository (Apache-2.0 allows it) so a fresh clone builds:

- `third_party/cmsis-device-g0/` - the `Include/` tree of
  STMicroelectronics' **cmsis-device-g0 v1.4.5** (every STM32G0 part:
  `stm32g0xx.h` is the umbrella that dispatches on `-DSTM32G0B1xx`,
  `stm32g0b1xx.h` the header of record for register names on the
  bench chip). When the reference manual and the header disagree on a
  name, the header wins in code and the manual's section is quoted in
  the comment. ST's startup templates are NOT vendored; the crt cites
  them for the handler NAMES only (`stm32g0/src/glue/`).
- `third_party/cmsis-core/` - the CMSIS-Core headers the device header
  includes (ARM CMSIS_5 5.9.0), shared with the samc21 stratum.
- `stm32g0/svd/STM32G0B1.svd` - ST's SVD for the debug Peripheral
  Viewer (Apache-2.0, from the cmsis-svd-data mirror of ST's pack),
  with its description text SANITIZED TO ASCII (the repo rule): ST's
  file carries double-encoded non-breaking spaces and curly quotes,
  mapped to plain ones, and a few symbols (micro, greater-or-equal)
  replaced by `?`. Register data untouched, the XML re-parsed.

## The bench chips, all three

| Board | Part | IDCODE | Sheet | Read at the bench? |
|---|---|---|---|---|
| E, Nucleo-G0B1RE (MB1360) | STM32G0B1RE, 512 KB / 144 KB, LQFP64 | **0x10016467** (DEV_ID 0x467, REV_ID 0x1001 = **revision Z**, ES0548 table 2) | ES0548 Rev 3 | yes, in hand |
| F, Nucleo-G071RB | STM32G071RB, 128 KB / 36 KB, LQFP64 | **0x20006460** (DEV_ID 0x460, REV_ID 0x2000 = **revision B**, ES0418 table 2) | ES0418 Rev 5 | yes, in hand |
| G, Nucleo-G031K8 (Nucleo-32) | STM32G031K8, 64 KB / 8 KB, LQFP32 | **0x10036466** (DEV_ID 0x466, REV_ID 0x1003) | ES0487 | **NO - not obtained** |

Every bench suite prints `part DEV_ID .. REV_ID ..` at boot from
`DeviceIdcode::read()`, because a measurement that differs between two
boards is only a finding once the die it was taken on is on the record.
Read the IDCODE at the bring-up of any new board - the errata columns
are per revision, and the SHEET is per part number.

## Errata ES0548: what touches the bring-up (revision Z)

Encoded in code or stated where the code cannot enforce it:
- **2.2.10 Prefetch failure when branching across flash memory banks**
  (no workaround, both revisions): why `FlashAccel::prefetch` is a
  verb and not a default - PRFTEN stays at its reset value (clear)
  until a measurement says otherwise (flash.hpp).
- **2.2.4 Wakeup from Stop not effective** with HSIDIV != 0 (no
  workaround): a divided `ClockSource::internal` rate is a stated
  caveat on the clock task and on the sleep sites (clock.hpp).
  **STAGED TWICE, WITH OPPOSITE ANSWERS, AND THE DIFFERENCE IS THE
  ERRATUM'S OWN WORDING.** It does NOT reach an RTC wake (a 250 ms Stop
  at HSIDIV = /4 woken by the RTC lasted its full length - rtc.md), and
  it DOES reach a USART wake (the same divider, a byte on the receive
  line, WUF never rising and the RTC backstop ending the sleep -
  usart.md): the item is about CLOCK-REQUEST-CAPABLE peripherals, and a
  serial port makes a request where a counter on LSE makes none. Setting
  RCC_CR.HSIKERON so no request is needed does NOT rescue it, which is
  measured and not explained.
- **2.11.1 Data corruption due to noisy receive line** (no
  workaround): a sub-half-bit glitch inside the second half of a stop
  bit corrupts the byte; the noise flag NE is counted separately by
  the Uart task for exactly that reason (usart.hpp). **STAGED AND
  REPRODUCED, WITH ITS CONTROL**: a quarter-bit glitch to zero placed by
  software in the second half of a stop bit at 2400 baud spoiled 8 of 8
  frames - 0x96 read back as 0xCB with NO error flag at all - while the
  identical glitch in the FIRST half spoiled none (usart.md).
- **2.8.1 Device may remain stuck in LPTIM interrupt when entering Stop
  mode** (no workaround but a SUBSTITUTION): clearing `CR.ENABLE` near
  an LPTIM interrupt can freeze the wake-up signal active, after which
  the device cannot enter Stop at all. The erratum's own remedy is "do
  not clear its ENABLE bit... instead, reset the whole LPTIMx peripheral
  via the RCC controller", so NO VERB IN `lptim.hpp` WRITES ENABLE = 0 -
  `disable()` and `reset()` are both an `RCC_APBRSTR1` pulse. The item
  is answered structurally and is NOT staged: reproducing it needs the
  very write the driver does not have, and its failure mode would leave
  the board unable to Stop (lptim.md).
- **2.8.2 Device may remain stuck in LPTIM interrupt when clearing event
  flag**: with at least one interrupt enabled, clearing a flag whose own
  interrupt is disabled at the instant a new event arrives can leave the
  interrupt line stuck high. All three parts of the workaround are code:
  `clear_flags()` REFUSES from thread mode while IER is nonzero
  (`__get_IPSR() == 0`), and `isr()` clears the disabled-interrupt flags
  FIRST and the enabled ones second. Both halves are bench-verified
  (lptim.hpp, test_stm32_lptim letter i).
- **2.13.1 Desynchronization under specific condition with edge
  filtering enabled** (workaround: disable edge filtering, or wait for
  the retransmission): `CCCR.EFBI` is a verb of `fdcan.hpp` with the
  obligation STATED on it - nothing a driver can enforce, since the
  coincidence is an end of integration landing on a falling FDCAN_RX
  edge. **STAGED AND NOT REPRODUCED**: 1500 CAN FD frames with bit rate
  switching in each arm, EFBI set and clear, every one byte-exact with
  no protocol error either side - recorded as unreproduced and not as a
  disproof, because a loop-back whose RX pin is disconnected may never
  offer the edge (fdcan.md).
- **2.13.2 Tx FIFO messages inverted under specific buffer usage and
  priority setting** is **UNREACHABLE BY CONSTRUCTION on this silicon,
  with the register as the evidence**: `FDCAN_TXBC` written 0xFFFFFFFF
  reads back 0x01000000, so the only implemented bit is TFQM. The item
  needs "both a dedicated Tx buffer and a Tx FIFO" and its own
  workaround names Tx buffers 4 and 5; this M_CAN is configured with
  three Tx elements and one mode bit, so the area is either a FIFO or a
  queue and a dedicated buffer cannot be declared (fdcan.md).
- 2.2.8 (boot select after a debug connection) and 2.11.2 (the USART
  prescaler exists only on some instances) are documentation errata,
  read and applied: the crt's boot assumptions and RM0444 table 183
  respectively. 2.11.2's subject is MEASURED and is worse than a missing
  divider: a BASIC instance TAKES a PRESC value and reads it back, and
  then transmits nothing at all - so `prescaler()` refuses on such an
  instance rather than trusting the readback (usart.md).

Read and NOT applicable to this bring-up (the peripheral is not
driven yet): 2.2.1 (LSI), 2.2.2 (PWR wake-up flags), 2.2.3 (a flash
double-word all-ones cannot be re-programmed to all zeros - the FLASH
campaign's), 2.2.6 (PC13 disturbs LSE), 2.2.11 (RTC domain), 2.3.1
(GPIO after Standby), 2.4.x/2.5.x (DMA/DMAMUX), 2.6.x (ADC), 2.7.x
(TIM), 2.9.1 (RTC), 2.10.x (I2C), 2.12.x (SPI). Revision A only, absent
on Z: 2.2.5, 2.2.7, 2.2.9, 2.6.5.

**No item of ES0548 touches the CRC calculation unit** - a statement
about the document, not a claim about the silicon (crc.md).

## The second silicon: STM32G071RB, and its errata read at the bench

The Nucleo-G071RB at desk position F carries an **STM32G071RB, DBGMCU_IDCODE
DEV_ID 0x460, REV_ID 0x2000 = silicon revision B** (ES0418 table 2; Y is
0x2002 and is the newer of the two). Every bench suite of this stratum
prints that pair at boot, through `DeviceIdcode::read()` in `flash.hpp`,
because a measurement that differs between two boards is only a finding
once the die it was taken on is on the record.

FOURTEEN OF THE SEVENTEEN SUITES RUN THERE (`../README.md` has the table
and the reasons for the other three), so ES0418 can be read against real
letters rather than against a driver's intentions. The status letters are
ST's own: A = present with a workaround, N = present with none, P =
partial, "-" = absent.

| ES0418 (rev B) | Where a letter reaches it | What the bench found |
|---|---|---|
| **2.2.4** DMAMUX cannot be synchronized or triggered by EXTI (N) | `test_stm32_serial`'s boot probe; `test_stm32_dma` letter `f` | **REPRODUCED, WITH ITS MECHANISM.** Four pad edges into a request generator moved 0 words with the EXTI event mask alone and 1 of 4 with the interrupt mask armed too, while the EXTI's own rising pending bit saw every edge and four SOFTWARE events on the same line, generator and channel moved 4. The trigger follows the LEVEL of the line's pending interrupt: no IMR bit, no level, no trigger; with one, the first edge raises it and nothing more arrives until the pending bit is cleared. `test_stm32_dma` letter `f` PASSES on this die because its loop clears after every SWIER pulse - a SWIER-only staging reports the path healthy. Four verdicts of the serial suite skip; the sheet's "no workaround" holds, since the clear would need a CPU in a loop that has none |
| **2.2.6** Wakeup from Stop not effective (N) | `test_stm32_serial` letter `w`, the RTC-wake controls in `test_stm32_rtc` and `test_stm32_sleep` | the G0B1's 2.2.4, staged the same way: letter `w` scores 2 of 2 on this die too - the USART poke at HSIDIV = /4 does not wake the part, the RTC backstop ends the sleep, and HSIKERON does not rescue it, exactly as on the G0B1 (usart.md); REPRODUCED on revision B |
| **2.7.1** Invalid DAC output if MODE is written before data (A) | every DAC letter of `test_stm32_analog` - `configure()` writes MCR.MODE with the channel disabled and before any data write, the erratum's exact condition | **NOT REPRODUCED** under configure-then-set: the ADC reads a correct output through PA4 every time. Recorded as unreproduced, not as a disproof; the workaround (one write to any data register, then the MODE) is STATED on `dac.hpp`'s `configure()`, and `set()` is that write |
| **2.6.2 / 2.6.4** CFGR1 or CFGR2 written with ADEN set resets RES or CKMODE (A) | `adc.hpp` | **ANSWERED STRUCTURALLY**, and audited: every CFGR1 and CFGR2 store in the file is under a cleared ADEN - `configure()` refuses while enabled, `rebase()`'s store is behind a `disable()` whose failure aborts it, and `configure_while_enabled()` is the deliberate staging ground. There is no third |
| **2.6.7** ADC offset out of specification (A on rev B) | `test_stm32_analog` letter `a` prints CALFACT | not reached: the item's condition is VREF+ below 3.0 V and this rail measures 3317 mV. CALFACT came out at 46, neither of the 0x00 / 0x7F extremes |
| **2.6.6** trigger latency documentation | - | no letter measures a trigger latency, so nothing is judged against the corrected 6.5 / 12.5 / 3.5 PCLK figures |
| **2.9.1 / 2.9.2** LPTIM stuck (A / P) | the G0B1's 2.8.1 / 2.8.2, structurally and in `test_stm32_lptim` letter `i` | unchanged: 82/82 on this die, both halves of the flag-clear discipline behaving as on the other |
| **2.11.1** I2C tSU;DAT floors (P) | `test_stm32_i2c` letter `m`'s refusals | identical - the floors are the driver's compile-time and run-time refusals and do not depend on the die |
| **2.11.2** I2C spurious BERR (A) | `test_stm32_i2c` letter `m` counts BERR over the run | not seen |
| **2.11.6** I2C stalled when PCLK/I2CCLK is in 1.5..3 (A) | - | NOT REACHED: this suite's ratios are 4 (a 64 MHz PCLK with the HSI16 kernel), 1 (PCLK as the kernel) and 0.125 (the 2 MHz core), none of them in the band |
| **2.11.3 / 2.11.5 / 2.11.7** own-address match, NOSTRETCH underrun, SMBus slave timeout | multi-master, the NOSTRETCH target and the SMBus target are SELF-LINK roles | not staged on this board: its I2C1 faces a peer, not a second instance of its own |
| **2.12.2** noisy receive line (A) | `test_stm32_serial` letter `f` | the G0B1's 2.11.1, staged identically |
| **2.12.3** prescaler documentation | `test_stm32_serial` letter `a` | measured on this die too, and worse than a missing divider: a BASIC instance takes the PRESC value, reads it back and transmits nothing |
| **2.12.4** data corrupted when ABREN is cleared during a reception (A) | `usart.hpp` | **ANSWERED STRUCTURALLY**: `auto_baud_off()` refuses with UE set, so there is no path to that write and a disabled receiver is not receiving |
| **2.12.5** NE set with ONEBIT on a noisy START bit (N) | letter `f`'s ONEBIT rows | not exercised: those rows run on a clean start bit, the suite's bit-banger putting its glitch in the stop bit |
| **2.12.1** USART SPI-slave TC anticipated (A) | - | not staged: there is no synchronous slave on this desk |
| **2.13.1** LPUART transmitter jitter, kernel/baud a non-integer in 3..4 (P) | `test_stm32_serial` letter `v`, whose LSE-kernel leg at 9600 baud is the sheet's own 32768 / 9600 = 3.41 | the band is a stated caveat in [../lpuart.md](../lpuart.md); letter `n`'s own rungs (ratios 555.6 and 6666.7) are outside it |
| **2.14.1 / 2.14.2** SPI BSY on disable, slave BSY (A) | `test_stm32_spi` letter `h`, a SELF-LINK letter | NOT STAGED on this board: the self-link is not wired here, so the letter skips. Both are the G0B1's 2.12.1 and 2.12.2 and are measured there |
| **2.8.6** TIM16/TIM17 clocked by SYSCLK rather than TIMPCLK (N) | - | UNOBSERVABLE by construction: this stratum pins HPRE and PPRE at 1, so the two clocks are one |
| **2.8.4 / 2.8.5** bidirectional break with short pulses, sync trigger missed with a faster master clock (N) | - | not reached: no letter sets BKBID, and one clock feeds every timer |
| **2.2.5** a flash location of all ones cannot be re-programmed to all zeros (N) | `test_stm32_nvm`, which stays the G0B1RE's | not staged here (no storage geometry on a single-bank part) |
| **2.2.3, 2.2.7, 2.2.9** RDP1 boot via PA14, PCROP weakness, option-byte lock | - | not staged: there is no option-byte write verb, PCROP is a read-only decode and RDP is one-way |
| **2.2.1, 2.2.2, 2.2.8, 2.2.10, 2.2.11, 2.10.2** LSI, WUFx, boot select, RTC domain, tamper flag on LSE failure | the same items as the G0B1's, applied the same way | unchanged; the LSE CSS stays declined, its only way back being the domain reset this bench must not take |
| **2.3.1** a GPIO assigned to a DAC channel cannot be an output while that channel is on-chip only (N) | `test_stm32_analog`'s `DacMode::internal_unbuffered` leg | not reached: no letter drives PA4 or PA5 as a GPIO output while such a mode holds |
| **2.7.2** DAC DMA underrun flag missed with concurrent SW+HW triggers (N) | `test_stm32_analog` letter `p` | not reached: the condition needs software and hardware triggers used together and this letter uses one at a time |
| **2.5.4, 2.6.1, 2.6.3, 2.6.5** and the remaining shared items | the G0B1's twins, applied where they are | unchanged |

**ES0418 HAS NO PREFETCH ITEM** - the G0B1's 2.2.10 (prefetch failure
branching across flash memory banks) has no twin here, and it could not:
this part has ONE bank. `FlashAccel::prefetch` stays a verb left at its
reset value on both parts, because the reason to leave it there is the
G0B1's and a common default is worth more than a per-part one.

**AND ONE PER-PART FACT THAT IS NOT AN ERRATUM AT ALL**, found the hard
way and now in the reserve: RM0444 22.4.25's ETRSEL list footnotes codes
0100 (MCO), 0101 (MCO2) and 0110 (COMP3) "available on STM32G0B1xx and
STM32G0C1xx sales types only". No TIM register differs between the
headers, so a timer told to take MCO on a smaller part counts nothing, in
silence - which wedged a suite whose own microsecond wait rode that timer
([../tim.md](../tim.md), [../clock.md](../clock.md)).

## The third silicon: STM32G031K8, and a sheet that is not in hand

The Nucleo-G031K8 at desk position G reports **DEV_ID 0x466, REV_ID
0x1003**, and its errata sheet is **ES0487** - a number read off st.com's
own listing and nothing more, because THE DOCUMENT WAS NOT OBTAINED:
every fetch route timed out at the bench. So the rule is simple, and it
is visible in the suites' own text: **no item
number of ES0487 is ever cited**. Where a letter stages a behaviour named
by a sheet that IS in hand it says whose - "the G0B1's 2.7.2, ES0487's
twin pending" - and what it records is what THIS DIE DID, with the
sheet's verdict left open.

What the letters found on it, all of it measured on REV_ID 0x1003:

| The behaviour staged (named by the G0B1's ES0548 item) | Where a letter reaches it | What this die did | ES0487's own verdict |
|---|---|---|---|
| **2.7.2** the second of two adjacent compare matches | `test_stm32_tim` letter `k`, staged with a control | did NOT reproduce: the second compare raised its flag and toggled its output all eight rounds - as on the other two dies | pending |
| **2.7.1, 2.7.3** the one-pulse trigger, output-compare clear | - | not staged: 2.7.1 needs a trigger placed at CNT = ARR of a cascaded master, and 2.7.3 needs `ocref_clr`, which on this part has no comparator to come from at all | pending |
| **2.2.4** peripherals that request HSI16 fail to wake from Stop with HSIDIV != 0 | `test_stm32_sleep` letter `g`, with its control | the hazard predicate is quiet at HSIDIV 0 and speaks as soon as the divider moves, and an RTC wake is NOT reached by it: a Stop armed for 250 ms under HSIDIV /4 lasted its time. The USART half has no subject here - this part's USART2 cannot wake from Stop at all | pending |
| **2.2.2** a spurious wake-up flag | `test_stm32_sleep` letter `h` | unchanged: `wakeup_pin()` clears WUFx as part of the configuration, so the flag cannot reach a caller | pending |
| **2.9.1** consecutive initialization-mode entries corrupt the calendar | `test_stm32_rtc` letter `h`, guarded against raw | did not reproduce on the unguarded path in this run either; the guarded path never corrupts a calendar | pending |
| **2.8.1, 2.8.2** the LPTIM's disable, and a flag cleared from thread mode | `test_stm32_lptim`, as code | unchanged and measured as code: a thread-mode clear with an interrupt enabled is refused, `disable()` IS the RCC reset | pending |
| **2.10.1** the I2C's minimum kernel clock per speed | `test_stm32_i2c` letter `m` | unchanged: the erratum's floors (4, 10 and 20 MHz) are stricter than the datasheet's in all three modes, and the driver refuses below them | pending |
| **2.10.2** a master's spurious BERR | `test_stm32_i2c` letter `m` | not seen: BERR swept 0 times since the host's init, and the driver counts it and never reports it - the erratum's own workaround | pending |
| **2.12.1, 2.12.2** the SPI's BSY and its last frame | `test_stm32_spi` letter `h` | NOT REACHED: that letter's instrument is a wire, and this board has none (nor a second SPI whose pads the package bonds) | pending |
| **2.2.10** prefetch across flash banks | - | cannot apply: one bank | pending |
| the FDCAN and DAC items | - | cannot apply: this part has neither | pending |

**AND ONE THING THE SHEET'S ABSENCE DOES NOT EXCUSE.** Two facts of this
board were found the hard way and belong beside the errata because they
look like them until they are named: **Shutdown powers the LSI down**
(DS12992 3.7.4 - it is in the datasheet, not in an errata sheet, and it
means an RTC on LSI cannot end a Shutdown), and **the debug port does not
attach** (`docs/bench.md`, a desk fault under investigation and not a
silicon claim).
