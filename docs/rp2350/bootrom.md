# The bootrom's function table (RP2350)

This document is for a programmer who wants something the RP2350's mask
ROM offers a running program: the 128-bit random number it made at
boot (a cheap seed), the chip's identity and boot state as the ROM
reports them, or one of its public functions that brio does not wrap.
`Bootrom` is the driver, a monostate (a type with no objects, all of
whose functions are static, because the ROM exists once): it finds the
ROM's entry points through the ROM's own table, the right way for the
architecture the image runs on, and wraps the two read-only calls this
stratum needs. Both architectures run it from one source.

The smallest use, an excerpt of the block under "How to use it" below:

```cpp
if (brio::Bootrom::present()) {
    if (auto info = brio::Bootrom::sys_info()) {
        if (info->has(brio::SysInfoFlag::boot_random)) {
            seed(info->boot_random);      // 128 bits the ROM made at boot
        }
        const uint64_t id = info->device_id;
    }
}
```

Line by line: `present()` checks the three magic bytes at 0x10 that make
the ROM's other fixed words trustworthy; `sys_info()` looks up the ROM's
`get_sys_info`, calls it for every item this driver can decode and sorts
the answer into a `BootromSysInfo`, or answers nothing when the call
failed; `has(flag)` asks whether the ROM actually served that item,
since what a caller asks for and what the ROM supports are not the same
set; `boot_random` is the four words of the per-boot random number and
`device_id` the 64-bit identity, the ROM's two answer words with the
first as the low half. It covers the same OTP rows as [otp.md](otp.md)'s
`chip_id()`, but not in the same order: `chip_id()` assembles CHIPID0..3
least significant row first, and `device_id` holds those rows with its
two 32-bit halves the other way round (otp.md's "Bench findings").

What decides correctness even there. The served word comes first: the
answer's words carry no tags and follow in flag order, so a field is
meaningful only where `has()` says so, and a call that serves less than
was asked for is not a failure. The calls run ROM code on the caller's
stack and in its context, with nothing here masking interrupts: they
belong to main context and never to an interrupt handler, and on the
RISC-V half 5.4.8 warns that some ROM functions want a good deal of
stack. Every lookup answers nullptr, every raw wrapper
`BootromError::not_found` and `sys_info()` and `git_revision()` nothing
(`std::nullopt`), rather than jumping to address zero when the table is
not where it should be.

The rest of this page, in order: the documents of record; what the
silicon does (the fixed words and the two sets of them, the two
lookups, the security state, the codes, `get_sys_info`'s answer,
`get_partition_table_info`, the boot locks); what is deliberately not
wrapped; the types and verbs; how to use it; the bench findings; and
what is not covered.

## Documents of record

The RP2350 datasheet (build d126e9e), 5.4 (the
bootrom APIs: 5.4.1 locating them, with table 453 for Arm and table 454
for RISC-V, 5.4.2 which architectures and security states each entry
point serves, 5.4.3 the return codes, 5.4.4 the boot locks, 5.4.6 and
5.4.7 the lists, 5.4.8.16 `get_partition_table_info`, 5.4.8.17
`get_sys_info`, 5.4.8.19 `git_revision`, 5.4.8.21 `otp_access`), 3.9 (the
architecture select the ROM acts on), 12.12.4.1 (what the ROM does with
the entropy source at boot). The ROM's own ABI constants - the lookup
flags and the two-character codes - are published in the pico-sdk's
`boot/bootrom_constants.h` at tag 2.3.1, which is the vendored device
description this stratum reads its register names from. The driver:
`brio/rp2350/bootrom.hpp`. The reference suite: `test_rp2350_blocks`,
letters a and k, which runs on both of this chip's architectures from one
source (a letter is one command key of the suite's console and the group
of verdicts it runs; `z` is the key that runs them all).

## What the silicon does

32 kB of mask ROM at address zero, which booted this image and is still
there. Beside the boot code it carries public entry points, and 5.4.1
says plainly why they are found through a table rather than called at a
fixed address: their locations "may change with each bootrom release".

**What is fixed is a handful of well-known words**, and there are two
sets of them:

| Address | Contents | For |
|---------|----------|-----|
| `0x10` | `'M'`, `'u'`, `0x02` - the magic that makes the rest trustworthy | both |
| `0x13` | the ROM version byte, 2 on A2 silicon; informational only | both |
| `0x14` / `0x16` / `0x18` | the table, and the two lookup helpers | Arm |
| `0x7df6` / `0x7df8` / `0x7dfa` | the same three, near the top of the ROM | RISC-V |
| `0x7dfc` | the ROM's RISC-V entry point | RISC-V |

Each pointer cell is a halfword, the ROM being small enough that every
address in it fits in sixteen bits.

**The two architectures are two lookups, not one.** On Arm the table
holds a pointer to the function, so the helper that returns the stored
value is the right one; on RISC-V the table entry is a jump instruction,
so the helper that returns the entry's own address is. And the flag
differs: a Secure Arm entry point, or the RISC-V one. This driver asks
`core_kind` - a constant, not the preprocessor - and does the right one,
which is the only place in the stratum outside `core.hpp` where the
architecture shows at all.

**Security state.** brio runs Secure on the Arm half, because the bootrom
hands over that way and nothing here writes the SAU. The Non-secure entry
points are not offered: 5.4.2 says each must be enabled individually by
Secure code before it exists, and all of them are disabled initially.

**A function is asked for by a two-character code** ('G','S' for
`get_sys_info`), packed as `c1 | c2 << 8`.

**`get_sys_info` answers with a buffer whose first word is the set of
flags the ROM actually served** (5.4.8.17), and the words after it follow
in flag order with no tags - so a caller must check that word before
reading any of the rest. What it can bring back: the package select bit
and the 64-bit device id (which is OTP's CHIPID rows, read the other way
round), the OTP CRITICAL register, the architecture of the calling core,
the flash device info, the 128-bit per-boot random number the ROM made by
streaming raw entropy through the SHA-256 accelerator (12.12.4.1), and
four words about how this boot went. NONCE is listed and is not supported
on this silicon.

**`get_partition_table_info`** (5.4.8.16) reports
`BOOTROM_ERROR_PRECONDITION_NOT_MET` when no partition table has been
loaded - which, measured, is not what an image that simply carries no
table gets: a flash boot leaves the ROM's resident view loaded whether
there is anything in it or not, so the call is served and answers with a
table that is not present. The precondition code belongs to the boot
paths that skip that step, a watchdog boot or a RAM boot.

**The boot locks are off by default** (5.4.4). The ROM checks them only
when boot lock 7 has been claimed to turn checking on; the SDK claims it,
brio does not, and the two functions wrapped here own no hardware, so
there is nothing to arbitrate.

**Nothing here is called from an interrupt.** These are ordinary C
functions in ROM with ordinary stack needs, and 5.4.8 warns that some
want a good deal of it on RISC-V.

**The fixed words are in the first page of the address space**, which gcc
takes for the null page and refuses to read through a constant - the same
trap the RP2040's flash driver met at the same addresses, answered the
same way: an address the optimizer cannot fold.

## What is deliberately not wrapped

- **The flash functions.** They disconnect the QSPI interface from the
  execute-in-place window, so they must run from RAM with interrupts
  masked; that discipline is a chapter of its own and belongs beside the
  flash driver.
- **`reboot()`.** This stratum's reboot is the watchdog's
  ([reset.md](reset.md)), which is what the reset chapter measured. A
  second spelling of the same act would be a second thing to keep true.
- **`otp_access()`** (5.4.8.21). It is the ROM's OTP read AND WRITE entry
  point: its `row_and_flags` argument carries an IS_WRITE bit. This
  project's rule is that no write path to OTP exists in the tree
  ([otp.md](otp.md)), and the read side is memory-mapped and needs no ROM
  call - so there is nothing lost and one irreversible mistake made
  impossible. The family check asserts that the wrapper is absent.

## Types and verbs

- `Bootrom::present()` - the three magic bytes at `0x10`.
  `Bootrom::version()` - the version byte.
- `Bootrom::lookup_function(code)` / `lookup_data(code)` - one entry
  point or one ROM data location, or nullptr. The architecture decides
  which helper is asked and with which flag; `Bootrom::function_flag` is
  that constant, and `table()`, `lookup_value()` and `lookup_entry()` are
  the pieces underneath for a program that wants to report them.
- `Bootrom::git_revision()` - the ROM's own revision word.
- `Bootrom::get_sys_info(out, words, flags)` and
  `get_partition_table_info(out, words, flags_and_partition)` - the two
  calls, raw, answering the ROM's own codes and
  `BootromError::not_found` when the function is not in the table at all.
- `Bootrom::sys_info(flags)` - `get_sys_info` decoded into a
  `BootromSysInfo`: the served word, the package select, the device id,
  the critical word, the CPU, the flash device info, the boot random and
  the boot info, with `has(flag)` for each.
- The constants: `BootromAddress` (the fixed words of both tables),
  `bootrom_magic`, `bootrom_code(c1, c2)` and `BootromCode`,
  `BootromFlag`, `BootromError` (5.4.3's codes), `SysInfoFlag` and
  `PartitionInfoFlag`.

## How to use it

```cpp
if (brio::Bootrom::present()) {
    if (auto info = brio::Bootrom::sys_info()) {
        if (info->has(brio::SysInfoFlag::boot_random)) {
            seed(info->boot_random);      // 128 bits the ROM made at boot
        }
        const uint64_t id = info->device_id;
    }
}

// or one entry point, by its code, for a call this file does not wrap
using Fn = int32_t (*)(uint32_t);
if (auto* p = brio::Bootrom::lookup_function(brio::bootrom_code('G', 'B'))) {
    const int32_t b = reinterpret_cast<Fn>(p)(partition_a);
}
```

## Bench findings

On an RP2350 in the QFN-80 package, **stepping A2**, clk_sys at 150 MHz,
on BOTH architectures: `test_rp2350_blocks` reports **76 pass, 0 fail**
on the Cortex-M33 pair and on the Hazard3 pair, from one source, each on
three flash-and-run cycles. Two numbers of this chapter differ between
the halves, and both must.

- **THE TABLE IS AT 0x7CD4 ON BOTH HALVES, AND EVERY ADDRESS IN IT
  DIFFERS.** The three magic bytes stand at 0x10, the version byte reads
  **2** (A2 silicon) and the git revision word is present and not blank.
  The two lookup helpers and the two functions resolve out of the
  architecture's OWN entry: on the Cortex-M33 half `lookup_val` 0x75,
  `lookup_entry` 0x35, `get_sys_info` 0x9C1, `get_partition_table_info`
  0xACD; on the Hazard3 half 0x75FE, 0x75C2, 0x7CE0 and 0x7CFC. Both sets
  are inside the 32 kB ROM, and no address is shared between them - one
  table, an entry per architecture, and the driver asks with the flag of
  the half it is running on.
- **CPU_INFO READS 0 UNDER AN ARM IMAGE AND 1 UNDER A RISC-V ONE.** It is
  the ROM's own answer to the question [otp.md](otp.md)'s ARCHSEL_STATUS
  answers from the other side, and the two agree on every run.
- **`get_sys_info` SERVES EVERY FLAG THIS DRIVER ASKS FOR**, returning
  0x5F - the chip info, the critical word, the per-boot random, the boot
  info, the flash device info. Cross-checked against two other blocks:
  the device id it reports IS OTP's CHIPID rows (0x9026BDD4 0xF5D324A1),
  and the critical word IS the OTP block's own CRITICAL register (0x0).
  `package_sel` reads 0 and `flash_devinfo` 0xC00 - the ROM's own default,
  OTP's FLASH_DEVINFO row being unwritten on this part.
- **THE PER-BOOT RANDOM NUMBER IS PRESENT AND CHANGES EVERY BOOT**: four
  different words on each of the six runs (0xF895BF1B..., 0x6440418E...,
  0x9D902B09..., 0xF3505A1C..., and so on). It is the TRNG's raw samples
  through the SHA-256 accelerator, both of which this suite also drives
  directly - so the cheap seed and the expensive one come from the same
  silicon.
- **`get_partition_table_info` IS SERVED, NOT REFUSED, ON AN IMAGE WITH
  NO PARTITION TABLE.** 5.4.8.16 says a call made before the table has
  been loaded answers `BOOTROM_ERROR_PRECONDITION_NOT_MET`; asked for
  PT_INFO after an ordinary flash boot it returns **4** - the count of
  words it filled - with the words **0x1, 0x0, 0xFFFFE000, 0xFC078000**.
  Word 0 is the subset of the asked flags the API supports (PT_INFO), and
  word 1 is zero: no partition table present, no partitions. A flash boot
  therefore leaves the ROM's resident view loaded even when there is
  nothing in it, and the precondition code belongs to the boot paths that
  do not load it - a watchdog or a RAM boot. The boot info words read
  0xFE00FE and 0x500D.
- **A CODE THE TABLE DOES NOT CARRY** (letter k, fifteen runs on each
  half): the made-up 'QZ' looks up as nullptr on both helpers, the
  function's and the data's - the state every wrapper here answers
  with `BootromError::not_found` (-17) instead of calling through.

## Not covered yet

Driver gaps, each with its reason:

- **The flash functions, `reboot()` and `otp_access()`**: the section
  above says why each.
- **`load_partition_table()`, `pick_ab_partition()`, `chain_image()`,
  `explicit_buy()` and the rest of the secure-boot and A/B update
  family**: they want a program that HAS a partition table, a work area
  of three kilobytes and something to say about versions. Born with the
  first image on this target that is built as a signed or an updatable
  one.
- **The Non-secure entry points and `set_ns_api_permission()`**: brio
  runs everything Secure on this chip, so the entry points do not exist
  for it. Born with a program that partitions the address space.
- **The boot locks** (5.4.4): nothing here claims lock 7, so the ROM
  does no checking, and the two calls wrapped own no hardware. Born with
  the flash functions, which is where the arbitration is real.
- **`partition_table_ptr` and the other ROM DATA locations**:
  `lookup_data()` reaches any of them by code; only the git revision has
  a named verb, because only it is meaningful with no partition table.

Implemented but not bench-verified, each with what would measure it:

- **The precondition path of `get_partition_table_info`.** It is reached
  by a boot that does NOT load the partition table - a watchdog boot or a
  RAM boot - and every image here is a flash boot, where the call is
  served. A letter that rebooted through the watchdog and asked again
  would measure it, and belongs with the reset chapter rather than here.
