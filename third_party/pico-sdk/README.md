# pico-sdk (vendored subset)

Two chips' device descriptions out of Raspberry Pi's **pico-sdk** at tag
**2.3.1** (commit 079c6f3, https://github.com/raspberrypi/pico-sdk,
BSD-3-Clause - see LICENSE.TXT), each in the form its stratum consumes.
The two chips' files have THE SAME NAMES (`hardware/regs/clocks.h`
describes a different register block on each), so the RP2350's tree is
an include root of its own, `rp2350/`, and the RP2040's is this
directory: a build puts exactly one of them on the include path and
cannot mix them.

## The RP2040 (`brio/rp2040/`)

- `CMSIS/RP2040.h` and `CMSIS/system_RP2040.h` - the CMSIS device
  header generated from the RP2040's SVD: the register blocks as
  `*_Type` structs, the instance pointers (`UART0`, `SIO`, `RESETS`,
  ...), the `IRQn_Type` enumerators, the core's revision and priority
  width, and the include of `core_cm0plus.h` (from
  `third_party/cmsis-core/`). The stratum's `cortexm/` core files are
  written against exactly that contract. `system_RP2040.h` declares
  `SystemInit`/`SystemCoreClock` and nothing here defines or calls
  them: brio's clock has one truth, `Clock::hz`.
- `hardware/regs/*.h` and `hardware/platform_defs.h` - the bit-field
  definitions (`_BITS`, `_LSB`, `_VALUE_*`, `_OFFSET`, `_RESET`) of
  every register, generated from the same SVD, in the chapter's own
  names; the CMSIS header carries the structs and no field masks, so
  these are the other half. `platform_defs.h` defines the `_u()` the
  regs headers assume. `hardware/regs/addressmap.h` is vendored for
  completeness but NEVER included beside `RP2040.h`: both define the
  `*_BASE` macros, and the CMSIS header's are the ones the stratum
  uses.
- the SVD itself lives in `rp2040/svd/RP2040.svd` (the Peripheral
  Viewer's map), the same file the two headers were generated from.

## The RP2350 (`brio/rp2350/`)

`rp2350/` is the same three things for the second chip, and one file of
this project's own:

- `rp2350/CMSIS/RP2350.h` and `rp2350/CMSIS/system_RP2350.h` - the
  device header of the RP2350, whose `IRQn_Type` enumerators are the
  system interrupt numbers of BOTH architectures (datasheet 3.8.4.2:
  the numbering is shared), and whose include of `core_cm33.h` is what
  the Arm half resolves against `third_party/cmsis-core/`.
- `rp2350/hardware/regs/*.h` and `rp2350/hardware/platform_defs.h` -
  the field definitions of every register of this chip, `addressmap.h`
  among them and never included beside the CMSIS header, for the
  RP2040's reason.
- `rp2350/no_core/core_cm33.h` - NOT the SDK's: a stub of this
  project's own, the file's own header comment says why. The RISC-V
  build puts this directory first on the include path, so the device
  header's unconditional include of the Cortex-M33 core description
  resolves to nothing, and Hazard3 reads the same chip description the
  M33 does.
- the SVD is `rp2350/svd/RP2350.svd`.

## What is NOT vendored, for either chip

The SDK's runtime (`hardware_*`, `pico_*` libraries, the
`hardware/structs/` typedefs), TinyUSB, the crt and the linker
scripts. brio writes its own crt (`rp2040/src/glue/`,
`rp2350/src/glue/`) and linker script; the RP2040's second-stage
bootloaders in `rp2040/src/glue/boot2_*.S` are the SDK's `boot_stage2`
sources assembled, padded and checksummed once and checked in as bytes,
each with its provenance in its header comment, and the RP2350 needs no
such stage at all (datasheet 5.9.5: the bootrom sets XIP up itself).
The handler NAMES the crts spell (`isr_uart0`, `isr_systick`, ...) are
the SDK's own (`hardware/regs/intctrl.h`), the spelling every Pico user
and tool knows.
