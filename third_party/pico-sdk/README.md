# pico-sdk (vendored subset)

Three things out of Raspberry Pi's **pico-sdk** at tag **2.3.1** (commit
079c6f3, https://github.com/raspberrypi/pico-sdk, BSD-3-Clause - see
LICENSE.TXT), the RP2040's device description in the form the
`brio/rp2040/` stratum consumes:

- `CMSIS/RP2040.h` and `CMSIS/system_RP2040.h` - the CMSIS device
  header generated from the RP2040's SVD: the register blocks as
  `*_Type` structs, the instance pointers (`UART0`, `SIO`, `RESETS`,
  ...), the `IRQn_Type` enumerators, the core's revision and priority
  width, and the include of `core_cm0plus.h` (from
  `third_party/cmsis-core/`). The stratum's `armv6m/` core files are
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

NOT vendored: the SDK's runtime (`hardware_*`, `pico_*` libraries, the
`hardware/structs/` typedefs), TinyUSB, the crt and the linker
scripts. brio writes its own crt (`rp2040/src/glue/`) and linker
script; the second-stage bootloaders in `rp2040/src/glue/boot2_*.S`
are the SDK's `boot_stage2` sources assembled, padded and checksummed
once and checked in as bytes, each with its provenance in its header
comment. The handler NAMES the crt spells (`isr_uart0`, `isr_systick`,
...) are the SDK's own (`hardware/regs/intctrl.h`), the spelling every
Pico user and tool knows.
