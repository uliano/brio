# Vendor documents

The datasheets and errata the `samc/` stratum is written against
(every target folder has its own `vendor/`). They are NOT in the
repository (Microchip's documents are not redistributable, and they
are large): `docs/*/vendor/*.pdf` is git-ignored. Keep the files
listed below here (downloaded or symlinked), and cite them from code
and docs by document number and SECTION, never by page
("DS60001479M 28.6.2"): sections survive revisions, pages do not.

Check the revision before trusting a local copy: Microchip
republishes both datasheets and errata (the errata grow with each
silicon revision). The revision letter is in the first-page footer
(`DS60001479M`, `DS80000740S`).

## SAM C20/C21 (`brio/samc/`)

| Document | Number / revision of record | File name here | Notes |
|----------|-----------------------------|----------------|-------|
| SAM C20/C21 Family Data Sheet | **DS60001479M** (2025) | symlink to `~/Documenti/Elettronica/SAM/SAM-C20-C21-Family-Data-Sheet-DS60001479.pdf` | the whole family E/G/J/N x 15..18; chapters cited per driver |
| SAM C20/C21 Family Silicon Errata | **DS80000740S** (2026) | symlink to `~/Documenti/Elettronica/SAM/SAM-C20-C21-Family-Silicon-Errata-and-Data-Sheet-Clarification-DS80000740.pdf` | one doc for ALL silicon revisions B..H,N with a per-item revision matrix - always check the matrix against the chip's own DID.REVISION |
| SAM C21 Xplained Pro User Guide | Atmel-42460 | in ~/Documenti/Elettronica/SAM | the OFFICIAL eval board, NOT our board - loose cross-reference only |
| The user's C21J board pin plan | - | `~/Documenti/Elettronica/SAM/C21J.ods` | the board's own pin/function spreadsheet (KiCad project source of truth is elsewhere) |

Canonical URLs (redirect to the current revision):
`https://www.microchip.com/DS60001479` (datasheet),
`https://www.microchip.com/DS80000740` (errata).

## Device pack (headers + SVD): vendored in the repo

Unlike avr-gcc (which bundles the device headers), arm-none-eabi-gcc
ships none, so the CMSIS device headers ARE in the repository
(Apache-2.0 allows it) so a fresh clone builds:

- `third_party/samc21-dfp/` - the include/ tree of
  **Microchip.SAMC21_DFP 3.9.248** (all C21 E/G/J variants; the C21N
  subfamily is a separate tree in the pack, not vendored - no N board
  here). Header of record for register names: `samc21j18a.h` +
  `component/*.h`; when datasheet and header disagree on a name, the
  header wins in code and the datasheet section is quoted in the
  comment.
- `third_party/cmsis-core/` - the 5 CMSIS-Core headers the device
  header includes (ARM CMSIS_5 5.9.0: core_cm0plus, cmsis_compiler,
  cmsis_gcc, cmsis_version, mpu_armv7).
- `samc/svd/ATSAMC21J18A.svd` - from the same DFP, for the debug
  Peripheral Viewer.

Pack source: `https://packs.download.microchip.com/`
(`Microchip.SAMC21_DFP.3.9.248.atpack`).

## The bench chip

ATSAMC21J18A on the user's C21J rev 1.1 board. DSU DID reads
**0x11010500** (verified over SWD 2026-08-27, exact match with the
ATDF's declared value): DEVSEL 0x00 = SAMC21J18A, DIE.REVISION 5 =
**silicon rev F**. Read the DID at bring-up of any new board - the
errata matrix is per-revision.

## Errata DS80000740S: what touches the bring-up (silicon rev F)

Encoded in code (each with its comment citing the item):
- **1.2.2 OSC48M Division Ratio** - changing OSC48MDIV while the
  OSC48M is running but NOT requested by any GCLK generator leaves
  OSC48MSYNCBUSY.OSC48MDIV stuck. Clear ONDEMAND before touching DIV
  in that state (clock.hpp).
- **1.2.3 OSC48M Start-Up** - rare no-start at power-up or restart
  (parts produced before 2025-01). Keep the main oscillator
  ENABLE=1, ONDEMAND=0 (clock.hpp does, deliberately, with the cite);
  RUNSTDBY joins when the power pass arrives.
- **1.17.16 SERCOM Software Reset** - CTRLA.SWRST does nothing while
  CTRLA.ENABLE=0. The sercom.hpp release/recover discipline must
  never rely on SWRST from the disabled state.
- **1.10.4 DMAC Concurrent channels triggers** (the summary table
  files it under "Linked Descriptors") - concurrent channel triggers
  may corrupt WRITE-BACK descriptors; E/G/J at revisions E, F and H,
  so LIVE on this chip and positively observed at the bench. The
  workaround Microchip offers (single channel, linked descriptors)
  forbids concurrency itself, which a duplex serial port cannot
  honour - dmac.hpp instead validates every write-back reading
  against the loaded descriptor and refuses inconsistent ones
  (dmac.md carries the measurements).

NOT applicable to rev F (rev B..E items - do not code around them):
- 1.10.1 DMAC CRCDATAIN two-instruction hazard - rev B only (and the
  CRC engine is not built).
- 1.10.2 / 1.10.3 DMAC linked-descriptor items - for the E/G/J family
  these mark revisions B..D only. THE TRAP: each item's matrix also
  carries an N-family row, and it is the N row that is marked under E
  and F - read the row, not the column.
- 1.2.1 System Reset (the "do not exceed 4 MHz" cap) - rev B only.
- 1.8.1 Idle Sleep + FDPLL keeps AHB/APB running - rev B only (and
  no FDPLL in this design yet).
- 1.8.6 SYSTICK calibration value wrong - rev B only, and the ticker
  never reads CALIB (reload computed from Clock::hz).
- 1.8.12 OSC48M accuracy vs VDD - rev B..E.
- 1.17.2 auto-baud missing-stop-bit detection - rev B..E.

For the FUTURE passes (rev F affected; recorded here so the pass
that touches the module starts from them):
- **1.8.13 SysTick + standby back-bias hard fault** (STDBYCFG.BBIAS=1
  + SysTick interrupt coinciding with standby entry): the kernel tick
  IS SysTick - the power pass must disable the SysTick interrupt
  around standby entry or keep BBIAS=0.
- 1.8.14 Performance-mode regulator in standby (VREGSMOD=1 wrongly
  switches + keeps GCLK0 requested).
- 1.13.2/1.13.3 PORT PAC protection gaps; 1.13.4 PA28 cannot be a
  PORT event user.
- 1.16.x RTC read-sync items; the 1.17.x SPI/I2C items.
- NVMCTRL: NO errata at all (1.14.1 "Reserved").
