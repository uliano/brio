# Vendor documents

The reference manuals, datasheets and errata the `stm32f4/` stratum is
written against (every target folder has its own `vendor/`). They are
NOT in the repository (`docs/*/vendor/*.pdf` is git-ignored): keep the
files listed below here (downloaded or symlinked), and cite them from
code and docs by document number and SECTION, never by page ("RM0090
7.3.2"): sections survive revisions, pages do not.

Check the revision before trusting a local copy: ST republishes all of
them, and the errata sheet is PER PART CLASS - the F429's is ES0206
(shared with the F427/F437/F439), the F446's is ES0298, the F411's is
ES0287 - and the number of a part's errata sheet is never guessable
from a neighbour's (verify on st.com before citing).

## STM32F4 (`brio/stm32f4/`)

Four parts on the bench, FOUR reference manuals: one stratum, because
the device headers (cmsis-device-f4) put the same IP under the same
register names on all of them and the umbrella `stm32f4xx.h` dispatches
on a define; where the manuals differ (the frequency ladders, the
instance sets, the regulator's bits) the difference is a fact of the
part class, keyed in `stm32f4/device_tables.hpp`.

| Document | Number / revision of record | File name here | Notes |
|----------|-----------------------------|----------------|-------|
| STM32F405/415, F407/417, F427/437, F429/439 reference manual | **RM0090 Rev 22** | symlink to `~/Documenti/Elettronica/STM32/STM32F4/STM32F407_429_Reference_Manual.pdf` | the F429's manual and the F405 class's in one document, with separate RCC and PWR chapters for the two classes (ch. 6 and 7, 5.4 and 5.5): a citation names the class's chapter |
| STM32F446xx reference manual | **RM0390 Rev 6** | symlink to `.../STM32F446_Reference_Manual.pdf` | the F446's; the same peripherals under the same names, the F446's own additions (QUADSPI, SAI2, SPDIFRX, FMPI2C, CEC) |
| STM32F411xC/xE reference manual | **RM0383 Rev 4** | symlink to `.../STM32F411_Reference_Manual.pdf` | the F411's: the 100 MHz ladder, no over-drive, one ADC, no DAC/CAN |
| STM32F469xx/F479xx reference manual | **RM0386 Rev 6** | symlink to `.../rm0386-*.pdf` | the F469/F479's: the F42x/F43x's frequency ladder row for row (3.3.2 table 9, 6.3.3), the same DMA request mapping with QUADSPI added (tables 29 and 30), a DSI host beside the LTDC, the 48 MHz domain switchable to PLLSAI's P output (DCKCFGR.CK48MSEL); its chapters are numbered differently from RM0090's (FLASH 3, PWR 5, RCC 6, DMA 9, FMC 12, ADC 14, DAC 15, LTDC 17, DSI 18, RTC 28, SPI/I2S 31, bxCAN 34, USB OTG 35) |
| STM32F427xx/F437xx, F429xx/F439xx datasheet | **DocID024030 Rev 10** | symlink to `.../STM32F429_Data_Sheet.pdf` | pinout and the alternate-function table 12 (the AF numbers no header carries), electrical characteristics |
| STM32F446xC/xE datasheet | **DS10693 Rev 11** | symlink to `.../STM32F446_Datasheet.pdf` | alternate-function table 11 |
| STM32F411xC/xE datasheet | **DS10314 Rev 8** | symlink to `.../STM32F411_Data_Sheet.pdf` | alternate-function table 9 |
| STM32F469xx datasheet | **DS11189 Rev 8** | symlink to `.../DS_stm32f469ae.pdf` | alternate-function table 12; the 2 MB / 320 + 64 KB memories |
| STM32F427/437 and F429/439 device errata | **ES0206 Rev 24** | symlink to `.../es0206-*.pdf` | silicon revisions A, Y, 1, 3, 4/5/B by REV_ID (table 2); the STM32F429I-DISC1's part reads REV_ID 0x2003 = markings 4, 5 and B |
| STM32F446xC/xE device errata | **ES0298 Rev 8** | symlink to `.../es0298-*.pdf` | revisions A and 1 share REV_ID 0x1000 (table 2); the Nucleo-F446RE's part reads it |
| STM32F411xC/xE device errata | **ES0287 Rev 6** | symlink to `.../es0287-*.pdf` | revisions A, 1 and 2 share REV_ID 0x1000; the black pill's part reads it |
| STM32F469xx/F479xx device errata | **ES0321 Rev 14** | symlink to `.../es0321-*.pdf` | revisions A and 1 share REV_ID 0x1000 (table 2); the 32F469IDISCOVERY's part reads it; its bxCAN item is 2.15.1, its RTC items 2.11.x, its OTG_FS items 2.16.x |
| STM32 Cortex-M4 programming manual | **PM0214 Rev 10** | symlink to `~/Documenti/Elettronica/STM32/pm0214-*.pdf` | the core: PRIMASK/BASEPRI, SysTick, the NVIC, the FPU's CPACR and lazy stacking, the ISB the pending-interrupt test needs |
| ARMv7-M Architecture Reference Manual | ARM DDI 0403E.e | symlink to `~/Documenti/Elettronica/STM32/ARM-CORTEX References/DDI0403E_e_armv7m_arm.pdf` | the architecture behind PM0214 |
| Cortex-M4 Devices Generic User Guide | ARM DUI 0553B | symlink to `.../DUI0553.pdf` | |
| Introduction to system memory boot mode | AN2606 Rev 70 | symlink to `~/Documenti/Elettronica/STM32/an2606-*.pdf` | the ROM bootloader (DFU through BOOT0), the CLI's future fourth flash mechanism |
| STM32F429I-DISC1 user manual | UM1670 Rev 6 | symlink to `.../STM32F429I-DISC1/um1670-*.pdf` | + the MB1075 schematics B01/C01/D02/E01 beside it |
| STM32 Nucleo-64 boards (MB1136) user manual | UM1724 Rev 17 | symlink to `~/Documenti/Elettronica/STM32/ST Boards/um1724-*.pdf` | + the MB1136 schematics C03/C04 |
| 32F469IDISCOVERY user manual | UM1932 Rev 5 | symlink to `.../STM32F469I-DISCO/um1932-*.pdf` | + the MB1189 schematics B-07/B-08/C-01 (the MCU sheet is where the SDRAM's pads come from) and the display board's MB1166 A-08/A-09/A-10 beside it |

The black pill's own documents (WeAct's schematic) are the board
page's business ([../boards/blackpill-f411ce.md](../boards/blackpill-f411ce.md)).

Canonical URLs (redirect to the current revision):
`https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf`,
`https://www.st.com/resource/en/errata_sheet/es0206-stm32f427437-and-stm32f429439-device-errata-stmicroelectronics.pdf`.

## The manuals that were NOT read, and what that means

The pack carries twenty-three device headers and the stratum compiles
on all of them (`brio check stm32f4`), but the frequency ladders - how
fast a part may run at each regulator scale, how many flash wait
states a rate needs, how fast each APB may go - are facts of a
REFERENCE MANUAL that the header does not carry, and only five part
classes' manuals are on the desk (RM0090 for the F405 and F42x/F43x
classes, RM0390, RM0383, RM0386). On the F401, F410, F412 and
F413/F423 headers the reserve says so (`sysclk_ladder().known ==
false`) and a `Clock` above the 16 MHz reset rate is refused at compile
time; RM0368, RM0401, RM0402 and RM0430 would each add one row to
`stm32f4/device_tables.hpp`'s ladder when read.

## Device headers + SVD: vendored in the repo

arm-none-eabi-gcc ships no device headers, so the CMSIS device headers
ARE in the repository (Apache-2.0 allows it) so a fresh clone builds:

- `third_party/cmsis-device-f4/` - the `Include/` tree of
  STMicroelectronics' **cmsis-device-f4 v2.6.9** (every STM32F4 part:
  `stm32f4xx.h` is the umbrella that dispatches on `-DSTM32F429xx`,
  `stm32f429xx.h`, `stm32f446xx.h`, `stm32f411xe.h` and `stm32f469xx.h`
  the headers of record for register names on the bench chips). When the reference
  manual and the header disagree on a name, the header wins in code and
  the manual's section is quoted in the comment. ST's startup templates
  are NOT vendored; the crt cites them for the handler NAMES only
  (`stm32f4/src/glue/`).
- `third_party/cmsis-core/` - the CMSIS-Core headers the device header
  includes (ARM CMSIS_5 5.9.0), shared with the other ARM strata;
  `core_cm4.h` joined the vendored set for this family.
- `stm32f4/svd/STM32F429.svd`, `STM32F446.svd`, `STM32F411.svd`,
  `STM32F469.svd` - ST's SVDs for the debug Peripheral Viewer, one per
  bench part (Apache-2.0, from the cmsis-svd-data mirror of ST's pack).
  ASCII as shipped, every file re-parsed as XML: 92, 78, 55 and 94
  peripherals.
