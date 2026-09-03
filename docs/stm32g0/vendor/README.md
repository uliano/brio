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
| STM32G071x8/xB device errata | ES0418 Rev 5 (November 2023) | symlink to `.../es0418-*.pdf` | the SECOND-SILICON board's (a Nucleo-G071RB in the drawer); not the bench chip's |
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
  includes (ARM CMSIS_5 5.9.0), shared with the samc stratum.
- `stm32g0/svd/STM32G0B1.svd` - ST's SVD for the debug Peripheral
  Viewer (Apache-2.0, from the cmsis-svd-data mirror of ST's pack),
  with its description text SANITIZED TO ASCII (the repo rule): ST's
  file carries double-encoded non-breaking spaces and curly quotes,
  mapped to plain ones, and a few symbols (micro, greater-or-equal)
  replaced by `?`. Register data untouched, the XML re-parsed.

## The bench chip

STM32G0B1RE on an ST Nucleo-G0B1RE (MB1360). DBGMCU_IDCODE reads
**0x10016467** over SWD: DEV_ID 0x467 = STM32G0B1/G0C1, REV_ID 0x1001
= **silicon revision Z** (ES0548 table 2, the newer of the two). The
flash size register (0x1FFF75E0) reads 0x200 = 512 KB; the 96-bit UID
is recorded in `tools/bench_boards.py`. Read the IDCODE at bring-up of
any new board - the errata columns are per revision.

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
(TIM), 2.9.1 (RTC), 2.10.x (I2C), 2.12.x (SPI), 2.13.x
(FDCAN). Revision A only, absent on Z: 2.2.5, 2.2.7, 2.2.9, 2.6.5.

**No item of ES0548 touches the CRC calculation unit** - a statement
about the document, not a claim about the silicon (crc.md).
