# Vendor documents

The datasheets and errata the `avrdx/` stratum is written against
(every target folder has its own `vendor/`). They are NOT in the
repository (Microchip's documents are not redistributable, and they
are large): `docs/*/vendor/*.pdf` is git-ignored. Keep the files listed below here (downloaded or
symlinked), and cite them from code and docs by document number and
SECTION, never by page ("DS40002247B 16.5.2"): sections survive
revisions, pages do not.

Check the revision before trusting a local copy: Microchip republishes
both datasheets and errata (the errata grow with each silicon
revision), and file names in the wild are unreliable (a file named
"AVR64DB...Errata" may in fact be the AVR128DB errata). The revision
letter is in the first-page footer (`DS40002247B`, `DS80000915F`).

## AVR DA/DB (`lib/brio/src/avrdx/`)

| Document | Number / revision of record | File name here | Notes |
|----------|-----------------------------|----------------|-------|
| AVR128DB28/32/48/64 Data Sheet | **DS40002247B** (2023, complete, 730 pp) | `AVR128DB28-32-48-64-DataSheet-DS40002247.pdf` | chapters: 16 EVSYS, 17 PORTMUX, 18 PORT, 21 VREF, 33 ADC, 34 DAC. Supersedes the 2020 preliminary rev. A (660 pp) that older local copies are |
| AVR128DB28/32/48/64 Silicon Errata and Data Sheet Clarifications | **DS80000915F** (2025-04, 27 pp; silicon rev. A4/A5/B0) | `AVR128DB28-32-48-64-SilConErrataClarif-DS80000915.pdf` | items for ADC (MUX/accumulation delayed update), DAC (output buffer lifetime drift), CLKCTRL, PORT, SPI, TCA, TCB, TWI, USART - read before each driver. Rev. A (13 pp, 2020) is what older local copies are: incomplete |
| AVR128DA28/32/48/64 Data Sheet | DS40002183A (2020, preliminary) | `AVR128DA28-32-48-64-DataSheet-DS40002183A.pdf` (symlink) | the DA sibling; a newer revision likely exists - check before relying on it |
| AVR128DA28/32/48/64 Silicon Errata | DS80000882C | `AVR128DA28-32-48-64-SilConErrataClarif-DS80000882C.pdf` (symlink) | check for newer |
| Technical briefs TB3209 ADC, TB3213 RTC, TB3214 TCB, TB3217 TCA, TB3218 CCL, TB3212 TCD, TB3211 AC | DS9000320x | not copied here (in ~/Documenti/Elettronica/AVR) | Microchip "Getting started" briefs for megaAVR-0/Dx peripherals: useful complements, not authoritative |
| AN2434 Interfacing Quadrature Encoder using CCL with TCA and TCB | DS00002434C | not copied (`https://ww1.microchip.com/downloads/en/Appnotes/Interf-Quad-Encoder-CCL-w-TCA-TCB-DS00002434C.pdf`) | the one Microchip AN that composes CCL + TCA + TCB on this family; a model for the timer tasks |
| AN2451 Getting Started with Core Independent Peripherals on AVR | DS00002451 | not copied (`https://www.microchip.com/DS00002451`) | EVSYS/CCL/timer composition patterns; background only |
| Microchip examples (github.com/microchip-pic-avr-examples): `avr128da48-getting-started-with-tcb-*`, `avr128da48-tcb-frequency-dutycycle-measurement-*` (TCB FRQPW + PIT, 500 Hz..200 kHz), `avr128db48-blink-led-ccl-mplab-mcc` (CCL JK flip-flop toggled by a timer event) | - | online | register-level recipes to compare our drivers against; MCC-generated, not authoritative |

Canonical URLs (redirect to the current revision):
`https://www.microchip.com/DS40002247` (datasheet),
`https://www.microchip.com/DS80000915` (errata). Direct file URLs:
`https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/AVR128DB28-32-48-64-DataSheet-DS40002247.pdf`,
`https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/Errata/AVR128DB28-32-48-64-SilConErrataClarif-DS80000915.pdf`.

## Errata DS80000915F: what touches brio

Applies to ALL silicon revisions (A4/A5/B0) unless noted:
- **2.2.4 Write operation lost on consecutive writes** - an ST/STD/STS
  to an address >= 64 immediately followed by an ST/STD to an address
  < 64 (VPORT/GPIOR space) or by a write to `SLPCTRL.CTRLA` loses the
  last write. brio: `Pin` uses SBI/CBI (I/O instructions, not ST) for
  VPORT, so bare-pin code is unaffected; `AvrPlatform::idle()` writes
  `SLPCTRL.CTRLA` with the documented NOP before each write (see the
  comment there). Any future driver storing to VPORT with ST must
  respect it.
- **2.3.2 ADC MUX/accumulation delayed update** with `INITDLY != 0`:
  set MUXPOS/MUXNEG/CTRLB before enabling or changing the reference,
  or do a dummy conversion - a rule for the ADC driver.
- **2.15.2 TWI Flush non-functional** - `Twi` must never use MCTRLB
  FLUSH; recover by ENABLE off/on (it does not use FLUSH today).
- **2.16.3 USART receiver dead after ISFIF** - only in auto-baud
  modes, which `Uart` does not use.
- 2.9.1 PD0 input buffer floating (28/32-pin only) - not our package.
- A4/A5 only (not B0): 2.3.1 ADC single-ended offset (-3 mV typ.),
  2.6.1 **DAC output buffer lifetime drift** (keep OUTEN on, or
  calibrate against the ADC - a rule for the DAC driver), 2.5.x
  CLKCTRL: EXTS/status bit not set for external sources (A4), PLL
  items, RUNSTDBY with external clock (A4); TCA restart resets
  direction, TCB CCMP/CNT 16-bit in 8-bit PWM, SPI1 alt2 on 48 pins,
  TWI output override, USART open-drain, CCL whole-module disable.
The device revision is readable at run time (`SYSCFG.REVID`, MAJOR
0x01 = A, 0x02 = B): worth printing in a console banner once.

Header of record for register names: the toolchain's
`avr/ioavr128db48.h` (avr-libc); when the datasheet and the header
disagree on a name, the header wins in code and the datasheet section
is quoted in the comment.
