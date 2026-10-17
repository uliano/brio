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

## AVR DA/DB (`brio/avrdx/`)

| Document | Number / revision of record | File name here | Notes |
|----------|-----------------------------|----------------|-------|
| AVR128DB28/32/48/64 Data Sheet | **DS40002247B** (2023, complete, 730 pp) | `AVR128DB28-32-48-64-DataSheet-DS40002247.pdf` | chapters: 11 NVMCTRL, 15 CPUINT, 16 EVSYS, 17 PORTMUX, 18 PORT, 21 VREF, 33 ADC, 34 DAC, 39.8 memory programming times. Supersedes the 2020 preliminary rev. A (660 pp) that older local copies are |
| AVR128DB28/32/48/64 Silicon Errata and Data Sheet Clarifications | **DS80000915F** (2025-04, 27 pp; silicon rev. A4/A5/B0) | `AVR128DB28-32-48-64-SilConErrataClarif-DS80000915.pdf` | items for ADC (MUX/accumulation delayed update), DAC (output buffer lifetime drift), CLKCTRL, NVMCTRL, PORT, SPI, TCA, TCB, TWI, USART - read before each driver. Rev. A (13 pp, 2020) is what older local copies are: incomplete |
| AVR128DA28/32/48/64 Data Sheet | DS40002183A (2020, preliminary) | `AVR128DA28-32-48-64-DataSheet-DS40002183A.pdf` (symlink) | the DA sibling; a newer revision likely exists - check before relying on it |
| AVR128DA28/32/48/64 Silicon Errata | DS80000882C | `AVR128DA28-32-48-64-SilConErrataClarif-DS80000882C.pdf` (symlink) | check for newer |
| Technical briefs TB3209 ADC, TB3213 RTC, TB3214 TCB, TB3217 TCA, TB3218 CCL, TB3212 TCD, TB3211 AC | DS9000320x | not copied here (in ~/Documenti/Elettronica/AVR) | Microchip "Getting started" briefs for megaAVR-0/Dx peripherals: useful complements, not authoritative |
| AN2434 Interfacing Quadrature Encoder using CCL with TCA and TCB | DS00002434C | not copied (`https://ww1.microchip.com/downloads/en/Appnotes/Interf-Quad-Encoder-CCL-w-TCA-TCB-DS00002434C.pdf`) | the one Microchip AN that composes CCL + TCA + TCB on this family; a model for the timer tasks |
| AN2451 Getting Started with Core Independent Peripherals on AVR | DS00002451 | not copied (`https://www.microchip.com/DS00002451`) | EVSYS/CCL/timer composition patterns; background only |
| Microchip examples (github.com/microchip-pic-avr-examples): `avr128da48-getting-started-with-tcb-*`, `avr128da48-tcb-frequency-dutycycle-measurement-*` (TCB FRQPW + PIT, 500 Hz..200 kHz), `avr128db48-blink-led-ccl-mplab-mcc` (CCL JK flip-flop toggled by a timer event) | - | online | register-level recipes to compare our drivers against; MCC-generated, not authoritative |

## AVR EA (the stratum's widening, not yet a build target)

| Document | Number / revision of record | File name here | Notes |
|----------|-----------------------------|----------------|-------|
| AVR64EA28/32/48 Data Sheet | **DS40002443A** (565 pp) | `AVR64EA-28-32-48-DataSheet-DS40002443.pdf` | the EA's own chapters where they differ from the DA/DB: CLKCTRL (OSCHF frequency chosen by the OSCCFG fuse, the main clock prescaler, XOSCHF, no PLL), NVMCTRL (NVM version 3, page-buffer writes), the 12-bit differential ADC with PGA, the fuses (SYSCFG0's UPDIPINCFG bit) |
| AVR64EA28/32/48 Silicon Errata and Data Sheet Clarifications | **DS80001048D** (2024, 17 pp; silicon rev. B1/B2) | `AVR64EA-28-32-48-SilConErrataClarif-DS80001048.pdf` | seven of its eight items apply to rev. B1 (NVM below 2.7 V, flash endurance below BODLEVEL3, partial CRC scan, multi-page erase from UPDI, self-programming with a flash read in progress, the USART's inconsistent sync field, consecutive writes to specific address spaces); B2 keeps three and adds the boot/code size fuse limitation |
| TB3254 How to Use the 12-Bit Differential ADC in Burst Accumulation Mode | DS90003254A | `TB3254-How-to-Use-the-12-Bit-Differential-ADC-Burst-Accumulation-Mode-DS90003254A.pdf` | Microchip's register-level recipe for the EA's ADC; not authoritative |
| TB3256 How to Use the 12-Bit Differential ADC with PGA in Single Mode | DS90003256A | `TB3256-How-to-Use-the-12-Bit-Differential-ADC-with-PGA-in-Single-Mode-DS90003256A.pdf` | the same, for the PGA; not authoritative |

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
  VPORT, so bare-pin code is unaffected, and `PinRef` - the one pin
  reached through a pointer - stores into the PORT's OUTSET/OUTCLR at
  0x0400 and up, never into VPORT; `AvrPlatform::idle()` writes
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
