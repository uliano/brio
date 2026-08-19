# Vendor documents

The datasheets and errata brio's target strata are written against.
They are NOT in the repository (Microchip's documents are not
redistributable, and they are large): `docs/vendor/*.pdf` is
git-ignored. Keep the files listed below here (downloaded or
symlinked), and cite them from code and docs by document number and
SECTION, never by page ("DS40002247B 16.5.2"): sections survive
revisions, pages do not.

Check the revision before trusting a local copy: Microchip republishes
both datasheets and errata (the errata grow with each silicon
revision), and file names in the wild are unreliable (a file named
"AVR64DB...Errata" turned out to be the AVR128DB errata). The revision
letter is in the first-page footer (`DS40002247B`, `DS80000915F`).

## AVR DA/DB (`lib/brio/src/avrdx/`)

| Document | Number / revision of record | File name here | Notes |
|----------|-----------------------------|----------------|-------|
| AVR128DB28/32/48/64 Data Sheet | **DS40002247B** (2023, complete, 730 pp) | `AVR128DB28-32-48-64-DataSheet-DS40002247.pdf` | chapters: 16 EVSYS, 17 PORTMUX, 18 PORT, 21 VREF, 33 ADC, 34 DAC. Supersedes the 2020 preliminary rev. A (660 pp) that older local copies are |
| AVR128DB28/32/48/64 Silicon Errata and Data Sheet Clarifications | **DS80000915F** (2025-04, 27 pp; silicon rev. A4/A5/B0) | `AVR128DB28-32-48-64-SilConErrataClarif-DS80000915.pdf` | items for ADC (MUX/accumulation delayed update), DAC (output buffer lifetime drift), CLKCTRL, PORT, SPI, TCA, TCB, TWI, USART - read before each driver. Rev. A (13 pp, 2020) is what older local copies are: incomplete |
| AVR128DA28/32/48/64 Data Sheet | DS40002183A (2020, preliminary) | `AVR128DA28-32-48-64-DataSheet-DS40002183A.pdf` (symlink) | the DA sibling; a newer revision likely exists - check before relying on it |
| AVR128DA28/32/48/64 Silicon Errata | DS80000882C | `AVR128DA28-32-48-64-SilConErrataClarif-DS80000882C.pdf` (symlink) | check for newer |
| Technical briefs TB3209 ADC, TB3213 RTC, TB3214 TCB, TB3217 TCA, TB3218 CCL, TB3212 TCD, TB3211 AC | DS9000320x | not copied here (in ~/Documenti/Elettronica/AVR) | Microchip "Getting started" briefs for megaAVR-0/Dx peripherals: useful complements, not authoritative |

Canonical URLs (redirect to the current revision):
`https://www.microchip.com/DS40002247` (datasheet),
`https://www.microchip.com/DS80000915` (errata). Direct files fetched
2026-08-19:
`https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/AVR128DB28-32-48-64-DataSheet-DS40002247.pdf`,
`https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/Errata/AVR128DB28-32-48-64-SilConErrataClarif-DS80000915.pdf`.

Header of record for register names: the toolchain's
`avr/ioavr128db48.h` (avr-libc); when the datasheet and the header
disagree on a name, the header wins in code and the datasheet section
is quoted in the comment.
