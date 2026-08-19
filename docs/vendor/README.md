# Vendor documents

The datasheets and errata brio's target strata are written against.
They are NOT in the repository (Microchip's documents are not
redistributable, and they are large): keep local copies here as
`docs/vendor/*.pdf` (git-ignored) or symlinks to wherever you store
them, and cite them from code and docs by document number and section
("DS40002247A 16.5.2") so anyone can find the passage.

## AVR DA/DB (`lib/brio/src/avrdx/`)

| Document | Number / revision | Local file | What we use it for |
|----------|-------------------|------------|--------------------|
| AVR128DB28/32/48/64 Data Sheet | DS40002247A (2020, preliminary) | `AVR128DB28-32-48-64.pdf` | everything; chapters: 16 EVSYS p.141, 17 PORTMUX p.155, 18 PORT p.170, 21 VREF p.216, 33 ADC p.494, 34 DAC p.524 |
| AVR128DB28/32/48/64 Silicon Errata and Data Sheet Clarification | DS80000915 (rev. A4/A5 silicon) | `AVR128DB28-32-48-64-Errata.pdf` | checked before every peripheral driver (no EVSYS items as of 2026-08) |
| AVR128DA28/32/48/64 Silicon Errata | DS80000882C | `AVR128DA-Errata.pdf` | the DA sibling, when a driver claims DA support |

Sources: https://www.microchip.com/ (product page -> Documents).
Header of record for register names: the toolchain's
`avr/ioavr128db48.h` (avr-libc); when the datasheet and the header
disagree on a name, the header wins in code and the datasheet number
is quoted in the comment.
