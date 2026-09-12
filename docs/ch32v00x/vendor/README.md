# Vendor documents

The reference manual, datasheet and core manual the `ch32v00x/` stratum
is written against (every target folder has its own `vendor/`). They
are NOT in the repository (`docs/*/vendor/*.pdf` is git-ignored): keep
the files listed below here (downloaded or symlinked), and cite them
from code and docs by document and SECTION, never by page ("RM 6.5.4",
"QingKe V2 manual 3.4"): sections survive revisions, pages do not. WCH
republishes without renumbering, so the revision of record is the one
in the table and a local copy is checked against it before it is
trusted.

## CH32V00x (`brio/ch32v00x/`)

| Document | Revision of record | File name here | Notes |
|----------|--------------------|----------------|-------|
| CH32V00X reference manual | **V1.5** | `CH32V00XRM.PDF` (symlink to the desk's WCH folder) | the whole family (CH32V002/004/005/006/007): 21 chapters, RCC ch. 3, PFIC and STK ch. 6, GPIO/AFIO ch. 7, DMA ch. 8, ADC ch. 9, the three timers ch. 11..13, USART ch. 14, I2C ch. 15, SPI ch. 16, OPA/CMP ch. 17, FLASH ch. 18, ESIG ch. 19 |
| CH32V006/005 datasheet | **V2.0** | `CH32V006DS0.PDF` | the pin tables per package (the QFN32 column is what says which pads the bench part bonds), the default and remapped alternate functions, the electrical characteristics; the model table with the parts' peripheral counts |
| QingKe V2 microprocessor manual | **V1.3** | `QingKeV2_Processor_Manual.PDF` | the core: mstatus on interrupt entry (2.2), the PFIC register set (3.1), the hardware prologue/epilogue (3.4), the vector table free entries (3.5), what wakes a WFI and a WFE (5.2), the debug module (ch. 6), the CSRs (ch. 7) |
| WCH-Link user manual | **V2.4** | `WCH-LinkUserManual.PDF` | the probe: modes, the pin table per family (table 6: SWDIO = PD1, SWCLK = PB3 for this one), the firmware update path |
| CH32V006 EVT | the package dated by its own list file | `CH32V006EVT.ZIP` | WCH's SDK and examples - NOT used by the build (its licence is written for software running on WCH parts) but the only vendor voice on silicon habits: the startup's INTSYSCR and mtvec settings, `GPIO_IPD_Unused` pulling every unused pad down, the clock-setting sequences |
| CH32V003 datasheet | **V1.6** | `CH32V003DS0.PDF` | the second part: the memories (1.4.2), the TSSOP20 pin table with the default and remapped functions (2.2), the flash page of 64 bytes (table 3-14), the electrical characteristics |
| CH32V003 reference manual | **V1.9** | `CH32V003RM.PDF` | the document of record for the chapters that differ on the CH32V003: RCC ch. 3 (no clock monitor), PFIC ch. 6, GPIO/AFIO ch. 7 (the two-bit MODE, PCFR1 at 0x04 with the F1's fields, tables 7-8 to 7-14), DMA ch. 8, ADC ch. 9, the timers ch. 10 and 11 (no dead time on TIM2), USART ch. 12 (the synchronous mode and the smartcard), I2C ch. 13 (the rise-time register), SPI ch. 14, FLASH ch. 16, EXTEND ch. 17 (the OPA's three bits) |
| CH32V003 EVT | the package dated by its own list file | `CH32V003EVT.ZIP` | WCH's SDK for the part: its `startup_ch32v00x.S` (INTSYSCR 0x3 - the hardware stack and NESTING, which brio keeps off), its core header's SRAM-resident WFE with a clock-dependent spin, and its peripheral headers, the one local voice on the CH32V003's register bits until the manual is on the desk |

NO ERRATA SHEET. WCH publishes none for this family that the desk
could find; check the download page before each chapter, and until one
exists the "Bench findings" section of each document under
`docs/ch32v00x/` is where this family's errata are written - two stand
already, in [../platform.md](../platform.md): a WFI that wakes only for
an interrupt it can take, and a MIE that interrupt entry does not
clear.

The desk's copies live in `~/Documenti/Elettronica/WCH/`, together
with the factory image dumped from the bench module before its first
flash.
