# Vendor documents

The reference manual, datasheet and core manual the `ch32v203/` stratum
is written against (every target folder has its own `vendor/`). They are
NOT in the repository (`docs/*/vendor/*.pdf` is git-ignored): keep the
files listed below here (downloaded or symlinked), and cite them from
code and docs by document and SECTION, never by page ("RM 21.2.4",
"QingKe V4 manual 3.2"): sections survive revisions, pages do not. WCH
republishes without renumbering, so the revision of record is the one in
the table and a local copy is checked against it before it is trusted.

## CH32V203 (`brio/ch32v203/`)

| Document | Revision of record | File name here | Notes |
|----------|--------------------|----------------|-------|
| CH32F/V20x_V30x_V31x reference manual | **V2.3** | `CH32FV2x_V3xRM.PDF` (symlink to the desk's WCH folder) | FOUR FAMILIES IN ONE MANUAL - see the class rule below. 34 chapters: PWR ch. 2, RCC ch. 3, BKP ch. 4, CRC ch. 5, RTC ch. 6, IWDG ch. 7, WWDG ch. 8, the interrupt controller ch. 9, GPIO/AFIO ch. 10, DMA ch. 11, ADC ch. 12, TKEY ch. 13, the advanced timer ch. 14, the general-purpose ones ch. 15, USART ch. 18, I2C ch. 19, SPI/I2S ch. 20, the USB device controller ch. 21, the USB host/device one ch. 23, CAN ch. 24, OPA ch. 30, ESIG ch. 31, flash and the option bytes ch. 32, the extended configuration register ch. 33, debug support ch. 34 |
| CH32V203 datasheet | **V2.8** | `CH32V203DS0.PDF` | table 2-1 for the nine parts' resources (and the note that a package may limit them), the pinout figures of 3.1 and the pin tables of 3.2 per package, the alternate-function table of 3.3, the electrical characteristics of chapter 4 - tables 4-9 and 4-11 for the oscillator, whose "applied for V203RBT6" line is where that part's 32 MHz crystal comes from |
| QingKe V4 microprocessor manual | **V1.1** | `QingKeV4_Processor_Manual.PDF` | the core: the V4B this family carries (RV32IMAC, no floating point), the PFIC register set and the vector table (3.1, 3.2), the hardware prologue/epilogue, the free vectored entries, what wakes a WFI and a WFE, the STK system counter, and corecfgr (8.1) - the register WCH's own startup writes and the manual does not describe |
| WCH-Link user manual | **V2.4** | `WCH-LinkUserManual.PDF` | the probe: the modes, the pin table per family (this one is a TWO-wire port, PA13 = SWDIO and PA14 = SWCLK, not the CH32V00x's single wire), the firmware update path |
| CH32V20x EVT | the package dated by its own list file | `CH32V20xEVT.ZIP` | WCH's SDK and examples - NOT used by the build (its licence is written for software running on WCH parts) but the only vendor voice on silicon habits, and the ORACLE the working discipline names (CLAUDE.md): its `Startup/startup_ch32v20x_D6.S` and `startup_ch32v20x_D8.S` are where the two vector tables' tails are stated per class, its `EXAM/SRC` peripheral library is the sequence to read against ours when every variant of ours fails, and `EXAM/USB` is the one running USB device program on this silicon |

## The class rule, and where it bites

One manual covers the CH32F20x, the CH32V20x, the CH32V30x and the
CH32V31x, and it divides them by FLASH SIZE into classes - D6 is 32 or
64 KB, D8 is 128 or 256, with D8C and D8W for the connectivity and
wireless parts. Chapters, registers and single fields carry "applies to"
notes keyed by those names, and the CH32V203 spans TWO of them: every
part up to the CH32V203C8 is **CH32V20x_D6** and the CH32V203RB is
**CH32V20x_D8** (the manual's own list under "Specific classification
abbreviations"; the CH32V203K6, which the datasheet V2.8 adds and this
older manual does not name, is D6 by the size rule).

What is not ours is not implemented, and the document says so - but it
says so per class and not per family, which is why reading every such
note is part of a chapter's work rather than a courtesy. Two places
where it has already bitten:

- **the vector table** (9.2) is drawn as the union of this family with
  the CH32V30x, so its tail belongs to neither of ours: the D6's ends at
  62 with UART4 and the eighth DMA channel, the D8's runs to 69 with the
  Ethernet pair, TIM5 and the 32 kHz oscillator's two lines, and the
  table names the CH32V30x's TIM8 where both of those sit;
- **UART4's pads**: the chapter has two remap tables for it and the one
  starting at PC10/PC11 is the bigger classes', while this family's
  begins at PB0/PB1.

The chapters that are NOT this part's by class - the DAC (17), the other
USB host/device block (22), DVP (25), the FSMC (26), Ethernet (27) on
every part but the RB, SDIO (28) and the RNG (29) - are read only far
enough to confirm the note, and nothing of them is spelled in this
stratum.

## No errata sheet

WCH publishes no errata document for this family that the desk could
find: the CH32V203 folder of the vendor's site lists the datasheet, the
reference manual, the EVT package and the tool manuals and nothing
else, and their download pages are a JavaScript application that serves
no document list to a plain fetch, so the check is done by hand and
repeated before each chapter. Until one exists, the "Bench findings"
section of each document under `docs/ch32v203/` is where this family's
errata are written, and [../README.md](../README.md) carries the ones
that have no chapter document yet - the clock task's park on the HSI,
the PLL divider that lives in another chapter's register, the USB pads
that are still GPIO pads, and the controller that does not survive the
core's sleep.

The desk's copies live in `~/Documenti/Elettronica/WCH/`.
