# Vendor documents

The reference manual, the datasheet and the core manual the `ch32x035/`
stratum is written against (every target folder has its own `vendor/`).
They are NOT in the repository (`docs/*/vendor/*.pdf` is git-ignored):
keep the files listed below here (downloaded or symlinked), and cite them
from code and docs by document and SECTION, never by page ("RM 8.3.1.8",
"QingKe V4 manual 3.4"): sections survive revisions, pages do not. WCH
republishes without renumbering, so the revision of record is the one in
the table and a local copy is checked against it before it is trusted.

## CH32X035 and CH32X033 (`brio/ch32x035/`)

| Document | Revision of record | File name here | Notes |
|----------|--------------------|----------------|-------|
| CH32X035 reference manual | **V1.8** | `CH32X035RM.PDF` (symlink to the desk's WCH folder) | the whole series, the CH32X033 included: 23 chapters - memory and bus ch. 1, PWR ch. 2, RCC ch. 3, the auto-wakeup ch. 4, IWDG ch. 5, WWDG ch. 6, the PFIC with the EXTI, the vector table and the STK ch. 7, GPIO/AFIO ch. 8, DMA ch. 9, ADC ch. 10, TKEY ch. 11, the advanced timer ch. 12, the general-purpose ones ch. 13, USART ch. 14, I2C ch. 15, SPI ch. 16, OPA and CMP ch. 17, the USB host/device controller ch. 18, ESIG ch. 19, flash and the option bytes ch. 20, USB PD ch. 21, the PIOC ch. 22, debug support ch. 23 |
| CH32X035/X033 datasheet | **V1.7** | `CH32X035DS0.PDF` | the model table before chapter 1 - the seven parts, their memories, package and pin count, and how many of each block a part carries; the pinout figures of 2.1 and table 2-1, the pin table per package, with its notes: 4 to 7 on the pins that carry two pads, 2 and 3 on the reset pin's function; the remap columns under the table (TX2_2 is USART2's TX under code 10); chapter 3's electrical characteristics - table 3-9 for the HSI |
| QingKe V4 microprocessor manual | **V1.1** | `QingKeV4_Processor_Manual.PDF` | the core: table 1-1 for the V4C against the V4A, V4B and V4F, 2.2 for what a trap does to mstatus, 3.1 for the PFIC register set, 3.2 for INTSYSCR and mtvec, 3.4 for the hardware prologue, the chapter on the system counter, 6 for the sleep modes and what ends them, 8.2 for mstatus and the MPP an MRET leaves standing, 8.3 for corecfgr - the register WCH's startup writes and no manual describes. A V1.5 waits among the candidates of the sibling stratum's folder ([../../ch32vx03/vendor/README.md](../../ch32vx03/vendor/README.md)) |
| WCH-Link user manual | **V2.4** | `WCH-LinkUserManual.PDF` | the probe: the modes, and table 6 - PC18 = SWDIO and PC19 = SWCLK for the CH32X035/X034/X033, a two-wire port and no single-wire one |
| CH32X035 EVT | the package dated by its own list file (2024.11) | `CH32X035EVT.ZIP` | WCH's SDK, examples and boards - NOT used by the build (its licence is written for software running on WCH parts) but the only vendor voice on silicon habits, and the ORACLE the working discipline names (CLAUDE.md): `EXAM/SRC/Startup/startup_ch32x035.S` (V1.0.1) is the 55-word vector table the crt agrees with word for word; `EXAM/SRC/Ld/Link.ld` the 62 KB at `0x0000 0000` and the 20 KB; `EXAM/SRC/Peripheral/inc/ch32x035.h` the register map this stratum's was CROSS-CHECKED against, never copied; `EXAM/SRC/Peripheral/src/` the library whose sequences are read against ours - the CFGHR copy in `ch32x035_gpio.c`, the clock setup in each example's `system_ch32x035.c`; `PUB/` the evaluation board's reference (V1.3), its schematic `CH32X035SCH.pdf` with one sheet per edition, and `SCHPCB/` their sources, the QFN20 edition's under `CH32X035F8U6-R0` |
| PIOC manuals | as found | `PIOC/` (`PIOC-EN.pdf`, `CHRISC8B-EN.pdf`, `PIOC UserManual.pdf`) | the co-processor of RM ch. 22 and its instruction set; no driver reads them |

NO ERRATA SHEET. WCH publishes none for this series that the desk could
find; until one exists, the "Bench findings" of each document under
`docs/ch32x035/` is where this series' errata are written. NO SVD: the
EVT package ships none, so the build project has no `svd/`.

## Where the documents disagree

The register map was written from the reference manual and read against
the EVT's header and library; where they part, the manual's word stands
unless the other is evidently the corrected one. Where the CH32X035F8U6
has answered an item, the item says what was measured; the rest is
unmeasured.

- **RCC_RSTSCKR's reset value, manual against manual.** The register
  table of RM 3.4 (table 3-1) gives 0x0C000000, PORRSTF and PINRSTF both
  set; the bit table of 3.4.8 gives PINRSTF a reset value of 0. The
  QFN20 has no reset pin, and there the bit table is right: the boots of
  the CH32X035F8U6 read PORRSTF with no PINRSTF beside it
  ([../clock.md](../clock.md)). A package that bonds the reset pin would
  answer for the register table. Nothing in the stratum depends on
  which.
- **The STK's INIT bit, manual against manual.** The QingKe V4 manual's
  counter chapter gives STK_CTLR an INIT bit at 5; this series' RM
  7.5.5.1 gives bits 30:5 reserved. The vendor header declares the
  64-bit CNT and CMP pairs both documents agree on. Nothing here writes
  INIT.
- **INTSYSCR's bit 4, manual against manual**: HWSTKOVEN in the QingKe
  V4 manual's 3.2, reserved in RM 7.5.3.1. The crt writes HWSTKEN
  alone.
- **The EXTI's lines, datasheet against manual**: the datasheet counts
  28 edge detectors, RM table 7-2 lists 30 lines (24 pads, the debug
  port's two, the PVD, the auto-wakeup, the two USB wake-ups).
- **The prefetch buffer, manual against itself**: RM 3.4.2 demands one
  be turned on when HPRE divides, and no register of the manual has the
  bit; the vendor's clock setup does not look for one. The vendor's way
  is measured on the CH32X035F8U6: with nothing written but HPRE and the
  wait states, every divided rung from /2 to /16 ran the clock suite's
  ladder clean ([../clock.md](../clock.md)); what the sentence means
  stays the manual's to say.
- **FLASH_ACTLR's latency field**: the vendor header's comment calls it
  `LATENCY[2:0]` over a two-bit mask; RM 20.3.1 gives LATENCY[1:0], the
  fourth code invalid.
- **The PFIC's banks**: the vendor's core header declares eight words
  for each of ISR, IPR, IENR, IRER, IPSR, IPRR and IACTR where RM 7.5.2
  lists four; the offsets of the four agree, and a series of 55 vectors
  needs two.
- **CFGHR, manual against library**: the RM says nothing of it, and the
  library's `GPIO_Init` keeps a RAM copy of each port's CFGHR and never
  reads the register on a die whose word at 0x1FFFF704 has zero in bits
  7:4 ([../pin.md](../pin.md)). The CH32X035F8U6's word reads 0x035E0611,
  1 in those bits, and its CFGHR reads back what the copy wrote: it is
  not the die the library guards, which stays unmeasured.
- **LBCL's sense, manual against library**: RM 14.10.5 says a 1 means
  the last data bit's clock pulse is NOT output; the library names that
  value `USART_LastBit_Enable` ([../usart.md](../usart.md)).
- **USART3's remap field, manual against itself**: RM 8.3.2.1 describes
  USART3_RM as the field that "controls the mapping of USART2's RX, CTS,
  TX, CK, RTS", and the columns under it are USART3's pads.
- **TIM3's remap note, manual against itself**: RM 8.3.2.1 says the
  remap "does not affect TIM3_ETR on PD2", and this series has no
  port D.
- **The chip identifier of the CH32X033F8P6, library against itself**:
  `DBGMCU_GetCHIPID()`'s comment lists 0x035A06x1, and
  `GPIO_IPD_Unused()` switches on 0x03117000 for the same part. The
  first list's form holds for the CH32X035F8U6, whose word reads
  0x035E0611 against the list's 0x035E06x1 - which decides nothing for
  the CH32X033F8P6.
- **PC3 on the CH32X035F7P6, library against datasheet**:
  `GPIO_IPD_Unused()` pulls PC3 up as an UNUSED pad on that part, and
  table 2-1 bonds it at the TSSOP20's pin 4 with the reset function.
- **The serial ports of the CH32X035F7P6, datasheet against itself**:
  the model table counts three; the pins reach two once note 4 forbids
  PC16 and PC17 as outputs ([../usart.md](../usart.md)).
- **The startup file, a habit rather than a disagreement**: WCH's writes
  INTSYSCR = 3 (the hardware stack and nesting) and enters main() in
  user mode through an mret with mstatus 0x88; its core header's
  `__WFE()` - which is also its `__WFI()` - sets an event and executes
  the converted `wfi` TWICE, and with SLEEPDEEP set ORs EXTI_INTENR into
  EXTI_EVENR around the sleep, masked, and puts it back after. The crt
  here keeps nesting off and main() in machine mode
  ([../README.md](../README.md)) - measured: INTSYSCR reads 0x1 and
  mstatus 0x1888 in a letter of the platform suite
  ([../platform.md](../platform.md)).
