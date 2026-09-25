# Vendor documents

The reference manual, the two datasheets and the core manual the
`ch32vx03/` stratum is written against (every target folder has its own
`vendor/`). They are
NOT in the repository (`docs/*/vendor/*.pdf` is git-ignored): keep the
files listed below here (downloaded or symlinked), and cite them from
code and docs by document and SECTION, never by page ("RM 21.2.4",
"QingKe V4 manual 3.2"): sections survive revisions, pages do not. WCH
republishes without renumbering, so the revision of record is the one in
the table and a local copy is checked against it before it is trusted.

## CH32V203 and CH32V303 (`brio/ch32vx03/`)

| Document | Revision of record | File name here | Notes |
|----------|--------------------|----------------|-------|
| CH32F/V20x_V30x_V31x reference manual | **V2.3** | `CH32FV2x_V3xRM.PDF` (symlink to the desk's WCH folder) | FOUR FAMILIES IN ONE MANUAL - see the class rule below. 34 chapters: PWR ch. 2, RCC ch. 3, BKP ch. 4, CRC ch. 5, RTC ch. 6, IWDG ch. 7, WWDG ch. 8, the interrupt controller ch. 9, GPIO/AFIO ch. 10, DMA ch. 11, ADC ch. 12, TKEY ch. 13, the advanced timer ch. 14, the general-purpose ones ch. 15, the basic ones ch. 16, the DAC ch. 17, USART ch. 18, I2C ch. 19, SPI/I2S ch. 20, the USB device controller ch. 21, the USB host/device one ch. 23, CAN ch. 24, the FSMC ch. 26, SDIO ch. 28, the RNG ch. 29, OPA ch. 30, ESIG ch. 31, flash and the option bytes ch. 32, the extended configuration register ch. 33, debug support ch. 34. A V2.5 waits among the candidates (below), with what it changes |
| CH32V203 datasheet | **V2.8** | `CH32V203DS0.PDF` | table 2-1 for the nine parts' resources (and the note that a package may limit them), the pinout figures of 3.1 and the pin tables of 3.2 per package, the alternate-function table of 3.3, the electrical characteristics of chapter 4 - tables 4-9 and 4-11 for the oscillator, whose "applied for V203RBT6" line is where that part's 32 MHz crystal comes from |
| CH32V303/305/307/317 datasheet | **V3.5** | `CH32V307DS0.PDF` | the CH32V303's: table 2-1-1 for the four parts' resources (the CB and the RB 128 KB + 32 KB, the RC and the VC 256 KB + 64 KB as the factory's option byte splits them), the pinouts of 3.1 and the pin tables of 3.2 per package, the alternate-function table of 3.3 - THE pad authority for this part - and chapter 4's electrical characteristics, tables 4-12 and 4-14 to 4-16 for the oscillators and the PLL; the CH32V305, CH32V307 and CH32V317 it also covers are the D8C class and not this stratum's. A V3.9 waits among the candidates: its table 2-1-1 gains a "Code FLASH" row giving every CH32V303 480 KB, the CB and the RB included, while its note 1 still gives the non-zero-wait area to the 256 KB parts alone |
| QingKe V4 microprocessor manual | **V1.1** | `QingKeV4_Processor_Manual.PDF` | the core: the V4B of the CH32V203 (RV32IMAC, no floating point) and the V4F of the CH32V303 (RV32IMAFC: 8.2 for mstatus.FS and the floating-point CSRs, 3.4's note 3 for the hardware prologue saving integer registers only), the PFIC register set and the vector table (3.1, 3.2), the hardware prologue/epilogue, the free vectored entries, what wakes a WFI and a WFE, the STK system counter, and corecfgr (8.1) - the register WCH's own startup writes and the manual does not describe. A V1.5 waits among the candidates, adding a fifth core to the series, the V4J with an instruction cache and the cstrcr register that configures it |
| WCH-Link user manual | **V2.4** | `WCH-LinkUserManual.PDF` | the probe: the modes, the pin table per family (this one is a TWO-wire port, PA13 = SWDIO and PA14 = SWCLK, not the CH32V00x's single wire), the firmware update path |
| CH32V20x EVT | the package dated by its own list file | `CH32V20xEVT.ZIP` | WCH's SDK and examples - NOT used by the build (its licence is written for software running on WCH parts) but the only vendor voice on silicon habits, and the ORACLE the working discipline names (CLAUDE.md): its `Startup/startup_ch32v20x_D6.S` and `startup_ch32v20x_D8.S` are where the two vector tables' tails are stated per class, its `EXAM/SRC` peripheral library is the sequence to read against ours when every variant of ours fails, and `EXAM/USB` is the one running USB device program on this silicon |
| CH32V30x EVT | the package dated by its own list file (2024.10) | `CH32V307EVT.ZIP` | the same - the vendor's voice and the ORACLE - for the CH32V303: `EXAM/SRC/Startup/startup_ch32v30x_D8.S` is its vector table (104 entries), which this stratum follows in every number but two - the USB wake-ups at 58 and 84 that file leaves zero and the silicon raises ([../README.md](../README.md)) - `startup_ch32v30x_D8C.S` being the CH32V305's and the CH32V307's; `EXAM/SRC/Peripheral/` is the library to read against ours, and `PUB/` the evaluation board's schematic (the sheet `CH32xx0xV_EVT` of `CH32V30xSCH.pdf`, the Altium sources under `SCHPCB/CH32V303VCT6-R0`, whose PCB carries the board's silkscreen name `CH32xx0xV-R0-1v0`) |

## The class rule, and where it bites

One manual covers the CH32F20x, the CH32V20x, the CH32V30x and the
CH32V31x, and it divides them by FLASH SIZE into classes - D6 is 32 or
64 KB, D8 is 128 or 256, with D8C and D8W for the connectivity and
wireless parts. Chapters, registers and single fields carry "applies to"
notes keyed by those names, and this stratum spans THREE of them: every
CH32V203 up to the CH32V203C8 is **CH32V20x_D6**, the CH32V203RB is
**CH32V20x_D8**, and the four CH32V303 parts (CB, RB, RC, VC) are
**CH32V30x_D8** - the manual's own list under "Specific classification
abbreviations", where the CH32V305 and the CH32V307 are the D8C and
not this stratum's (the CH32V203K6, which the datasheet V2.8 adds and
this older manual does not name, is D6 by the size rule). Several notes
go one step finer and key a bit to the LOT NUMBER within a class (the
fourth USB divider, ADC_DUTY_SEL, the converters' ADCx_AUX, the timers'
TIMx_AUX, EXTEN_CTR2, ADC2's DMA request, DMA1's 64 KB boundary, the
RTC's HSE division, the option byte's fifth memory split, USART3's
partial remap): such a bit is a measurement of the die, never a fact
of the part.

What is not ours is not implemented, and the document says so - but it
says so per class and not per family, which is why reading every such
note is part of a chapter's work rather than a courtesy. Where it has
already bitten:

- **the vector table** (9.2) is drawn as the union of the families, so
  its tail belongs to none of ours as drawn: the D6's ends at 62 with
  UART4 and the eighth DMA channel, the D8's runs to 69 with the
  Ethernet pair, TIM5 and the 32 kHz oscillator's two lines, the
  CH32V303's runs to 103, and the table names that part's TIM8 where
  both of the others sit;
- **the USB device controller**: chapter 21 opens by applying itself to
  the whole family, and on the CH32V303 its clock gate is not there
  (measured, [../clock.md](../clock.md)) - that part's one full-speed
  controller is chapter 23's host/device block;
- **UART4's pads**: the chapter has two remap tables for it and the one
  starting at PC10/PC11 is the bigger classes' - the CH32V303's - while
  the CH32V203C8's begins at PB0/PB1;
- **the lot notes**: the CH32V303VCT6 of the bench is a lot they
  RESTRICT - five of the registers they name are absent on it, one of
  their addresses a MIRROR of another register, and DMA1's 64 KB wrap
  present - so every lot-keyed feature is a verb that asks the die
  ([../README.md](../README.md)).

The chapters that are no part's of this stratum - the high-speed USB
host/device block (22), DVP (25), and Ethernet (27) on every part but
the CH32V203RB - are read only far enough to confirm the note, and
nothing of them is spelled here. The FSMC (26, the CH32V303VC's) and
SDIO (28, the CH32V303RC's and VC's) are the CH32V303's, and the
register map carries their bases and clock gates and nothing else - a
memory on the bus and a card socket are what their chapters want, and
the evaluation board has neither ([../README.md](../README.md)); the
DAC (17) and the RNG (29), the CH32V303's too, have their chapters,
[../dac.md](../dac.md) and [../rng.md](../rng.md).

## No errata sheet

WCH publishes no errata document for either series that the desk
could find: the vendor's folders list the datasheets, the reference
manual, the EVT packages and the tool manuals and nothing else, and
their download pages are a JavaScript application that serves no
document list to a plain fetch, so the check is done by hand and
repeated before each chapter. Until one exists, the "Bench findings"
section of each document under `docs/ch32vx03/` is where this family's
errata are written, and [../README.md](../README.md) gathers the ones
that shape more than one chapter - the clock task's park on the HSI,
the PLL divider that lives in another chapter's register, the USB pads
that are still GPIO pads, the bus that serves the core alone in a
sleep, the lot the CH32V303VCT6 of the bench is.

The desk's copies live in `~/Documenti/Elettronica/WCH/`. Newer
revisions of three of the documents above - the reference manual V2.5,
the CH32V303/305/307/317 datasheet V3.9 and the QingKe V4 manual V1.5,
fetched from community mirrors of WCH's PDFs - wait in that folder's
`candidates/` with their provenance; the revisions in the table stay of
record until a candidate is compared with WCH's own download and
promoted.

## What V2.5 changes

The reference manual's V2.5 keeps V2.3's section numbers throughout,
so every citation by section in this stratum survives a promotion.
What it changes is below, chapter by chapter, beside what the silicon
said where a suite measured it - and, where V2.5 leaves a sentence the
silicon contradicts, that too. Chapters 4 to 8, 13, 19, 29 and 31 read
the same in both; a chapter not listed differs in layout, figures
redrawn as text and spelling alone.

- **Ch. 2, power control.** 2.3.4 rewrites Standby as the regulator off
  and every circuit but the wake-up logic and the backup domain powered
  down, left through a power-on reset; figure 2-1 draws the amplifiers
  in the analog domain; the sentence that VDDA and VSSA must be tied to
  VDD and VSS is gone, and so is DBP's note that an RTC on the HSE over
  128 needs it. The silicon: every way out of a Standby measured on the
  two parts is a reset, a pad's EXTI line among them on the CH32V203C8T6,
  which neither revision lists ([../sleep.md](../sleep.md)); on the
  CH32V303VCT6 the independent watchdog's reset is one, as both
  revisions say ([../watchdog.md](../watchdog.md)), and the RAM a
  Standby keeps is exactly 2.3.4's two banks - through bits that read
  back zero there, which neither revision says.
- **Ch. 3, RCC.** CSSON gains a note that the clock security system
  does not apply to CH32V20x_D6 dies whose fifth digit from the end of
  the lot number is 0; RCC_RSTSCKR's LPWRRSTF becomes Reserved; the
  D8C's DVPRST turns from read-only to read-write. The silicon: the
  CH32V203C8T6 - whose RTC divides the HSE by 128, which 3.4.9 gives to
  the lots that note does not name - arms its security system over a
  healthy crystal, and so does the CH32V303VCT6
  ([../clock.md](../clock.md)); no reset measured on either part has
  raised LPWRRSTF, and a Standby wake on the CH32V203C8T6 leaves
  PORRSTF ([../platform.md](../platform.md)).
- **Ch. 9, interrupts.** Table 9-2 becomes one vector table per device
  class (the EXTI and PFIC tables after it renumbered 9-6 to 9-10, and
  the number 9-5 printed twice). The CH32V20x_D6's ends at 61 with
  USART4 and has no DMA1 channel 8 - whose vector the CH32V203C8T6
  raises at 62 ([../dma.md](../dma.md)); the CH32V30x_D8's names 58
  USBWakeUp - which EXTI line 18 pends on the CH32V303VCT6 and WCH's
  startup file for the class leaves zero - and still has no 84, which
  line 20 pends there ([../README.md](../README.md)). And 9.5.2 gains a
  note asking for a `fence.i` after masking an interrupt through
  PFIC_IENRx or the global one through the CSR: on the CH32V303VCT6 the
  `csrrci` on mstatus.MIE has no shadow with or without it, while a
  store into a line's own PFIC_IRER lets a pending interrupt through for
  up to three instructions, which a `fence.i` does not close
  ([../platform.md](../platform.md)).
- **Ch. 10, GPIO and AFIO.** USART3's partial column 10b is given to the
  CH32F20x_D8, CH32V30x_D8 and D8C classes alone, on the lots whose
  fifth digit from the end is 2 or more or whose sixth is not 0 - the
  lot rule V2.3 already had, with a class rule added; a note confines
  USART1 to its two lower columns on every CH32V20x class; TIM3's full
  column and UART5's remap lose their "not below 64 pins" notes;
  USART1's table takes the register's field names. The silicon:
  the CH32V303VCT6 takes USART3's remap field and TIM2's internal
  trigger bit, which the CH32V203C8T6 holds at zero; the 10b column's
  pads are the debug port's, and no die here has driven it
  ([../pin.md](../pin.md)).
- **Ch. 11, DMA.** The request figures redrawn as text - where DMA2's
  channel 5 appears without ADC2, while the table keeps its row with the
  lot note; the count becomes "transfers" and not bytes, DMA_CCRx the
  DMA_CFGRx it is; the 64 KB note and the table numbers unchanged. The
  silicon: the CH32V303VCT6 raises no ADC2 request, and its DMA1 wraps
  a span crossing a 64 KB boundary inside the page it started in - a
  lot the note restricts ([../dma.md](../dma.md),
  [../adc.md](../adc.md)).
- **Ch. 12, ADC.** Tables 12-1 and 12-2's TIM8 footnotes, swapped in
  V2.3, are corrected - the regular group's code 110 remapped by
  ADCx_ETRGREG_RM, the injected group's by ADCx_ETRGINJ_RM - which is
  the pairing the CH32V303VCT6 obeys; the self-calibration paragraph is
  deleted. Unchanged: 12.2.2's conversion of the sampling time plus 11
  cycles, where both datasheets and the CH32V303VCT6, at four sampling
  codes, give 12.5 ([../adc.md](../adc.md)).
- **Ch. 14 to 16, the timers.** Chapter 16 no longer applies itself to
  the CH32V20x, which has no basic timer - the part tables say so;
  chapter 15's software event register loses the break and commutation
  bits the general-purpose timers never had; the input-capture
  procedure is written for channel 1, MMS's "trigger input" becomes the
  output it is, and OSSI's misspelling is put right. TIMx_AUX's lot
  note stands in both, and the CH32V303VCT6 is a lot without it
  ([../tim.md](../tim.md)).
- **Ch. 17, DAC.** "Applies to the whole family" becomes "some family",
  no CH32V203 having the block; figure 17-5's feedback taps are drawn
  legibly; and 17.4.1's note that two enabled channels output channel
  1's wave on both pads is deleted. The silicon: the noise generator
  equals figure 17-5's register trigger for trigger, and with both
  channels enabled each pad follows its own channel on the
  CH32V303VCT6, so V2.3's note does not hold ([../dac.md](../dac.md)).
- **Ch. 18, USART, and ch. 20, SPI/I2S.** Figures as text, the baud-rate
  example rewritten (9600 and 115200 bit/s from 12 MHz in place of
  115200 and 921600 from the top rate), a CRC example's value corrected;
  no register or rule changes.
- **Ch. 21, the USB device controller.** Unchanged in substance, and
  still applying itself to the whole family - where the CH32V303VCT6
  has no such controller, its clock gate absent
  ([../README.md](../README.md)).
- **Ch. 23, the USB host/device controller.** The six BUF_MOD notes of
  endpoints 1 to 6, which contradicted table 23-4, are removed, and so
  is endpoint 4's "CH32V103x only"; a note after table 23-4 lets
  endpoint 3 carry up to 1023 bytes in a synchronous transfer when one
  of its directions is enabled alone and BUF_MOD is clear; the
  misprinted addresses of endpoint 3's two control registers stand. The
  silicon, on the CH32V303VCT6: a SETUP's status byte names no endpoint,
  bit 7 of the interrupt enable is the device's frame interrupt, and of
  the two wake-up lines table 9-3 names for the class a host's resume
  raises line 18 alone - none of it in either revision
  ([../usbfs.md](../usbfs.md)).
- **Ch. 29, the RNG.** Word for word the same - including the PLL48CLK
  the chapter says clocks the generator, where on the CH32V303VCT6 it
  is SYSCLK that runs the block ([../rng.md](../rng.md)).
- **Ch. 30, OPA.** Figure 30-1, the four amplifiers' switches, is added
  and CHP0/CHN0 become CH0P/CH0N; the register is unchanged. Unchanged
  too is chapter 33's EXTEN_CTR2 and its lot note - on the CH32V303VCT6
  that address is a mirror of EXTEN_CTR ([../opa.md](../opa.md)).
- **Ch. 32, flash.** 32.2.1's note becomes "FLASH erase related
  functions can only be placed in the zero-wait area" - a statement
  about where the CODE sits, which the CH32V303VCT6 confirms: both
  methods write the tail ([../nvm.md](../nvm.md)); the 4 KB grain is
  renamed a sector; FLASH_CTLR's reset value is 0x00008080 with FLOCK
  set - the two locks shut at every boot, as both parts show - and
  OBR's and WPR's are X; figure 32-2, redrawn in English, erases "8
  pages at a time" where V2.3's said sixteen, the text's 4 KB of sixteen
  pages; WRPR covers "480K bytes", its last bit sectors 31 to 119.
  Unchanged: FLASH_OBR's split field printed at [9:8], where both parts
  carry the whole USER byte at [9:2].
