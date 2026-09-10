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
| STM32G071x8/xB device errata | **ES0418 Rev 5** (November 2023) | symlink to `.../es0418-*.pdf` | the STM32G071RB's (the Nucleo-G071RB, revision B); READ AT THE BENCH - see "STM32G071RB: its errata read at the bench" below |
| STM32G031x4/x6/x8 datasheet | **DS12992 Rev 4** | not fetched as a PDF; the TEXT was read at the bench | the STM32G031K8's: pin assignment (table 12 - the LQFP32 column is what says which pads exist), alternate-function tables 13..17, the low-power mode paragraphs of 3.7 |
| STM32G031x4/x6/x8 device errata | **ES0487 Rev 6** (September 2023) | symlink to `.../es0487-stm32g031x4x6x8-device-errata-stmicroelectronics.pdf` | the STM32G031K8's (the Nucleo-G031K8, revision Y); READ AT THE BENCH against the letters - see "STM32G031K8: ES0487 read against the letters" below. st.com serves it to `curl` with a browser user agent where the same URL times out elsewhere |
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
  includes (ARM CMSIS_5 5.9.0), shared with the samc21 stratum.
- `stm32g0/svd/STM32G0B1.svd`, `STM32G071.svd`, `STM32G031.svd` - ST's
  SVDs for the debug Peripheral Viewer, one per bench part (Apache-2.0,
  from the cmsis-svd-data mirror of ST's pack), with their description
  text SANITIZED TO ASCII (the repo rule): ST's files carry
  double-encoded non-breaking spaces and curly quotes, mapped to plain
  ones, and a few symbols spelled out (micro as `u`, greater-or-equal
  as `>=`, not-equal as `!=`, the ellipsis as `...`). Register data
  untouched, every file re-parsed as XML.

## The bench chips, all three

| Board | Part | IDCODE | Sheet | Read at the bench? |
|---|---|---|---|---|
| Nucleo-G0B1RE (MB1360) | STM32G0B1RE, 512 KB / 144 KB, LQFP64 | **0x10016467** (DEV_ID 0x467, REV_ID 0x1001 = **revision Z**, ES0548 table 2) | ES0548 Rev 3 | yes, in hand |
| Nucleo-G071RB | STM32G071RB, 128 KB / 36 KB, LQFP64 | **0x20006460** (DEV_ID 0x460, REV_ID 0x2000 = **revision B**, ES0418 table 2) | ES0418 Rev 5 | yes, in hand |
| Nucleo-G031K8 (Nucleo-32) | STM32G031K8, 64 KB / 8 KB, LQFP32 | **0x10036466** (DEV_ID 0x466, REV_ID 0x1003 = **revision Y**, ES0487 table 2) | ES0487 Rev 6 | yes, in hand |

Every bench suite prints `part DEV_ID .. REV_ID ..` at boot from
`DeviceIdcode::read()`, because a measurement that differs between two
boards is only a finding once the die it was taken on is on the record.
Read the IDCODE at the bring-up of any new board - the errata columns
are per revision, and the SHEET is per part number.

## Errata ES0548 on revision Z: where each item is answered

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
- **2.13.1 Desynchronization under specific condition with edge
  filtering enabled** (workaround: disable edge filtering, or wait for
  the retransmission): `CCCR.EFBI` is a verb of `fdcan.hpp` with the
  obligation STATED on it - nothing a driver can enforce, since the
  coincidence is an end of integration landing on a falling FDCAN_RX
  edge. **STAGED AND NOT REPRODUCED**: 1500 CAN FD frames with bit rate
  switching in each arm, EFBI set and clear, every one byte-exact with
  no protocol error either side - recorded as unreproduced and not as a
  disproof, because a loop-back whose RX pin is disconnected may never
  offer the edge (fdcan.md).
- **2.13.2 Tx FIFO messages inverted under specific buffer usage and
  priority setting** is **UNREACHABLE BY CONSTRUCTION on this silicon,
  with the register as the evidence**: `FDCAN_TXBC` written 0xFFFFFFFF
  reads back 0x01000000, so the only implemented bit is TFQM. The item
  needs "both a dedicated Tx buffer and a Tx FIFO" and its own
  workaround names Tx buffers 4 and 5; this M_CAN is configured with
  three Tx elements and one mode bit, so the area is either a FIFO or a
  queue and a dedicated buffer cannot be declared (fdcan.md).
- 2.2.8 (boot select after a debug connection) and 2.11.2 (the USART
  prescaler exists only on some instances) are documentation errata,
  read and applied: the crt's boot assumptions and RM0444 table 183
  respectively. 2.11.2's subject is MEASURED and is worse than a missing
  divider: a BASIC instance TAKES a PRESC value and reads it back, and
  then transmits nothing at all - so `prescaler()` refuses on such an
  instance rather than trusting the readback (usart.md).

Answered in the chapter documents that own them: 2.2.1 (LSI unstable
across a VDD reset) in reset.md and rtc.md; 2.2.2 (PWR wake-up flags)
in pwr.md; 2.2.3 (a flash double word holding all ones cannot be
re-programmed to all zeros) in nvm.md; 2.2.6 (PC13 disturbs LSE),
2.2.11 (the RTC domain reset) and 2.9.1 (RTC) in rtc.md; 2.4.1 and
2.5.1..2.5.4 (DMA/DMAMUX) in dma.md; 2.6.x (ADC) in adc.md; 2.7.x (TIM)
in tim.md; 2.10.x (I2C) in i2c.md; 2.12.x (SPI) in spi.md. Revision A
only, absent on Z: 2.2.5, 2.2.7, 2.2.9, 2.3.1 (GPIO after a Standby
wake-up), 2.6.5.

**No item of ES0548 touches the CRC calculation unit** - a statement
about the document, not a claim about the silicon (crc.md).

## STM32G071RB: its errata read at the bench

The Nucleo-G071RB carries an **STM32G071RB, DBGMCU_IDCODE
DEV_ID 0x460, REV_ID 0x2000 = silicon revision B** (ES0418 table 2; Y is
0x2002 and is the newer of the two). Every bench suite of this stratum
prints that pair at boot, through `DeviceIdcode::read()` in `flash.hpp`,
because a measurement that differs between two boards is only a finding
once the die it was taken on is on the record.

FOURTEEN OF THE SEVENTEEN SUITES RUN THERE (`../README.md` has the table
and the reasons for the other three), so ES0418 can be read against real
letters rather than against a driver's intentions. The status letters are
ST's own: A = present with a workaround, N = present with none, P =
partial, "-" = absent.

| ES0418 (rev B) | Where a letter reaches it | What the bench found |
|---|---|---|
| **2.2.4** DMAMUX cannot be synchronized or triggered by EXTI (N) | `test_stm32_serial`'s boot probe; `test_stm32_dma` letter `f` | **REPRODUCED, WITH ITS MECHANISM.** Four pad edges into a request generator moved 0 words with the EXTI event mask alone and 1 of 4 with the interrupt mask armed too, while the EXTI's own rising pending bit saw every edge and four SOFTWARE events on the same line, generator and channel moved 4. The trigger follows the LEVEL of the line's pending interrupt: no IMR bit, no level, no trigger; with one, the first edge raises it and nothing more arrives until the pending bit is cleared. `test_stm32_dma` letter `f` PASSES on this die because its loop clears after every SWIER pulse - a SWIER-only staging reports the path healthy. Four verdicts of the serial suite skip; the sheet's "no workaround" holds, since the clear would need a CPU in a loop that has none |
| **2.2.6** Wakeup from Stop not effective (N) | `test_stm32_serial` letter `w`, the RTC-wake controls in `test_stm32_rtc` and `test_stm32_sleep` | the G0B1's 2.2.4, staged the same way: letter `w` scores 2 of 2 on this die too - the USART poke at HSIDIV = /4 does not wake the part, the RTC backstop ends the sleep, and HSIKERON does not rescue it, exactly as on the G0B1 (usart.md); REPRODUCED on revision B |
| **2.7.1** Invalid DAC output if MODE is written before data (A) | every DAC letter of `test_stm32_analog` - `configure()` writes MCR.MODE with the channel disabled and before any data write, the erratum's exact condition | **NOT REPRODUCED** under configure-then-set: the ADC reads a correct output through PA4 every time. Recorded as unreproduced, not as a disproof; the workaround (one write to any data register, then the MODE) is STATED on `dac.hpp`'s `configure()`, and `set()` is that write |
| **2.6.2 / 2.6.4** CFGR1 or CFGR2 written with ADEN set resets RES or CKMODE (A) | `adc.hpp` | **ANSWERED STRUCTURALLY**, and audited: every CFGR1 and CFGR2 store in the file is under a cleared ADEN - `configure()` refuses while enabled, `rebase()`'s store is behind a `disable()` whose failure aborts it, and `configure_while_enabled()` is the deliberate staging ground. There is no third |
| **2.6.7** ADC offset out of specification (A on rev B) | `test_stm32_analog` letter `a` prints CALFACT | not reached: the item's condition is VREF+ below 3.0 V and this rail measures 3317 mV. CALFACT came out at 46, neither of the 0x00 / 0x7F extremes |
| **2.6.6** trigger latency documentation | - | no letter measures a trigger latency, so nothing is judged against the corrected 6.5 / 12.5 / 3.5 PCLK figures |
| **2.9.1 / 2.9.2** LPTIM stuck (A / P) | the G0B1's 2.8.1 / 2.8.2, structurally and in `test_stm32_lptim` letter `i` | unchanged: 82/82 on this die, both halves of the flag-clear discipline behaving as on the other |
| **2.11.1** I2C tSU;DAT floors (P) | `test_stm32_i2c` letter `m`'s refusals | identical - the floors are the driver's compile-time and run-time refusals and do not depend on the die |
| **2.11.2** I2C spurious BERR (A) | `test_stm32_i2c` letter `m` counts BERR over the run | not seen |
| **2.11.6** I2C stalled when PCLK/I2CCLK is in 1.5..3 (A) | - | NOT REACHED: this suite's ratios are 4 (a 64 MHz PCLK with the HSI16 kernel), 1 (PCLK as the kernel) and 0.125 (the 2 MHz core), none of them in the band |
| **2.11.3 / 2.11.5 / 2.11.7** own-address match, NOSTRETCH underrun, SMBus slave timeout | multi-master, the NOSTRETCH target and the SMBus target are SELF-LINK roles | not staged on this board: its I2C1 faces a peer, not a second instance of its own |
| **2.12.2** noisy receive line (A) | `test_stm32_serial` letter `f` | the G0B1's 2.11.1, staged identically |
| **2.12.3** prescaler documentation | `test_stm32_serial` letter `a` | measured on this die too, and worse than a missing divider: a BASIC instance takes the PRESC value, reads it back and transmits nothing |
| **2.12.4** data corrupted when ABREN is cleared during a reception (A) | `usart.hpp` | **ANSWERED STRUCTURALLY**: `auto_baud_off()` refuses with UE set, so there is no path to that write and a disabled receiver is not receiving |
| **2.12.5** NE set with ONEBIT on a noisy START bit (N) | letter `f`'s ONEBIT rows | not exercised: those rows run on a clean start bit, the suite's bit-banger putting its glitch in the stop bit |
| **2.12.1** USART SPI-slave TC anticipated (A) | - | not staged: there is no synchronous slave on this desk |
| **2.13.1** LPUART transmitter jitter, kernel/baud a non-integer in 3..4 (P) | `test_stm32_serial` letter `v`, whose LSE-kernel leg at 9600 baud is the sheet's own 32768 / 9600 = 3.41 | the band is a stated caveat in [../lpuart.md](../lpuart.md); letter `n`'s own rungs (ratios 555.6 and 6666.7) are outside it |
| **2.14.1 / 2.14.2** SPI BSY on disable, slave BSY (A) | `test_stm32_spi` letter `h`, a SELF-LINK letter | NOT STAGED on this board: the self-link is not wired here, so the letter skips. Both are the G0B1's 2.12.1 and 2.12.2 and are measured there |
| **2.8.6** TIM16/TIM17 clocked by SYSCLK rather than TIMPCLK (N) | - | UNOBSERVABLE by construction: this stratum pins HPRE and PPRE at 1, so the two clocks are one |
| **2.8.4 / 2.8.5** bidirectional break with short pulses, sync trigger missed with a faster master clock (N) | - | not reached: no letter sets BKBID, and one clock feeds every timer |
| **2.2.5** a flash location of all ones cannot be re-programmed to all zeros (N) | `test_stm32_nvm`, which stays the G0B1RE's | not staged here (no storage geometry on a single-bank part) |
| **2.2.3, 2.2.7, 2.2.9** RDP1 boot via PA14, PCROP weakness, option-byte lock | - | not staged: there is no option-byte write verb, PCROP is a read-only decode and RDP is one-way |
| **2.2.1, 2.2.2, 2.2.8, 2.2.10, 2.2.11, 2.10.2** LSI, WUFx, boot select, RTC domain, tamper flag on LSE failure | the same items as the G0B1's, applied the same way | unchanged; the LSE CSS stays declined, its only way back being the domain reset this bench must not take |
| **2.3.1** a GPIO assigned to a DAC channel cannot be an output while that channel is on-chip only (N) | `test_stm32_analog`'s `DacMode::internal_unbuffered` leg | not reached: no letter drives PA4 or PA5 as a GPIO output while such a mode holds |
| **2.7.2** DAC DMA underrun flag missed with concurrent SW+HW triggers (N) | `test_stm32_analog` letter `p` | not reached: the condition needs software and hardware triggers used together and this letter uses one at a time |
| **2.5.4, 2.6.1, 2.6.3, 2.6.5** and the remaining shared items | the G0B1's twins, applied where they are | unchanged |

**ES0418 HAS NO PREFETCH ITEM** - the G0B1's 2.2.10 (prefetch failure
branching across flash memory banks) has no twin here, and it could not:
this part has ONE bank. `FlashAccel::prefetch` stays a verb left at its
reset value on both parts, because the reason to leave it there is the
G0B1's and a common default is worth more than a per-part one.

**AND ONE PER-PART FACT THAT IS NOT AN ERRATUM AT ALL**, found the hard
way and now in the reserve: RM0444 22.4.25's ETRSEL list footnotes codes
0100 (MCO), 0101 (MCO2) and 0110 (COMP3) "available on STM32G0B1xx and
STM32G0C1xx sales types only". No TIM register differs between the
headers, so a timer told to take MCO on a smaller part counts nothing, in
silence - which wedged a suite whose own microsecond wait rode that timer
([../tim.md](../tim.md), [../clock.md](../clock.md)).

## STM32G031K8: ES0487 read against the letters

The Nucleo-G031K8 reports **DEV_ID 0x466, REV_ID
0x1003**, which ES0487's table 2 names **revision Y** (0x1001 is Z).
The sheet numbers its items on its own scale and A NUMBER DOES NOT
TRAVEL BETWEEN SHEETS, so the table pairs each limitation BY TITLE with
its ES0548 twin; the suites' own prints still name the G0B1's number as
the description being staged ("ES0487's twin pending" - true when they
were written, the sheet arrived after), and the verdict is here.

| ES0487 item, status on revision Y | ES0548 twin | Where a letter reaches it on the G031K8 | What this die did |
|---|---|---|---|
| **2.2.4** DMAMUX cannot be synchronized or triggered by EXTI - **ABSENT ON Y** (N on Z; the G071's ES0418 2.2.4, reproduced on the G071RB) | none | `test_stm32_serial`'s boot probe on PB3 | THE DIE AGREES: four pad edges into a request generator move **4 words with the event mask alone and 4 with the interrupt mask armed** (F: 0 and 1 of 4), so the four edge-counter verdicts that skip on F run here |
| **2.2.6** wakeup from Stop not effective with HSIDIV != 0 - N/N | 2.2.4 | `test_stm32_sleep` letter `g`, with its control | the hazard predicate is quiet at HSIDIV 0 and speaks as soon as the divider moves; an RTC wake is not a clock-request wake and a 250 ms Stop under HSIDIV /4 lasted its time. No USART subject: this part's USART2 cannot wake from Stop at all |
| **2.2.2** WUFx set while configuring the pin - A/A | 2.2.2 | `test_stm32_sleep` letter `h` | `wakeup_pin()` clears WUFx as part of the configuration, so the flag cannot reach a caller - the workaround as code |
| **2.2.1** unstable LSI when it clocks the RTC and VDD resets without the backup domain - P/P | 2.2.1 | - | NOT STAGED, and it weighs more on this board than on the other two: ITS RTC RUNS ON LSI. The workaround is a program's - on a power-on (BORRSTF) reset the backup domain before trusting it |
| **2.2.5** overwriting all ones with all zeros fails - N/N | 2.2.3 | - | not reached: the flash suites stay the G0B1RE's |
| **2.2.8** PC13 transitions disturb LSE - N/N | 2.2.6 | - | moot: PC13 reaches no pin and the LSE does not start |
| **2.2.11** RTC domain corrupted on a missed power-on reset - A/A | 2.2.11 | - | not staged |
| **2.3.1** DMA disable failure on a transfer error coinciding with a GIF clear - A/A; **2.4.1..2.4.4** the DMAMUX flags and the synchronization write - N/N/N/A | 2.4.1; 2.5.1..2.5.4 | dma.hpp | the same block as the other dies', item for item by title: whatever [dma.md](../dma.md) records for ES0548's holds here |
| **2.5.1..2.5.4** ADC overrun flag, CFGR1 under ADEN, AWD1 in single mode, sampling one cycle longer - P/A/A/N | 2.6.1..2.6.4 | `test_stm32_analog`, adc.hpp | 2.5.2 is structural on every part (a configure with ADEN set is refused), as audited on the G071RB; **2.5.6** (offset out of specification) is Z only |
| **2.6.2** consecutive compare event missed - N/N | 2.7.2 | `test_stm32_tim` letter `k`, staged with a control | did NOT reproduce in that staging: the second compare raised its flag and toggled its output 8 of 8 - unrefuted rather than disproved, as on the other two dies |
| **2.6.1, 2.6.3** the one-pulse trigger, output-compare clear - P/P; **2.6.4** TIM1's sync trigger missed by a slower slave - N/N; **2.6.5** TIM16/17 clocked by SYSCLK | 2.7.1, 2.7.3; none; none | - | not staged (2.6.1 needs a trigger placed at CNT = ARR of a cascaded master, 2.6.3 an `ocref_clr` this part has no comparator for, 2.6.4 has no G0B1 twin and no letter); 2.6.5 is Z only |
| **2.7.1, 2.7.2** LPTIM stuck in its interrupt on a disable / on a flag clear - A/A, P/P | 2.8.1, 2.8.2 | lptim.hpp, as code | unchanged: `disable()` IS the RCC reset, a thread-mode clear with an interrupt enabled is refused |
| **2.8.1** calendar initialization fails on consecutive INIT entries - A/A | 2.9.1 | `test_stm32_rtc` letter `h`, guarded against raw | did not reproduce on the unguarded path here either; the guarded path never corrupts a calendar |
| **2.9.1** tSU;DAT shorter than one kernel clock - P/P | 2.10.1 | `test_stm32_i2c` letter `m` | the floors (4, 10, 20 MHz) are stricter than the datasheet's and the driver refuses below them |
| **2.9.2** spurious BERR in master mode - A/A | 2.10.2 | `test_stm32_i2c` letter `m` | not seen: BERR swept 0 times; the driver counts it and never reports it - the workaround |
| **2.9.3** spurious master transfer upon own slave address match - P/P | none | - | not staged: it wants one node that is master and slave at once while a second master addresses it |
| **2.10.1** data corruption from a glitch in the stop bit's second half - **A/A here, N/N on the G0B1** | 2.11.1 | `test_stm32_serial` letter `f`, with its control | REPRODUCES on this die: the glitch in the second half reaches the byte, the one in the first half does not |
| **2.11.1, 2.11.2** SPI BSY high after a disable / at the end of a slave transfer - A/A | 2.12.1, 2.12.2 | `test_stm32_spi` letter `h` | NOT REACHED: that letter's instrument is the self-link, which the LQFP32 cannot carry (SPI2's pads reach no pin) |
| documentation errata **2.2.10**, **2.5.5**, **2.10.2** | - | - | 2.5.5 corrects the ADC trigger latency to 6.5/12.5/3.5 PCLK cycles; 2.10.2 says PRESC is absent on some instances, which is the fact the serial suite measured on USART2 |
| ES0548 2.13.x (FDCAN), 2.14.x (UCPD) | - | - | no counterpart: this part has neither |

**AND ONE THING THE SHEET DOES NOT CARRY.** Two facts of this board were
found the hard way and belong beside the errata because they look like
them until they are named: **Shutdown powers the LSI down** (DS12992
3.7.4 - it is in the datasheet, not in an errata sheet, and it means an
RTC on LSI cannot end a Shutdown), and **the debug port can go silent**
until the board is replugged ([../../probes/st-link.md](../../probes/st-link.md) - a state of the ST-LINK
half, not a silicon claim).
