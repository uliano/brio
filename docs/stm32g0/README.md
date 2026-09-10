# Target: STM32G0 (`stm32g0/`)

The operational page for brio's STM32G0 target: an ARM Cortex-M0+
(STM32G0B1RE on the bench, on an ST Nucleo-64), an ARMv6-M family that
shares the `armv6m/` core stratum with the SAM C21. `kernel/` and
`util/` run here as written: time events pacing a pin, and the full
console over the board's own virtual COM port.

Peripheral documents live next to this page, one per chapter - the
map below; the documents of record are in [vendor/README.md](vendor/README.md)
together with the errata pass and the bench chip's identity (silicon
revision Z, DBGMCU_IDCODE read over SWD).

## The documents

One document per peripheral driver, in the shape
[../README.md](../README.md) prescribes; the vendor page carries the
documents of record.

| Document | Content |
|----------|---------|
| [platform.md](platform.md) | Platform (STM32G0): `Stm32g0Platform<TB>` templated on its TIMEBASE - PRIMASK critical section, WFI idle in Sleep mode, the 16 + 32 vector table with its shared lines, the crt, the SRAM parity option - with TWO timebases: the SysTick `Ticker` (1000 Hz, stops in Stop) and `LptimTicker` (the LPTIM on the LSE crystal undivided, the kernel tick its count shifted to 1024 Hz), which makes a program TICKLESS: no periodic interrupt at all, the kernel's optional `idle_until()` placing every deadline in the compare, kernel time running through a Stop with the plain sleep site and no resync. Measured: a CMP write lands in 2..3 counts of WHATEVER clock the counter runs on (74..88 us undivided, 2.0..2.8 ms at /32 - the finding that put the counter at /1 with a shifted tick), CMPM fires at the count edge AFTER equality, a compare at ARR matches, a store immediately before a Stop 1 lands with PCLK stopped; three periodics at 7/30/250 ticks fire exactly on their ticks for three seconds with one LPTIM wake per deadline and NOT ONE EARLY; a 500-tick deadline through Stop 1 under the manager matures at 488 ms of RTC wall with 500 ticks elapsed. Not covered: VTOR relocation, NVIC priorities (exposed, nothing assigns one), a per-package pin-bonding table |
| [reset.md](reset.md) | Reset + IWDG + WWDG (STM32G0): RCC_CSR's reset flags as the ACCUMULATING history they are, with PINRSTF the catch-all that names the pin only alone; the independent watchdog whose keyed registers do not update until the START key gives the block its clock, and which the reset it causes really does stop (measured, against the family lore); the window watchdog whose counter free-runs and whose EWIF rises with the activation bit clear, though the interrupt does not; and the panic breadcrumb proven across six real resets including a HardFault; not covered: the option bytes behind both watchdogs, the NRST pin's modes, the PWR side of PWRRSTF |
| [clock.md](clock.md) | Clock (STM32G0): the THIRD clock model - one SYSCLK, shared bus prescalers, per-peripheral enable bits, kernel-clock multiplexers, the MCO clock output; HSI16 and the PLL to 64 MHz, the flash latency sequence, the three voltage regimes (Range 1, Range 2 with its own latency column, low-power run) as static rates; THE DYNAMIC CLOCK over a pack of rate tuples - measured: every rung as the type claims and the CPU at the claimed rate on the crystal's scale, a switch 48..384 us, the four once-unbenched steps closed, the rebased USART and ADC exact through 72 switches, a Stop at each end of the ladder (the part wakes IN low-power run with HSIDIV kept), HSIDIV never left behind a PLL rate |
| [port.md](port.md) | GPIO (STM32G0): a port has a clock and it is off at reset, analog is the reset state, AF numbers are the datasheet's, BSRR/BRR atomic values - and FOUR PADS ARE NOT ORDINARY GPIOs OUT OF A POWER-ON, PA8/PB15 and PD0/PD2 carrying the UCPD Type-C dead-battery pull-downs until the SYSCFG strobe releases them (measured: the pad does not follow its own pull-up until it does; UCPD itself - G071/G081/G0B1/G0C1 only - is not implemented, deferred to a board with the connector) |
| [exti.md](exti.md) | EXTI (STM32G0): where this family keeps its PIN INTERRUPTS - sixteen GPIO lines whose NUMBER IS THE PIN NUMBER and whose PORT is the one choice (so PA3 and PB3 are one line and cannot both be armed), edges only and no level, rising and falling in SEPARATE pending registers, a software trigger that needs no pad, three shared vectors and the ISR body that confines a handler to its own lines; the pending bit is only set for an UNMASKED interrupt (13.3.1, measured - a line cannot be polled without arming it), IMR1's reset value follows 13.5.12's RULE and not its printed number, and an EXTI EVENT returns the core from WFE with nothing to acknowledge; the driver owns the fabric and not the vocabulary of the direct lines; not covered: wake-up from Stop/Standby (no PWR driver), the direct lines' peripherals, the non-GPIO configurable lines |
| [nvm.md](nvm.md) | FLASH (STM32G0): the program/erase engine of RM0444 ch. 3 - page 2048, row 256, double word 8, a cell written once between erases - with the option bytes as a READ-ONLY decode (nothing here can brick a board), the ECC status and the one interrupt; the storage design that gives BANK 2 to util/nv_heap.hpp and util/nv_journal.hpp while the linker keeps the image in bank 1, so read-while-write makes a save legal from the loop and ES0548 2.2.10's own note blesses the arrangement; EOP proven not to rise without EOPIE (3.7.4 against 3.3.8's step 7), fast programming measured at a third of its datasheet typical, and a malformed program found to WEDGE CFGBSY until the next reset rather than merely raise a flag; not covered: writing any option byte, WRP/PCROP/securable/RDP, the OTP write, a RAM-resident erase path |
| [tim.md](tim.md) | Timers (STM32G0): TEN timers behind ONE `TIM_TypeDef`, so what a timer IS - width, channels, complementary outputs, slave controller, break unit, centre-aligned counting - is the DOCUMENTS' table and every verb naming a feature an instance lacks refuses; the status register that is `rc_w0` and not write-one-to-clear (the one register of this stratum where the reflex is wrong); the shadowed prescaler and auto-reload; MOE clear out of reset; shared vectors with TIM1 alone on two; TISEL, the input multiplexer that makes a CAPTURE MEASURABLE WITH NO PAD (LSI into TIM16, cross-checking the watchdog's own LSI figure); a centre-aligned period measured to be 2 x ARR and not 2 x (ARR + 1); a dead time asked for in tDTS and delivered to the nanosecond, with the pair never both high; and the two util contracts running here unchanged - PwmChannel through TimPwm/TimPairPwm and MeterSampler through a capture ISR, plus the DMA REQUESTS each instance publishes (table 55's numbers, TIM14 having none at all) and the DMA BURST engine whose register map HAS HOLES; ALL TEN INSTANCES have now counted here, the six that never had - TIM4, TIM6, TIM7, TIM14, TIM15, TIM17 - each against SysTick, each on the vector the reserve derives from what shares it, and each with channels driving a pad, where a complementary pair's dead time comes off BOTH halves exactly; not covered: encoder and hall-sensor modes, the commutation event, the ETR sources other than MCO, LPTIM/IRTIM |
| [dma.md](dma.md) | DMA + DMAMUX (STM32G0): two controllers (seven channels and five on the G0B1, one controller elsewhere) behind ONE request multiplexer, three vectors for twelve channels, and a request vocabulary NO HEADER OF THIS PACK DECLARES - so the driver owns the fabric and each peripheral publishes its own table-55 codes. A REQUEST IS A LEVEL SERVED ON ENABLE, NOT AN EDGE LATCHED ON THE RISE (measured on a USART with no pads claimed and a control with the request bit clear), which is why this driver has no kick() where the SAM's needs one; suspend-and-resume is unsupported by the silicon and only abort-and-restart is offered; a transfer error disables the channel in HARDWARE and gates the re-enable behind its own flag; ES0548 2.4.1 and 2.5.4 are answered STRUCTURALLY (CGIFx is unwritable here, and SE can only be set with SPOL in the same word). THE FIXED POINT: util/block_stream.hpp's two concepts, written before this implementation, are met UNCHANGED - BlockPlayer fits the hardware CIRCULAR mode exactly and gains by it (a player runs with its interrupt disarmed, where the SAM re-armed once a lap), and BlockSource does NOT, because a circular channel never stops and the 'skip rather than tear' rule can only be decided after the edge: measured, six elements had already landed in the half a caller was holding at a 2 MHz request rate, so the ping-pong engine uses a non-circular channel. Also measured: two memory-to-memory channels ALTERNATE and the software priority does not enter into it (10.4.4's own clause), the lower channel index winning every time; 49.7 MB/s of memory-to-memory; table 51's widening and truncation as printed; a request generator turning TIM14_OC into batches of GNBREQ + 1; one channel pacing another through EGE with nothing in between; and the ST-LINK VCP's own ceiling, byte-exact to 921600 in both directions and broken at 2 Mbaud. DMA2 is FIVE channels and not one (every width on every one of them), 10.4.5's peripheral-to-peripheral runs (TIM6's update moving TIM3's CCR1 into TIM4's), and THE SLEEP STORY has two halves with opposite answers for one reason - the controller is on HCLK: a block runs to the end with the CPU in WFI and its own completion wakes the core, while a paced stream is FROZEN by a Stop and picks up exactly where it stopped. Not covered: linked lists (there are none), RCC_AHBSMENR's sleep-mode enables (no verb in clock.hpp) |
| [usart.md](usart.md) | USART (STM32G0): RM0444 ch. 33 WHOLE - every field of both register views - and almost all of it measured on ONE BOARD WITH NO WIRES, because CR3.HDSEL turns any instance into its own loop-back (there is no LBME here) and a pad under the RX alternate function is an INPUT whose internal pull the CPU moves in under a microsecond, which makes software a transmitter at 2400 baud. The instances are not copies of each other and THREE AUTHORITIES are compared on every one: the reserve's stated table 183, the device header's pointer-comparison macros and the silicon. ONE ROW OF TABLE 184 IS NOT THE FULL/BASIC SPLIT - synchronous mode belongs to every USART and to no LPUART, CR2.CLKEN sticking on all six here and on neither LPUART. And the PRESCALER IS WORSE THAN ABSENT ON A BASIC INSTANCE: the field takes the value and reads it back, and the transmitter then emits nothing at all, which is why the verb refuses instead of trusting. Measured: all thirteen frame formats byte-exact; both of 33.5.7's baud examples in BOTH oversamplings, with OVER8's register proven not to be the divisor; the twelve prescaler codes; the console MOVED UNDER ITSELF to HSI16 and to SYSCLK while its own verdict lines came out at 115200; USART1 run off the 32768 Hz crystal at 2048 baud; the FIFOs' depth, thresholds, overrun order and OVRDIS bypass - with the honest finding that A LOOP CANNOT SHOW WHAT A FIFO IS FOR (one interrupt a byte either way, because a single wire is its own pacer); the noise flag proven to be a MAJORITY-VOTE DISAGREEMENT and nothing else; the tolerance of tables 188/189 walked from both sides in all four arrangements and MEETING every row; all four auto-baud patterns learning a rate nobody was told to one part in a thousand; the LIN break at thirteen bits and its detector counting to exactly LBDL; mute mode, a 22-bit receiver time-out and the character match, with CR2.ADD proven to be ONE field with two jobs; smartcard 8E1.5 with its guard time in BAUD periods, its CK at kernel/(2 x PSC) and its NACK retries counted on the pad; IrDA's 3/16 pulse, its low-power width that no longer moves with the baud rate, an RZI frame decoded, and a glitch filter one PSC period wide; SWAP, the three inversions, MSBFIRST as an exact bit reversal, DE in SAMPLE times, RTS and CTS; and the synchronous master's CK counted with no CPU. Errata: 2.11.1 REPRODUCED WITH ITS CONTROL (8 of 8 frames silently corrupted by a quarter-bit glitch in the second half of a stop bit, 0 of 8 in the first), 2.11.2 measured, and 2.2.4 REPRODUCED ON A USART WAKE where it does not reach an RTC one - with HSIKERON proven NOT to be the way round it. Not covered: the synchronous DATA path and the SLAVE half (a second board), smartcard block mode (a real card), a LIN network |
| [lpuart.md](lpuart.md) | LPUART (STM32G0): RM0444 ch. 34, which is ch. 33 minus synchronous/smartcard/IrDA/LIN/Modbus/receiver-timeout/auto-baud/OVER8 and PLUS a baud generator of its own - 256 x fck / LPUARTDIV, twenty bits, a floor of 0x300, and 34.4.7's two rules proven to be ONE (256 x 3 IS 0x300). `LpUart` is `usart.hpp`'s OWN `UartTask` named over an `Lpuart<n>`: one implementation, two peripherals, because a second copy would have been a second place to get the ORE storm wrong. Measured on both instances' own single wires: all six rows of table 198 in BRR (three of them nearer the truth than the manual's own truncation), 9600 proven to be the LSE's ceiling by refusal, all four kernel clocks byte-exact, the FIFO and the prescaler, and the shared vectors - LPUART1 on USART3..6's line and LPUART2 on the console's. AND THE REASON THE PERIPHERAL EXISTS: a WHOLE CONSOLE moved onto LPUART1 through the console's own pads on AF6 - the same task, the same verbs - took the host's stream byte-exact on the 32768 Hz crystal at 9600 and on HSI16 at 115200, and came back; then, with the console down, an LPUART on the crystal brought the part out of STOP 1 on a start bit and the character that woke it survived. Not covered: the features shared with ch. 33 that are measured on a USART instead, the DMA engine slots (compile-only here), LPUART2 as a wake source |
| [irtim.md](irtim.md) | Infrared interface (STM32G0): RM0444 ch. 27 - two pages, three bits of SYSCFG_CFGR1 and one pad, and an AND gate that is the whole physical layer of an infrared remote with no CPU in it. TIM17 channel 1 is always the carrier and IR_MOD chooses the envelope among TIM16, USART1 and one more USART whose INSTANCE IS A PER-PART FACT (USART4 wherever the part has one, USART2 otherwise - derived in the reserve from USART4_BASE). NEITHER TIMER NEEDS A PAD: figure 278's connections are internal, so an infrared output costs exactly one pin. Measured: a 38 kHz carrier under a 1 kHz 50 % envelope gives 1901 rising edges on PB9 in 100 ms against a product of 1900, counted by a DMA channel with no CPU; IR_POL inverts the resting level; and THE FINDING CHAPTER 27 DOES NOT CARRY - the USART envelope is ACTIVE LOW where TIM16's is active high, so an idle transmit line shuts the gate completely (0 edges a millisecond) and a 0x00 character opens it for exactly nine tenths of its frame, which is the right way round for infrared. The high-sink LED driver is SYSCFG_CFGR1.I2C_PB9_FMP - one bit with two names, PB9's alone. Not covered: what the high-sink bit is worth in milliamps, the second USART envelope on the pad, and the PA13 IR_OUT pad, which is SWDIO on every Nucleo |
| [adc.md](adc.md) | ADC (STM32G0): the one 12-bit SAR converter of RM0444 ch. 15 - a MONOSTATE, because every part has exactly one - with the two things it has instead of a factory trim (an internal regulator with a start-up time and an ADCAL self calibration, both procedures `init()` spends real microseconds on) and the three factory MEASUREMENTS it reads instead (VREFINT_CAL and TS_CAL1/2 at the datasheet's addresses, ADC results taken at 3.0 V and never written back anywhere). THE SEQUENCER HAS TWO FACES AND A HANDSHAKE - a bitmap scanned in numeric order or eight ordered slots, and neither is in force until ISR.CCRDY rises, with an ADSTART before that IGNORED - and there is NO current-channel register at all, so `selected()` is the driver's own memory and is exact precisely because the util sampler selects one channel at a time. Measured: VDDA 3310 mV with no meter and the junction temperature 32.6 C once the raw reading is rescaled from the supply it was taken at to the 3.0 V the calibration assumes (the step 15.9's formula leaves out); CONVERSION TIME EXACT TO THE CPU CYCLE in six configurations, the DMA counting 256 conversions so no reader is in the loop; the oversampler's full scale as table 79 draws it, with the noise floor measured FIRST (4 and 7 counts) and the reduction claimed only against it; and A FORBIDDEN WRITE IS NOT ONE THING ON THIS CONVERTER - CFGR1 takes an AWD1 bit with ADEN set (the door ES0548 2.6.2 comes through) while AWD2CR ignores the identical write in complete silence, which is how the watchdogs became disabled-state verbs; the THRESHOLDS alone stay live (15.7.4). Errata: 2.6.2 answered structurally and staged, 2.6.3 reproduced with a control, 2.6.4 measured at one cycle, 2.6.1 a timing obligation no driver can enforce, 2.6.5 revision A only. WAIT and AUTOFF are measured (a converter nobody reads OVERRUNS, one with WAIT stalls on its result; AUTOFF costs 700..900 ns of start-up a conversion), all eight triggers fire once each - TIM1's TRGO2 needing no new verb because MMS2 resets to the UPDATE event, TIM1_CC4 being a COMPARE and not an EGR strobe - and the external inputs are a MAP measured on twelve free pads, each following its own pad and not its neighbour's. Not covered: a known VOLTAGE on any pad but PA4, because a precharged pad cannot be one (an ADC's sample-and-hold is a capacitor of the pad's own order and SHARES its charge, where a comparator's input takes none), PLLPCLK as the async root, VREF+ as anything but the supply |
| [dac.md](dac.md) | DAC (STM32G0): the two 12-bit channels of RM0444 ch. 16 - absent entirely on the G031/G041, which the header says by declaring no DAC1_BASE - with the fact neither chapter states: THE DAC REACHES THE ADC THROUGH THE PAD AND NOTHING ELSE, because the ADC's own connectivity figure lists no DAC among its nineteen inputs while DAC_MCR's internal path goes to the COMPARATORS. So PA4, which is DAC1_OUT1 and ADC_IN4 at once, is a zero-length wire and the shape of every wireless analog experiment on this part. Also: DHR is not DOR (16.4.5, and `code()`/`output()` are two verbs for that reason), a trigger is an EDGE whose first datum must precede it (16.4.8, which is why a DMA stream's launch block is one entry out of phase - measured), and a wave generator without a trigger is refused because 16.7.1 says the bits are only used with TENx set. Measured: the transfer curve monotonic and straight to 2 counts of 4096 END TO END THROUGH THE ADC, reported as the pair's nonlinearity and not apportioned; THE BUFFER CANNOT REACH THE RAILS and turning it off says by how much (46/3276 mV buffered against 3/3308 unbuffered on a 3312 mV supply); the three data formats proven to be PLACEMENTS of one datum; and LD4 measured NOT to be a load the DAC's buffer can feel, the two channels agreeing to 5 mV at every code, which contradicts the obvious guess about how that lamp is wired. All nine triggers are run one event at a time (every TIMx_TRGO, both LPTIM outputs, and EXTI 9 through a pull-walked pad), and THE DMA UNDERRUN is caught: a converter asking for a DMA with no channel armed raises DMAUDR on the trigger after the first and carries it to the shared vector, once per unserved trigger - a flag that has to be read with the interrupt OFF, since a handler that serves it clears it. Not covered: the wave generators from a HARDWARE trigger (they run one software step at a time), what sample-and-hold costs, the dual holding registers as a stream |
| [vref.md](vref.md) | Voltage reference (STM32G0): chapter 17's buffer, implemented whole and NEVER ENABLED. This target's `brio::Ref` and `ref_mv()` live here rather than in the ADC, because unlike the SAM C21 this family really does have ONE shared rail - the ADC, the DAC and the comparators' scaler all work against VREF+ - so the enum sits in the chapter that owns it. THE BUFFER STAYS OFF because 17.1 forbids driving VREF+ where a board ties it to VDDA and NOTHING INSIDE THE CHIP CAN TELL THE TWO APART: `enable()` refuses unless the caller states, in the config and by name, that the pin is free, and table 91's other off mode (VREF+ pulled down to VSSA) is not offered at all, being a path from the supply to ground on such a board. Measured: THE BLOCK IS BEHIND SYSCFG'S CLOCK GATE, which chapter 17 never mentions and only the address map reveals (VREFBUF_BASE = SYSCFG_BASE + 0x30) - and read through the closed gate VREFBUF_CSR answers 0x0, which is not its reset value but the pulled-down mode, so 5.2.17 bites here in the one place where its answer is not merely absent but WRONG; with the gate open the register reads 0x2, the safe external-reference default, and the factory trim is loaded (36). Not covered: the buffer itself at either scale, hold mode, and every converter against a reference that is not the supply - all of which need the board's schematic, not code |
| [comp.md](comp.md) | Comparators (STM32G0): three on the G0B1 class, two on the G071, none on the G031, each one CSR living inside the SYSCFG block so RCC_APBENR2.SYSCFGEN is what makes it readable at all - which settles 18.3.3's two contradictory sentences by the address map. THE ASYMMETRY THAT SHAPES THE BENCH: the inverting input reaches VREFINT, its three taps and BOTH DAC channels with no pad, while the non-inverting one is a pad and nothing else (tables 93/95/97 carry no internal signal), and 7.3.13 disconnects a pad's own pull the moment it goes analog - so a threshold is free and a SIGNAL is not, and the SAM's pull-walked analog stimulus has no twin here (measured: 960 and 1056 counts for a pull-up and a pull-down). What replaces it is A PRECHARGED PAD, driven to a rail by GPIO and then handed over, whose node holds past the 100 ms cap the letter puts on it. Also found: WINMODE's partner is NOT n + 1 (COMP1 borrows COMP2's input, COMP2 borrows COMP1's, COMP3 borrows COMP2's - 18.6.1 one register at a time); THE BLANKING WINDOW IS A REAL GATE measured with no wire (TIM1's OC4 forced active drives VALUE low and releasing it gives the answer back, once the source rises AT a live comparator and the timer's MOE is up); a WINDOW COMPARATOR WITH ONE PAD through WINMODE; and the output counted twice over, 5 EXTI interrupts on line 17 and 5 TIM1 captures through TIM1_TISEL, from six rail changes. ONE HONEST DECLINE: WINOUT did nothing measurable - not to CSR.VALUE, not to the EXTI line - with the bit written and standing and the pair made to disagree by inverting the partner's polarity; recorded, not explained, and the wire that would settle it is named. Not covered: the offset and the hysteresis levels (declined with the reason: a released pad on this board holds a third of the supply), WINOUT, TIM1_OC5 as a blanking source, COMP3's PE7, LOCK (offered, never set), the low-power modes |
| [rtc.md](rtc.md) | RTC + backup registers (STM32G0): the clock that outlives everything else - a power domain of its own that a system reset does not reach, that Stop, Standby and Shutdown do not stop, and that TWO INDEPENDENT LOCKS guard on the way in (PWR_CR1.DBP over the whole domain, the RTC's own 0xCA/0x53 key over most of its registers, and the two have different reach - the backup registers are behind the first and not the second, measured). RTCSEL is ONE-WAY until BDRST, and BDRST costs the calendar, the alarms and the five backup words - which this board needs, because its domain comes up on LSI. Errata as code: ES0548 2.9.1's workaround is applied unconditionally on the way OUT of initialization mode, so a second entry is safe by construction. THE NUCLEO'S X2 CRYSTAL IS FITTED AND RUNS (32703 Hz against the core, the core's own 1 % trim and not the crystal's), and LSI weighs 32586 Hz - confirming from a period capture what reset.md derived from an IWDG time-out. Findings no chapter carries: what TISEL calls "RTC wake-up" is the masked interrupt LINE and not a pulse, so a late acknowledgement DELETES the next interval; an unfiltered capture of an internal clock line is not a measurement at all; and THE SMOOTH CALIBRATION DOES NOT REACH THE DIVIDED RTCCLK WAKE-UP CLOCKS - 1022 ppm of swing on ck_spre where 30.3.13 promises 975, and exactly ZERO on RTCCLK/16. An alarm lands AT its match (the SAM's counter compare does not), the calendar's every boundary is exact in one second each, and the five backup registers plus the calendar survive a real reset. RTC_REFIN drags the calendar onto an outside reference built by the CORE on a pad (a second measured at 64131086 core ticks on the crystal and 64006056 against a 50 Hz reference whose fifty periods ARE 64000000), with a HALTED reference giving the crystal's second back - so the correction is an edge and not a mode - and it is quantized at one ck_apre period. RTC_SHIFTR is timed as a LENGTH (a delay of 64/256 makes one second exactly a quarter longer; SHPF stands for 47 ms). TAMPER DETECTION is built and run, and two facts shaped it: TAMP_CR1 comes out of a domain reset with FOUR INTERNAL TAMPERS ARMED (0xFFFF0000, none with a NOERASE bit), and ARMING A TAMPER INPUT TAKES THE PAD, pulls and all - so no program on this board can put an EDGE on one, and the filtered detector is measured over a pad the BOARD holds, where the latency from the ARMING is the filter's own N/f. Not covered: the output pads, RTCSEL = HSE/32, a tamper on a controlled edge (measured impossible here), the LSE CSS armed (its enable reads one-way) |
| [pwr.md](pwr.md) | PWR + the three sleep sites (STM32G0): the STOPPING half of the platform - chapter 4 whole, and `util/power.hpp`'s depth ladder realized here with the model UNCHANGED. The sleep mode is TWO REGISTERS IN TWO PLACES (the Cortex's SLEEPDEEP and PWR_CR1.LPMS), which is why `Stm32g0Platform::idle()` needed no change at all - the one target of the three whose idle hook was already right. THE LADDER IS NOT THE IDENTITY AND SAYS SO: `light` maps to Sleep because the only thing between Sleep and Stop is Low-power sleep, which needs the whole program at 2 MHz; and STANDBY AND SHUTDOWN ARE OFF THE LADDER ON PURPOSE, because the power model is built on the program RESUMING and those two come back through the reset vector. Measured: a Stop entered with the kernel's 1 kHz tick armed LASTS on a bare board (250 ms of 250) and is cut to 1 ms by a debugger's DBGMCU_CR.DBG_STOP - a bit OpenOCD sets at every connection and that survives every reset but a power-on, which is how it was first recorded as a silicon fact (the site pauses the ticker for the deep rungs anyway, which costs nothing and makes the Stop last in both states); a Stop drops SYSCLK to HSISYS and the site's `resume_clock()` puts it back; kernel time stands still for the whole sleep; a 500 ms deadline matures at 723 ms of wall with the plain site and at 501 ms with the TIMED one, six repeats of 150 ms all landing at 150..151 and NOT ONE EARLY; ES0548 2.2.4 proven NOT to reach an RTC wake (250 ms of Stop with HSIDIV at /4); and Standby and Shutdown both indistinguishable afterwards - PWR_SR1 reads 0x8100 for both and RCC_CSR names NO reset source at all. Under the TICKLESS platform the plain site is the only site and meets a 500-tick deadline through a Stop 1 with time running (488 ms of wall, 500 ticks), the two timed sites refused at compile time. The per-rung wake latency is DECLINED with the reason: every counter fine enough to resolve it stops in Stop, and the RTC's own tick is 30 us against a datasheet 5.6. Not covered: the VBAT charger, the BOR levels (option bytes), SLEEPONEXIT, Low-power run as a rung - and sleep CURRENT, which needs a meter |
| [lptim.md](lptim.md) | Low-power timers (STM32G0): RM0444 ch. 26 whole, on the two instances that are NOT copies of each other - table 135 gives encoder mode to LPTIM1 alone and figure 271's footnote gives LPTIM2 one input channel, while the device header declares ENC, UP/DOWN and IN2SEL once for the struct both share, so what an instance IS comes from the manual with its citation. THE ENABLE RULES ARE OPPOSITE ON THE TWO HALVES of the block (CFGR/CFGR2/IER disabled-only, CMP/ARR enabled-only, the start bits discarded by hardware when disabled) and a write to CMP or ARR is not finished when the store returns, so the STORE and the OBSERVATION are separate verbs. Both errata are answered by construction: 2.8.1 means NO VERB EVER CLEARS ENABLE (`disable()` is an RCC reset pulse), and 2.8.2 means `clear_flags()` refuses from thread mode while any interrupt is enabled while `isr()` clears the disabled-interrupt flags first. Measured, wireless, with all four LPTIM1 signals on pads walked by their own internal pulls: A PAD HANDED TO AN INPUT ALTERNATE FUNCTION STILL FOLLOWS ITS PULL; A FORBIDDEN CFGR WRITE LANDS ANYWAY (the register takes it - what the chapter forbids is what the counter then does with it, so the refusal has to be the driver's) while a start written to a disabled timer really is discarded; CFGR2 KEEPS EIGHT BITS, settling 26.7.9's two-bit drawing against the header's four-bit fields; the waveform arithmetic 26.4.10 never prints (period ARR + 1, high time ARR - CMP + 1, and a FLAT LOW at the forbidden CMP >= ARR); 26.4.12's lost edges are EXACTLY FIVE; the output rate ceiling of kernel/2 counted by a DMAMUX request generator with no pad, no peripheral and no CPU; all three encoder sub-modes exactly as table 144 says; and THROUGH A STOP only LSE and LSI keep counting - HSI16 STOPS, because a counter that merely counts makes none of the clock REQUESTS 5.3 lists it among. A compare match is a PER-LAP event, not a one-shot. The tickless kernel timebase over it, `LptimTicker`, is platform.md's; its letter x found that THE WRITE LATENCY SCALES WITH THE PRESCALER (2..3 prescaled counts, not kernel clocks) and that CMPM fires at CMP + 1. LPTIM2 IS MEASURED ON ITS OWN THREE PADS (PC0, PC3 and PD6 at AF2): its own CCIPR field on all four clocks, its waveform, both counting arrangements, table 56's other trigger input, the two tables' asymmetries as refusals both ways, TRGFLT as a threshold in its own right - and ONE VECTOR WITH TWO OWNERS, LPTIM2's ARRM and TIM7's update served on the same line in the same window. Not covered: the TAMP trigger rows (arming one erases the backup registers), set-once's discarded triggers, debug_freeze |
| [crc.md](crc.md) | CRC calculation unit (STM32G0): RM0444 ch. 14 whole and every field of it bench-verified - a MONOSTATE, one per part, with no interrupt, no request line and NO ERRATUM. One data register takes words, right-aligned half-words and right-aligned bytes and returns the running result; the polynomial is programmable at four widths with two rules 14.3.3 states and no register enforces (an even polynomial, and one wider than the size it declares), both refused before a register is touched. `configure()` ENDS IN A RESET because 14.3.3 forbids changing the polynomial mid-calculation, and `feed(span)` assembles words BIG-ENDIAN - a 32-bit write is processed most significant byte first - falling back to byte-by-byte under the two reversals that cross a byte boundary, so the verb means one thing whatever is configured. Measured: all three of 14.3.3's REV_IN examples are exactly what the silicon does, verified against a checksum the unit really computed; a polynomial changed mid-calculation gives the checksum of nothing, but UNRELIABLE DOES NOT MEAN RANDOM (the same illegal sequence twice gives the same number); the four catalogue check values over "123456789" at all four widths; `crc_ccitt_false_config` IS util/crc.hpp's CRC-16 bit for bit over 4 KB, by two mechanisms sharing only their definition; 2056 cycles per 1000 bytes through the word path against the bitwise loop's 109216, a factor of 53; and 16 KB of the program's own flash checksummed three ways - CPU-fed, DMA-fed at 36.5 MB/s with no CPU in the loop, and a software reference - to one number. util/crc.hpp is NOT hooked to it: that is a util design question this stratum must not open |
| [i2c.md](i2c.md) | I2C (STM32G0): RM0444 ch. 32 whole, both roles, in ONE image on ONE board over the Nucleo's own self-link (I2C1 host PB8/PB9 to I2C2 client PA11/PA12, two wires and their pull-ups) - and `I2cHost`'s Request is the other two targets' VERBATIM, so `util/i2c_bus.hpp` and `util/bus_master.hpp` run here exactly as written. THE TIMING REGISTER IS THE CHAPTER and its arithmetic goes both ways: a chooser that solves 32.4.5's inequalities and a readback that prices a register value, the second pinned against every cell of tables 172..174. THE ENABLE PROTECTION IS REAL HERE - a raw write with PE set lands on NONE of the four PE-gated fields - which is the exact opposite of what the same manual's SPI chapter does. Measured: the silicon NAMES ITS OWN COLUMN of table 165 (an instance without SMBus forces TIMEOUTR to zero, so a write that reads back is the peripheral saying which column it is in: I2C1 yes, I2C2 yes, I2C3 no); a REPEATED START MUST BE ONE CR2 STORE with START in it, because raising AUTOEND first is read as a request to end the transfer that TC has just finished and puts a STOP on the wire where the restart belonged; RXNE MUST BE SERVED BEFORE STOPF (they stand together and RXNE clears only by reading RXDR, so the other order loses the last byte AND storms the vector); a NACK branch must RETURN, or the general sweep below it clears the STOPF the tenure is waiting for; a held SDA is a PARK and not an error, with BUSY standing and no ARLO - every silicon of this project answers that way, and the reason the per-bus timeout is the arbiter's; ES0548 2.10.1's 4/10/20 MHz floors beat the datasheet's own table 74 in every mode, so AT A 2 MHz CORE NO SPEED IS LEGAL ON PCLK and the same bus runs byte-exact with the instance's kernel on HSI16 - which is what the independent clock is for; and the wake from Stop and Fm+ are MUTUALLY EXCLUSIVE, the wake accepting HSI16 alone while Fm+ needs 20 MHz. The standard's worst-case edges make a real bus run FAST (this wire's own tSYNC is 438 ns, so a nominal 100 kHz is 105263 Hz until the measured budget is handed back, when it is 99690). Also: the three speeds bracketed between the register's own floor and the standard-edge prediction, clock stretching linear and free of the data, NOSTRETCH's underrun sending 0xFF, RELOAD past 255 in one tenure, the PEC pinned against a bitwise CRC-8, WHOSE hold each of the three SMBus time-outs polices (with a control on each side: its own, never a peer's - the samc21 answers the same way), 10-bit addressing with ADDCODE proven to be the HEADER, OA2's mask as a real range, and `I2cBus` with its per-bus timeout answering a wedged tenure at exactly 20 ms with SDA still low; not covered: the wake on silicon (both ends of this bus stop together - it wants a second node), the target half of the PEC, the SMBus alert, a live arbitration race |
| [spi.md](spi.md) | SPI / I2S (STM32G0): RM0444 ch. 35 whole, both roles, in ONE image on ONE board over the Nucleo's own self-link (SPI1 host to SPI2 client, four wires) - and `SpiHost`'s Request is the other two targets' VERBATIM, so `util/spi_bus.hpp` and `util/bus_master.hpp` run here exactly as written. THE SILICON ENFORCES NONE OF 35.5.7's CONFIGURATION RULE: a raw write with SPE set lands on all seventeen CR1/CR2 fields, the four whose register description says "only when the SPI is disabled" included, so the driver's refusals are the only protection there is - and 35.9.2's "Not used" data sizes really are FORCED to 8-bit rather than ignored. The disable is 35.5.9's PROCEDURE and the only way this driver turns the peripheral off, which makes ES0548 2.12.1 (REPRODUCED: a raw SPE clear with three frames in the FIFO leaves BSY standing) a workaround by construction. Measured: all eight BR codes byte-exact to PCLK/2 = 32 MHz when the host PACES the bus, and a burst loop whose frame period FLOORS at ~320 CPU cycles - this core cannot make a continuous SPI clock out of software, so the ceiling is found only with both ends on DMA (monotone, byte-exact to PCLK/4 = 16 MHz); a polled burst loop ERASES THE EVIDENCE OF ITS OWN OVERRUN, its poll being 35.5.11's clear sequence; A PAD THAT IS NOT AN ALTERNATE FUNCTION READS LOW AT THE NSS INPUT, so clearing SSM on a master whose NSS is a GPIO raises MODF at once; hardware NSS output falls 77 cycles after the SPE store and frames the PERIPHERAL'S LIFETIME while NSSP frames a DATA FRAME (eight pulses for eight frames, counted on EXTI 12) - between them the reason the engine's chip select is a GPIO; the CRC bit-exact against a bitwise reference in both lengths and both directions, with the checksum frame the one NEITHER SIDE WRITES; the LDMA odd-count rule (six frames with the bit clear, five with it set); and I2S1 master to I2S2 slave on three of the same wires, four standards, three data lengths, CHSIDE, the prescaler arithmetic timed and UDR on a slave transmitter - whose flag a poll loop clears by asking |
| [fdcan.md](fdcan.md) | FDCAN (STM32G0): the Bosch M_CAN of RM0444 ch. 36, whole - and the FIRST chapter of this stratum a part can be MISSING ENTIRELY, so `Fdcan<n>` does not exist on a G071 or a G031 while the protocol arithmetic still compiles there. ONE enable, ONE reset and ONE clock divider serve BOTH modules (a reset through FDCAN2 takes FDCAN1's registers down with it, measured), the kernel clock is chosen in RCC_CCIPR2 rather than the CCIPR every other multiplexer of this stratum uses, and the message RAM is a fixed 212-word map whose content at reset is UNDEFINED. Measured on a board with NO TRANSCEIVER, NO WIRE AND NO SECOND NODE, because the chapter supplies its own instruments: internal loop-back needs not one pad, external loop-back puts the frame on the pin, TEST.TX = 01 is a bit-rate meter a DMAMUX request generator counts with no CPU, the RX pad's own pull IS the bus, and the internal timestamp counter counts bit times whether or not anything is on the wire. THE 0xCC PARAGRAPH DOES NOT DESCRIBE THIS PART: 64 distinct bytes make the round trip byte-exact where 36.3.4 promises the tail padded and discarded. Also found: the registers ANSWER through a closed APB clock gate with their true reset values while a WRITE through it lands nowhere; IR.TC is gated by TXBTIE and 36.4.15 does not say so; 36.3.6's own safe read of an overwriting FIFO costs one more message than the overwrite did; the bus-off recovery measures 1421 bit times, which is 36.4.13's note (129 x 11) and not figure 398's (128 x 11); a restricted node sends NOTHING, settling 36.3.4 against 36.4.6; a stuck-dominant line is a wait and not an error; and TEST.TX = 01 is a STROBE and not a level. Errata: 2.13.1 staged in both arms and not reproduced, 2.13.2 unreachable by construction with TXBC as the evidence. Not covered: everything that needs a peer - arbitration, a real acknowledge, a foreign frame - plus the RAM watchdog's fault, MRAF, ARA, the protocol exception and the PLLQ/HSE kernel clocks |
| [vendor/README.md](vendor/README.md) | RM0444, DS13560, ES0548 (with ES0418 and ES0487 for the STM32G071RB and the STM32G031K8) by revision, the vendored cmsis-device-g0 headers and SVD, the bench chip's IDCODE, the errata pass |

## Toolchain

Self-built **arm-none-eabi-gcc 16.2** at `/sw/arm-none-eabi` - the
same compiler, flags and linker discipline as the samc21 project
(`stm32g0/cmake/toolchain-arm.cmake` is that file verbatim:
`CMAKE_SYSTEM_NAME Generic`, `STATIC_LIBRARY` try-compile,
`--specs=nano.specs -nostartfiles`, deliberately NO syscall stubs so
an accidental `_sbrk`/`_write` fails the link). What is ARMv6-M and not
ST lives in the `armv6m/` core stratum, so `nvic.hpp`, `ticker.hpp` and
`delay.hpp` here are the device header plus that core file plus this
family's own facts.

The device headers are vendored: `third_party/cmsis-device-g0/`
(ST's cmsis-device-g0 v1.4.5, every G0 part) and the shared
`third_party/cmsis-core/`. The device is selected ST's way, by a plain
`-DSTM32G0B1xx` define that the umbrella `stm32g0xx.h` dispatches on -
no device-specs machinery, so clangd needs no macro-delta feed.

## Board and build

The bench board is an **ST Nucleo-G0B1RE** (MB1360): STM32G0B1RE
(LQFP64, 512 KB dual-bank flash, 144 KB SRAM), silicon revision Z,
running at **3.3 V**. Verified at the bench, each by its own experiment
and not by the user manual: LD4 on **PA5** (driven over SWD), the
ST-LINK virtual COM port on **USART2 PA2 (TX) / PA3 (RX), AF1** (the
console answers through it), the CPU on **HSI16 through the PLL at
64 MHz**, and the **LSE 32.768 kHz crystal fitted and running** (X2 is
populated on this board: LSERDY rises and the period measures 32703 Hz
against the core, which is the core's own 1 % trim and not the
crystal's - [rtc.md](rtc.md)). NOT yet verified: the user button B1 on
PC13, and the absence of an HSE crystal (X3 is not fitted by default
and the ST-LINK's 8 MHz MCO reaches HSE only through solder bridges -
the HSE root is unbuilt anyway).

`stm32g0/` is its own CMake project, a sibling and peer of `avrdx/`,
`samc21/` and `test/`. Apps are auto-discovered from
`stm32g0/src/apps/*.cpp` - plus `experiments/*/stm32g0/*.cpp` - by
their `// build:` header comment, the other two projects' grammar with
this family's board names (`boards = g0b1re`, the default; an app that
also runs on the other two Nucleos lists `g071rb` and `g031k8`). One
configure targets one part (`STM32G0_MCU`, full part number: it decides
the device define `STM32G0B1xx`, the linker script `ld/<part>.ld`, the
crt `src/glue/startup_<header>.cpp` and the board name, the part's last
six characters - one Nucleo per part on this desk, so the part IS the
board); three parts have presets - the G0B1RE, and the two other Nucleos
below. Every other part of the family is compile-checked by
`brio check stm32g0`, which sweeps every positive TU in
`test/family_stm32g0/` across ALL TWELVE device headers the CMSIS pack
ships and requires every `neg/` TU to fail for the variants its
`// mcu:` line names - see "Family coverage" below.

## Family coverage

The stratum is written for the whole STM32G0 family, both lines:

- the **x1 line** - G031/G041, G051/G061, G071/G081, G0B1/G0C1 - which
  is what the bench chip belongs to (the G0B1 is its superset), and
- the **x0 value line** - G030, G050, G070, G0B0 - each of whose
  headers is a strict subset of its x1 twin: the same IP under the same
  register names, minus a list of peripherals (no LPUART, no LPTIM, no
  DAC, no comparator, no VREFBUF, no TIM2, no PVD/PVM, no programmable
  BOR, no PCROP or securable memory, no CEC/CRS/UCPD/FDCAN).

What differs is read off the DEVICE HEADER and never off a device
name: `brio/stm32g0/device_tables.hpp` (the reserve) probes the
header's own base-address and bit-mask macros and exports what it
finds as constexpr data. In particular THE VECTORS ARE DERIVED FROM
PRESENCE: an interrupt line's enumerator NAME lists what shares it
(`USART2_LPUART2_IRQn`, `TIM6_DAC_LPTIM1_IRQn`, `ADC1_COMP_IRQn`,
`DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn`...), a header declares the
shared spelling exactly when it declares the sharer, and IRQn values
are enumerators the preprocessor cannot probe - so each vector verb
asks for the sharer's base macro and names the enumerator that
follows. A header whose naming did not follow the rule would fail to
compile, never bind a wrong line in silence.

An absence is spelled one of two ways, and the docs say which:

- a peripheral the part has not got is a driver that DOES NOT EXIST
  there - `Dac`, `Comp<n>`, `Vref`, `Lptim<n>` and its sleep site,
  `Lpuart<n>`, `Fdcan<n>` compile their register-facing half only where
  the header declares the block, while their vocabulary (enums, config
  structs, validity and arithmetic) is every part's; spelling the
  driver on such a part is a compile error naming the reason, and
  `Tim<2>` on a value-line part is the "no such timer" refusal;
- a REGISTER or BIT a present block has not got is a verb that REFUSES
  at run time and a constexpr flag that says so at compile time:
  `Pwr::has_pvd` / `has_sram_retention` / `has_sampled_supply_monitor` /
  `has_dac_supply_monitor`, `Flash::has_debug_gate`,
  `FlashOptions::has_programmable_bor` / `has_shutdown_reset_option` /
  `has_nrst_mode`; the reserve's `flash_pcrop_capable` and
  `flash_securable_capable`.

The sweep proves it on every header the pack ships (22 positive TUs x
12 headers, and every negative refused on each variant it names); the
bench proves two of them.

## The Nucleo-G071RB

The **Nucleo-G071RB** (STM32G071RB, LQFP64, 128 KB single-bank flash,
36 KB SRAM, the same board layout as the G0B1RE's - LD4 on PA5, the VCP
on USART2 PA2/PA3) is tied to the G0B1RE by the six-wire bus link the
two bus suites' header comments describe. Its die reports
**DEV_ID 0x460, REV_ID 0x2000** - ES0418 silicon revision B - and every
bench suite prints that pair at boot through `DeviceIdcode::read()`
([platform.md](platform.md)), because a measurement that differs between
two boards is only a finding once the die it was taken on is on the
record.

**FOURTEEN OF THE SEVENTEEN SUITES RUN ON IT**, and the same fourteen
on the Nucleo-G031K8 below. An app that builds for more than one board says so
in its `// build: boards = g0b1re,g071rb,g031k8` line; what a suite
cannot do on a board it SKIPS BY NAME, printing the reserve's own fact
and claiming no verdict, so a smaller count is a shorter list of claims
and never a weaker one.

| Suite | G0B1RE | G071RB | What skips there, and why |
|---|---|---|---|
| `test_stm32_crc` | 27 | **27** | nothing - the one driver that needs no fact from the reserve |
| `test_stm32_platform` | 53 (+ `i` 26) | **53** (+ `i` 26) | nothing |
| `test_stm32_sleep` | 50 | **50** | nothing in `z`; letter `u` measures a DIFFERENT Shutdown wake ([pwr.md](pwr.md)) |
| `test_stm32_exti` | 89 | **89** | nothing; IMR1's reset value differs ([exti.md](exti.md)) |
| `test_stm32_rtc` | 125 | **125** | nothing in `z`; two tamper inputs instead of three, and arming one does not take its pad ([rtc.md](rtc.md)) |
| `test_stm32_lptim` | 82 | **82** | nothing |
| `test_stm32_tickless` | 47 | **45** | letter `i`'s awake-time meter needs TIM2's ETR taking MCO, which this part's ETRSEL has not got |
| `test_stm32_clock` | 42 | **40** | letter `h` (`delay_us` at 20 us) needs that same 4 us wall; the rest runs on an LSE wall instead |
| `test_stm32_tim` | 118 | **114** | TIM4: its counter, its four channels on PB6..PB9, and TIM3's shared vector |
| `test_stm32_dma` | 69 | **65** | DMA2's five channels, and letter `m`'s peripheral-to-peripheral destination |
| `test_stm32_analog` | 139 | **136** | COMP3's register block and its whole signal path |
| `test_stm32_serial` | 88 | **83** | LPUART2, and four verdicts to ES0418 2.2.4 ([usart.md](usart.md)) |
| `test_stm32_spi` | 38 (peer) | **38** (peer) | the same self-link letters that skip on the G0B1RE; SPI3 and SPI2's I2S are absent |
| `test_stm32_i2c` | 54 (peer) | **54** (peer) | the same self-link letters; I2C3 is absent and I2C2 has no independent clock |
| `test_stm32_nvm` | 85 | - | one flash bank: no storage attic ([nvm.md](nvm.md)) |
| `test_stm32_journal` | 52 | - | the same |
| `test_stm32_fdcan` | 96 | - | no FDCAN on this part ([fdcan.md](fdcan.md)) |

The two bus suites run with the OTHER board as their peer, roles
exchanged, on the same six wires.

WHAT THE PART HAS NOT GOT, all of it read off the header by the reserve:
TIM4, USART5, USART6, LPUART2, I2C3, SPI3 (and with it SPI2's I2S), DMA2
(DMA1 keeps seven channels and the DMAMUX seven), COMP3, FDCAN, USB, CRS,
GPIOE, the second flash bank, the EXTI's second-group TRIGGER registers,
WKUP3, TAMP_IN3, PWR's PUCRE/PDCRE, and `RCC_CCIPR_I2C2SEL` - which is
what takes I2C2's independent clock, its SMBus and its wake with it. Its
USART3 is BASIC where the G0B1's is FULL, so it has one wake line fewer;
its MCOSEL and MCOPRE are three bits rather than four. The ETRSEL list
is a per-part fact of its own in the reserve, `tim_etrsel_has_mco()`,
because RM0444 22.4.25 gives codes 0100, 0101 and 0110 to the G0B1/G0C1
sales types alone and no TIM register says so ([tim.md](tim.md)).

All three boards have their presets (`stm32g0b1re-*`, `stm32g071rb-*`,
`stm32g031k8-*`), linker scripts, crts and `bin/brio` board types,
and `blink`, `console` and `probe` build for all three.

## The Nucleo-G031K8

The **Nucleo-G031K8** (STM32G031K8, a Nucleo-32: **LQFP32, 64 KB
single-bank flash, 8 KB SRAM**, LD3 on PC6, the VCP on USART2 PA2/PA3)
carries the six bus wires to the Nucleo-G0B1RE. Its die reports
**DEV_ID 0x466, REV_ID 0x1003**.
Its **LSE crystal runs** (LSERDY in about 900 ms at the lowest drive,
32719..32753 Hz against the core) with the oscillator bridges at
UM2591's default, so the RTC, the tickless timebase, the timed sleep
sites and every wall run on the crystal exactly as on the Nucleo-64s
and the six suites those touch are configured identically for all three
boards; its **LSI** weighs 31403..31496 Hz on a TIM16 capture and 31400
by the watchdog, the slowest of the three dies. Two board facts shape
suites on it: **PA0 and PA4 lean HIGH when left floating** (the
Nucleo-64s' free pads drift down), which costs the tamper letter its
TAMPPUDIS contrast; and **its debug port can go silent** - a state of
the board's ST-LINK half that only unplugging the board clears - in
which case the ST-LINK's own mass-storage flasher is the way in
(`bin/brio`'s `stlink_msd` programmer kind) and nothing can be
halted or read over SWD until the replug ([../probes/st-link.md](../probes/st-link.md)).

**FOURTEEN OF THE SEVENTEEN SUITES RUN ON IT.**

| Suite | G0B1RE | G071RB | G031K8 | What skips on the G031K8, and why |
|---|---|---|---|---|
| `test_stm32_crc` | 27 | 27 | **27** | nothing - the boards line is the whole change |
| `test_stm32_platform` | 53 (+ `i` 26) | 53 (+ `i` 26) | **53** (+ `i` 26) | nothing; the LED moves to PC6 and LSI reads 31400 Hz |
| `test_stm32_sleep` | 50 (+ `s` 6, `u` 6) | 50 (+ `s` 6, `u` 6) | **50** (+ `s` 6, `u` 6) | nothing; a Shutdown wake is a literal power-on reset here as on the G071 ([pwr.md](pwr.md)) |
| `test_stm32_exti` | 89 | 89 | **85** | IMR2's reset value (no second register group at all) and the user button (PC13 is not bonded) |
| `test_stm32_rtc` | 125 (+ `w` 11, `v` 4) | 125 | **109** (+ `w` 11, `v` 4) | RTC_REFIN (PB15 unbonded), two tamper legs whose level is a board's, and the TAMPPUDIS contrast (a free pad that leans high) ([rtc.md](rtc.md)) |
| `test_stm32_lptim` | 82 | 82 | **78** | the two comparator routes, the comparator trigger row and the two-owner vector (no COMP, no TIM7); the LSE rows are measured |
| `test_stm32_tickless` | 47 | 45 | **45** (+ `u` 1) | the awake-time meter, which needs TIM2's ETR taking MCO |
| `test_stm32_clock` | 42 | 40 | **40** | letter `h`: the 4 us wall needs TIM2's ETR taking MCO, a G0B1 code; the LSE code is the wall here, as on the G071 ([clock.md](clock.md)) |
| `test_stm32_tim` | 118 | 114 | **108** | TIM4, TIM6, TIM7 and TIM15 - four instances the part has not got - and PB13..PB15, which the package does not bond |
| `test_stm32_dma` | 69 | 65 | **62** (+ `u` 3, `w` 0) | DMA2, the console's DMA-fed rate (five channels, and the letters own all five) and the peripheral-to-peripheral leg |
| `test_stm32_analog` | 139 | 136 | **67** | the DAC's letters and the comparators' - this part has neither block, so eight of the eighteen letters are compiled out ([adc.md](adc.md)) |
| `test_stm32_serial` | 88 (+ y/w/v) | 83 | **79** (+ `y` 1, `v` 2, `w` 0) | USART2 is BASIC here (no kernel-clock multiplexer, no wake), LPUART1's only bonded pads are the console's, and there is no LPUART2 |
| `test_stm32_spi` | 38 (peer) | 38 (peer) | **38** (peer) | the eleven self-link letters, COMPILED OUT: SPI2's four pads are bonded to no pin of the LQFP32, so that instrument cannot exist here at any wiring |
| `test_stm32_i2c` | 54 (peer) | 54 (peer) | **54** (peer) | the eleven self-link letters, skipped on the PROBE's answer like everywhere else - this package does bond I2C2's PA11/PA12, so what rules them out is the desk and not the plastic |
| `test_stm32_nvm`, `test_stm32_journal` | 85, 52 | - | - | one flash bank: no storage attic |
| `test_stm32_fdcan` | 96 | - | - | no FDCAN |

**64 KB AND 8 KB ARE PART OF THE DESIGN HERE.** Every image on that
board fits with the cuts named in its own suite: the largest is
`test_stm32_serial` at **61928 bytes of 65536**, then `test_stm32_i2c`
52676, `test_stm32_lptim` 50336 and `test_stm32_dma` 42392; the
hungriest in RAM is `test_stm32_dma` at **6108 bytes**, which leaves the
stack the 2 KB reserved for it. One suite makes a real cut rather than
losing letters: `test_stm32_dma` gives its three memory-to-memory
buffers 256 words there instead of 512 (every claim restated for the
length that fits) and gives the console's two DMA channels back to the
letters. `test_stm32_spi` compiles its eleven self-link letters out - a
package fact, not a budget one - and comes to 31104 bytes with the peer
half in; `test_stm32_i2c` keeps every letter it has anywhere.

**WHAT THE PART HAS NOT GOT**, all of it read off the header by the
reserve: TIM4, TIM6, TIM7, TIM15, USART3..6, LPUART2, I2C3, SPI3, DMA2
(DMA1 keeps five channels and the DMAMUX five with four generators),
**the DAC and every comparator**, FDCAN, USB, CRS, UCPD (so PA8 and PB15
carry no dead-battery Rd), GPIOE, the second flash bank, the EXTI's
entire second register group, WKUP3 and WKUP5, `RCC_CCIPR_I2C2SEL` and
`RCC_CCIPR_USART2SEL` - the last of which is what takes USART2's kernel
clock, its FIFO and its wake with it. And what the PACKAGE does not bond
is a separate list the SUITES own, never the reserve: PB10..PB15,
PC0..PC5, PC7, PC13, PD0..PD3, PF0 and PF1 (GPIOD is declared by the
header and reaches no pin at all). One per-part fact in the reserve
covers that difference: `ucpd_present(n)`, probed on SYSCFG's own
dead-battery strobe bit, so a pad that will not follow its pull can be
told from one that never had an Rd on it.

**Vector names across the boards.** The crt spells ST's own handler
names, and a SHARED line's name changes with what shares it - USART2's
is `USART2_LPUART2_IRQHandler` on the G0B1 and `USART2_IRQHandler` on
the G071/G031. An app for one board binds the bare name; an app for
more than one binds the name the reserve derives from the header's
presence macros, `BRIO_STM32G0_USART2_HANDLER` and its siblings (TIM3,
TIM6, TIM7, TIM16, TIM17, LPTIM1, LPTIM2, USART3, LPUART1, the DMA's
upper line, the ADC) - `test/family_stm32g0/handlers.cpp` proves every
one expands on every header and names the line the reserve's verb
answers. A name outside that set on the wrong board lands in
`Default_Handler`'s silent spin.

```bash
(cd stm32g0 && cmake --preset stm32g0b1re-release)                      # configure (once, or after adding an app)
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>)
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>-upload)
(cd stm32g0 && cmake --preset stm32g071rb-release)                      # the Nucleo-G071RB; stm32g031k8-release the Nucleo-32
brio check stm32g0 [name]                                          # family smoke, no hardware
```

Build outputs land in `build-cmake/stm32g0b1re-{release,debug}` at the
repo root: `<app>.elf/.bin/.hex`, `firmware-<app>.map`, `<app>.lst`. A
configure also writes this project's app roster,
`build-cmake/apps_stm32g0.json`, which `bin/brio` reads: the
board TYPE `g0b1re` is what tells that tool to build here and to flash
through OpenOCD's ST-LINK interface (`brio flash <board> <app>`). NB
`brio run` speaks the bench SUITES' single-letter grammar (no
line terminator): the line-oriented `console` app is driven with any
serial monitor, or pyserial, at 115200 8N1.

## Upload (OpenOCD, ST-LINK)

Flashing goes through OpenOCD driving the Nucleo's on-board
ST-LINK/V2.1: `interface/stlink.cfg` + `target/stm32g0x.cfg` (the
stm32l4x flash driver underneath) + `program <app>.elf verify`, then
`reset run` and a write of DHCSR that clears C_DEBUGEN: a core left with
halting debug enabled HALTS on a BKPT instead of faulting, and every
`panic()` ends in one. OpenOCD - the 0.12.0 release built from its
tarball into `/sw/openocd-0.12.0` (`/sw/openocd`), see
[the SAM page](../samc21/README.md) - drives the ST-LINK (firmware
V2J46M31) without incident; the probe carries a REAL USB
serial, so `adapter serial` names it and the same serial names the
console under `/dev/serial/by-id`. Single-client: close the debug
session before flashing.

ONE SWD CAVEAT worth knowing: memory reads THROUGH THE HLA TRANSPORT
WHILE THE CORE SLEEPS IN WFI ARE UNRELIABLE - a running console
(WFI between events) answered `0xffffffb7` for FLASH_ACR and zeros
for RCC_CR, values those registers cannot hold, while the same reads
after `halt` were exact. Halt first, read, resume; the CMSIS-DAP probe
on the SAM C21 board does not do this.

A SECOND SWD CAVEAT, and one that reads as a silicon fact until it is
found: `target/stm32g0x.cfg`'s examine-end hook writes
`DBGMCU_CR.DBG_STOP | DBG_STANDBY` ("enable debug during low power
modes"), and OpenOCD re-examines the target after every reset it
issues, so the bits are back on the reset a `program` ends with
whatever was cleared before it (measured: cleared to 0 while halted,
read 6 again right after `reset run`). The register survives every
reset but a power-on, and with DBG_STOP set the debug logic keeps
HCLK - and SysTick - running inside a Stop, which cuts a Stop entered
with the kernel tick armed from its full 250 ms to one tick.
`bin/brio` therefore ends every G0 flash with `reset halt`, a
clear of DBGMCU_CR through its clock gate (the gate put back to its
reset value) and a `resume`, so a board leaves the bench as a power-on
would leave it; a cortex-debug session sets the bits again, and
`Pwr::debug_in_stop()` ([pwr.md](pwr.md)) is how firmware tells. Both
states are measured in `test_stm32_sleep` letter c.

## Debugging (cortex-debug + OpenOCD)

The launch config is "Debug STM32G0 (OpenOCD, Nucleo-G0B1RE)" in
`.vscode/launch.json`: the SAM entry's shape with the two ST config
files, `adapter serial` through `openOCDPreConfigLaunchCommands`, and
`svdPath` at `stm32g0/svd/STM32G0B1.svd`. CMake Tools' Active Folder
must be `stm32g0/` and its launch target the app to debug. Not yet
exercised at the bench (mature tooling gets the light verification
policy).

## Editor (clangd)

`brio/stm32g0/.clangd` and `stm32g0/.clangd` route the stratum and the
project to `build-cmake/stm32g0b1re-release`;
`test/family_stm32g0/.clangd` lets the script-compiled family TUs
borrow flags from the same database. The repo-root `.clangd` rules
apply unchanged.

## Serial console

The ST-LINK's virtual COM port enumerates with the probe's own USB
serial, so the console is addressed by `/dev/serial/by-id` and never
moves with the socket - the first console on this desk that does not
need the by-path dance. Console apps run 115200 8N1. Measured: BRR 556
at 64 MHz gives 115107 baud (the arithmetic to the hertz), 300 lines
of 50 bytes exchanged with zero errors of any kind, and the kernel
tick +0.24 % against the PC's clock (inside HSI16's 1 % calibration).
