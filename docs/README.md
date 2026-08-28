# brio documentation

Design documentation for the `brio` framework and the multi-app testbed
around it. This directory is the reference companion to the code: the
rationale and the contracts between layers, as they are today.

## Map

The directory mirrors the strata of `brio/`: `design/` is the
target-independent framework (kernel, services, the models every
target realizes); one folder per target (`avrdx/`, `host/`, ...) holds
that target's operational page (`README.md`), one document per
peripheral driver, and its vendor documents. Within each, ordered by
stability - the kernel's ideas are settled enough to build on, the
services and drivers are here to stay but will change with the second
target, the bench is disposable.

Target-independent design:

| Document | Content |
|----------|---------|
| [design/overview.md](design/overview.md) | Philosophy, governing rules, layering, naming and style |
| [design/kernel.md](design/kernel.md) | The active-object kernel, by intent: model, AO contract, events and payloads, queues, FSM, delivery, scheduler, time, panic, platform - with C++ notes |
| [design/clock.md](design/clock.md) | The clock model: one rate truth, static and dynamic regimes, the synchronous rebase fan-out and its two compile-time checks |
| [design/serial.md](design/serial.md) | The serial stack: Uart driver below, SerialPort line events above |
| [design/spi-bus.md](design/spi-bus.md) | The shared SPI bus: engine descriptor, AO arbitration, multi-device rules |
| [design/i2c-bus.md](design/i2c-bus.md) | The I2C bus: BusMaster generalized, the TWI engine descriptor, status vocabulary |
| [design/nv-heap.md](design/nv-heap.md) | NvHeap (provisional): runtime-allocated blocks of flash that outlive the program - the FlashMedia contract, the ping-pong map pair and its atomicity, survival-aware mount, the placement rule, the wear accounting |
| [design/power.md](design/power.md) | The power model: the sleep-depth ladder, the site that only arms (so the kernel loop's own idle path does the sleeping), the round of votes among stakeholders, standing restrictions for the ones that live in interrupts, the deadline guard, and the first-event-after-wake contract |
| [design/ring.md](design/ring.md) | Ring: the SPSC FIFO, lock-free where the platform allows, guarded elsewhere |
| [design/analog.md](design/analog.md) | Analog in the kernel: the AnalogSampler usage type (converter concept, attribution by reported code, two paces, owner's duties) and the counts <-> mV arithmetic |
| [design/meters.md](design/meters.md) | Meters in the kernel: the MeterLatch that bridges a capture interrupt to the loop (last value wins, overwrites counted) and the MeterSampler that paces PUBLICATION instead of capture - a stale source publishes nothing |
| [design/architecture.svg](design/architecture.svg) | The strata diagram |

Target AVR DA/DB (`brio/avrdx/`):

| Document | Content |
|----------|---------|
| [avrdx/README.md](avrdx/README.md) | Toolchain, board, Atmel-ICE upload, PyAvrOCD debugging and its quirks, clock/timebase |
| [avrdx/platform.md](avrdx/platform.md) | Platform (provisional): what the kernel stands on - `AvrPlatform` (critical section, idle and the SLPCTRL erratum, timebase, atomic_width, the `.noinit` breadcrumb), the short-wait role with both delay paths measured, `Sleep`/`Vreg` over all three sleep modes and the regulator plus `AvrSleepSite`, the adapter that carries the power model onto them, and `Reset`/`Watchdog` over RSTCTRL and the WDT; not covered: idle detection and the RUNSTDBY policy, the BOD, the sleep current, and the wake-up sources a lone board cannot produce |
| [avrdx/clkctrl.md](avrdx/clkctrl.md) | CLKCTRL (provisional): oscillators, PLL, main clock mux/prescaler/CLKOUT, clock failure detection (DB) as resources; Clock/DynamicClock as tasks, DA external clock datasheet-trusted; not covered: the unbenched paths (XOSC32K, PLL, DA silicon) |
| [avrdx/evsys.md](avrdx/evsys.md) | EVSYS (provisional): the full typed vocabulary (generators and users, package-gated) + run-time connect/disconnect; not covered: static allocation, the vocabulary no driver exercises yet |
| [avrdx/vref.md](avrdx/vref.md) | VREF: the reference selector - levels, headroom, how the ADC/DAC name it |
| [avrdx/dac.md](avrdx/dac.md) | DAC: the 10-bit actuator - buffered/unbuffered outputs, the slow fall on a bare pin, usage |
| [avrdx/adc.md](avrdx/adc.md) | ADC (provisional): one task with knobs - inputs as types, triggers, accumulation, window (signed too), results as events, DB-only inputs gated; not covered: pin-level input legality, the standby paths |
| [avrdx/nvm.md](avrdx/nvm.md) | NVMCTRL (provisional): the `Nvm` resource - the four memories and what each is for, the Flash sections as a compile-time claim cross-checked against the fuses, ELPM/SPM only (never the data-space window), the command model, the one-way protections, the vector-table invariant every image carries, and the multi-page-erase erratum guarded page by page - plus the services over it (typed record, interrupt-paced writer AO, persistent panic record); not covered: a boot loader, any flash policy, DA silicon |
| [avrdx/userrow.md](avrdx/userrow.md) | USERROW: the board identity label - survives chip erase, written once over UPDI, read into every suite banner by board_id() |
| [avrdx/port.md](avrdx/port.md) | PORT (provisional): Port<L> resource, Pin (one-store PinConfig, senses, flags), PinSet across ports on the multi-pin engine, PinRef; not covered: INLVL/slew measurements, the fully-async wake |
| [avrdx/usart.md](avrdx/usart.md) | USART (provisional): the `Usart<n>` resource - routes incl. the pinless one, every frame format, the receiver modes, the errata verbs - and the tasks over it (`Uart`, `OneWire`, `Rs485`, `SyncHost`/`SyncClient`, `MspiHost`, `IrdaLink`, `AutoBaud`); not bench-verified: everything that needs a second board |
| [avrdx/spi.md](avrdx/spi.md) | SPI (provisional): the `Spi<n>` resource - the per-package route table with the errata that beats it, both roles, the seven rates, both INTFLAGS layouts and their clear disciplines, the host demotion - and the tasks over it (`SpiHost` transfer engine, `SpiClient`); not bench-verified: everything that needs a second device on the wire |
| [avrdx/twi.md](avrdx/twi.md) | TWI (provisional): the `Twi<n>` resource - the per-package route table with its dual pin pairs, the three errata as code, the chapter's own baud arithmetic with the bus's edges as arguments, both halves - and the tasks over it (`TwiHost` transfer engine, `TwiClient` including Dual mode); not bench-verified: everything that needs a second, independent device on the wire |
| [avrdx/rtc.md](avrdx/rtc.md) | RTC/PIT (provisional): RtcClock/Rtc/Pit resources (one clock for both functions, counter with period and compare, crystal error correction, the busy flags) + BasicTicker over the PIT; not covered: the crystal and external-clock sources, the standby and debug-run paths |
| [avrdx/tca.md](avrdx/tca.md) | TCA (provisional): the Tca resource (normal mode, buffered compares, event inputs, commands) + tasks TcaPwm/TcaPwm16/FrequencyGenerator/Heartbeat/EventCounter; not covered: the split halves' counters as verbs |
| [avrdx/tcb.md](avrdx/tcb.md) | TCB (provisional): the Tcb resource (eight modes, event clock/capture, cascade, routes) + tasks PeriodicTick/Timeout/OneShotPulse/PulseCounter/CascadedCounter/meters/Pwm8; not covered: pin-level bonding within a port |
| [avrdx/tcd.md](avrdx/tcd.md) | TCD (provisional): the `Tcd<0>` resource - the full chapter with its three synchronization disciplines enforced by the verbs, the per-package route table, the input-mode validity table and the errata that shrink it, the 12-bit captures and their read discipline, dithering - plus `TcdPwm`, the complementary pair with dead time, and the PLL made observable through it; not covered: the usage types waiting for their first user, the external clock source, two errata that would not reproduce on this die |
| [avrdx/ccl.md](avrdx/ccl.md) | CCL (provisional): Ccl + Lut<n> resources (inputs menu, truth table, filter/edge, clocks, pins, the whole-block reconfiguration erratum) + ToggleFlipFlop; not covered: typed per-input instance legality |
| [avrdx/ac.md](avrdx/ac.md) | AC (provisional): the Ac<n> resource (inputs and DACREF, hysteresis/power, pin/event/interrupt, window) + Threshold/Window; not covered: pin-level bonding (PD0, PC6 on small packages) |
| [avrdx/opamp.md](avrdx/opamp.md) | OPAMP (provisional, **DB only**): the `OpampSystem` block (the one ENABLE, the TIMEBASE that makes a settle time mean microseconds, and the ClockUser hook that keeps it true across a rebase) + `Opamp<n>` - both input multiplexers with their per-instance link codes, the 16R ladder and its eight exact gains, the output driver, the three enable regimes, the internal timer and READY, the four event users and the offset trim - plus the tasks `OpampFollower`, `OpampPga`, `OpampInvertingPga` and the chapter's three-op-amp `InstrumentationAmp`; not covered: the integrator usage type (external R and C, and a DUMP policy), RUNSTBY, IRSEL's electrical effect |
| [avrdx/vendor/README.md](avrdx/vendor/README.md) | The datasheets/errata the stratum is written against, by document number (PDFs kept local, not in git) |

Target SAM C21 (`brio/samc/`):

| Document | Content |
|----------|---------|
| [samc/README.md](samc/README.md) | Toolchain (vendored DFP/CMSIS, no device headers in arm-none-eabi-gcc), board, OpenOCD upload over SWD, cortex-debug, the clangd routing, the family smoke check |
| [samc/platform.md](samc/platform.md) | Platform (provisional): `SamPlatform` (PRIMASK critical section, WFI idle that takes whatever PM.SLEEPCFG holds, BKPT and its ARMv6-M caveat, the `.noinit` breadcrumb, atomic_width 4), `BasicTicker` over SysTick, `Nvic`, the hand-written crt - AND THE STOPPING HALF: PM ch. 19 (`Pm`, three sleep modes, the readback rule, the regulator and back-bias knobs) with `SamSleepSite` giving util/power.hpp its second silicon, unchanged. Measured: the SLEEPCFG bridge latency is ~5 us so the readback rule is not a formality; IDLE0 costs nothing to leave and IDLE2 costs 3.5 us more, which the chapter does not say; STANDBY costs ~17 us and NOTHING in the regulator or back-bias settings moves it (this family has no separate regulator bill, unlike AVR's 290 us); THE KERNEL TICK FREEZES in standby, exactly (a 499 ms sleep advanced it by 0), which is why the v1 rule is 'no armed time event, then standby'; a peripheral's OWN RUNSTDBY is the whole clock request, the generator's and the source's being unnecessary; and XOSC keeps running through standby whatever RUNSTDBY says, so there is no crystal-restart bill here at all. Not covered: sleep CURRENT (no meter on this bench), an RTC-backed timebase that would lift the standby restriction, EIC wake, PAC |
| [samc/clock.md](samc/clock.md) | Clock (provisional): ALL THREE OSCCTRL roots as resources - OSC48M, the external crystal with its clock-failure detector, and the fractional DPLL with the LDR/LDRFRAC arithmetic and an actual-hertz readback - plus GCLK/MCLK, bounded waits everywhere, flash wait states ordered around the change, and five errata as code (two OSC48M, three DPLL). Measured: the internal RC is 5100 ppm SLOW against the board's 24 MHz crystal, which is the first non-RC scale this stratum has had and corrects every absolute frequency measured before it; a clock failure induced with no wire and the safe-clock switch observed; the DPLL's ratios exact to the count at 48, 49 and 96/2 MHz; the CPU run from the crystal-locked DPLL and brought back; and GENCTRL's DIVSEL settled as 2^(DIV+1), correcting an earlier note in this file. Not covered: the main-clock TASK still implements `internal` only, DynamicClock, external-clock mode, sleep |
| [samc/port.md](samc/port.md) | PORT (provisional): two groups on every variant, the off-by-default input buffer, the directional pull, the PMUX handoff, WRCONFIG; not covered: PORT events (senses and pin interrupts are another peripheral - see samc/eic.md) |
| [samc/sercom.md](samc/sercom.md) | SERCOM USART (provisional): the one-vector reality and the INTFLAG-AND-INTENSET discipline, pads vs pins (TxD on PAD0/PAD2 only), the LSB-first default and its measured cautionary tale, enable-protection and both sync waits, the 16x-arithmetic baud math byte-exact on the wire, `Uart` with the AVR edge-return contract and its two OPTIONAL DMA engines (zero when absent, measured); not covered: the SPI/I2C personalities, the USART long tail |
| [samc/dmac.md](samc/dmac.md) | DMAC (provisional): the selector-guarded channel registers and the INTPEND dispatch that needs no selector, the SRAM descriptor tables, the end-address quirk decided by data against the data sheet's own disagreement, the silent disable/SWRST edge, harvest as the one window into progress, erratum 1.10.4 measured and VALIDATED AGAINST rather than trusted, the serial engines; not covered: the CRC engine, linked lists, the event hooks, standby |
| [samc/reset.md](samc/reset.md) | RSTC + WDT (provisional): RCAUSE as the EXCLUSIVE one-cause register it is (the AVR's accumulating RSTFR habit does not travel), what each reset does and does NOT clear, the watchdog whose reset values are fuses and whose CLEAR key resets whether it runs or not, the early-warning interrupt used to measure OSCULP32K by difference, and the panic breadcrumb proven across six real resets including a HardFault; not covered: SUPC behind two of the causes, always-on mode (one-way), VTOR |
| [samc/nvm.md](samc/nvm.md) | NVMCTRL (provisional): the two arrays and the one that does not stall the CPU (measured: ~3950 polling turns survive an RWWEE row erase, ONE survives a main-array one), erase-by-row/program-by-page, the page-buffer rule measured NARROWER than the chapter's ('one 64-bit section at a time' - descending loads are exact too, interleaved ones lose half the page), ADDR proven section-relative, region locks, the fuse row and the factory calibration typed, and `RwweeFlash` giving util/nv_heap.hpp its second target; not covered: writing the user row, the security bit, a main-array backend |
| [samc/ac.md](samc/ac.md) | AC (provisional): the four comparators, the VDD scaler, filters, both output routings, WINDOW MODE over the comparator pairs, and both event directions with the EVSYS codes published here (the SOC users being asynchronous-path-only) - plus per-package input legality, since the PAIR owns the pads and the E/G variants bond only AIN[5:4] for COMP2/3. Measured: the answer ch. 40 does not give - a synchronized output edge costs the fraction to the next GCLK_AC edge PLUS TWO whole periods - all three window states reached on a board with no analog source, a comparator flip and a window transition each moving a DMA block, and A PIN EDGE STARTING A COMPARISON through the AC's SOC0 user; not covered: sleep, the DAC and bandgap inputs (they need drivers this stratum has not got) |
| [samc/evsys.md](samc/evsys.md) | EVSYS (provisional): THE ONE PLACE THE AVR SHAPE DOES NOT TRANSFER - twelve identical channels with numeric generator/user codes, an ALLOCATOR where the AVR has a typed table, so this driver owns the fabric and each peripheral publishes its own codes. The user multiplexer's channel+1, the ordering rule, the three paths and the path/edge legality both ways, three live errata as code; proven end to end by a DMA transfer that only an event can start - and measured: a SOFTWARE event does not cross an asynchronous channel, which the chapter does not say; not covered: the generator/user tables (deliberate), SleepWalking |
| [samc/eic.md](samc/eic.md) | EIC (provisional): where this family keeps its PIN INTERRUPTS, since PORT has none - sixteen lines plus an unmaskable NMI, the pad-to-line map read out of the device header because it is irregular and per-package, the five senses, the majority filter, asynchronous detection, one NVIC vector for all sixteen, and every line an EVSYS generator. Five of the six errata are NOT this silicon (the trap of reading the column). Measured: a line that asks to be SAMPLED cannot even be ENABLED without a clock while a clockless one can, a HARDWARE generator DOES cross an asynchronous EVSYS channel where a software event does not, all sixteen lines generate events whatever 26.6.7's prose says, and the crt's core-exception vectors were spelled the CMSIS way rather than the device header's; not covered: the N-variant debouncer, sleep/wake |
| [samc/tc.md](samc/tc.md) | TC (provisional): five timers, the three counter resolutions with the 32-bit PAIRING and the SHARED generic clock channels read out of the device header, the four waveform modes, capture with its event actions, and READING COUNT AS A COMMAND (READSYNC, not a load). Erratum 1.20.3 as code (clear a buffer-valid flag TWICE); 1.20.1 and 1.20.2 are revision B only. Tasks give util its second implementation: PwmChannel and the capture meters behind MeterSource. Measured: the prescaler ratio exact to 4.00x, COUNT32 nine million counts past a 16-bit range, PWM duty read off the pad and its FREQUENCY read by a second timer counting its events, capture exact at 2344/937 ticks, reading CCx proven to BE the acknowledgement, a SYNCHRONIZED READ PROVEN ONE BEHIND (four consecutive reads of a running counter gave 0, 196, 201, 205), and a MeterSampler AO running live inside the kernel with NOT ONE LINE of util/ changed; not covered: the N-variant capture modes, DMA, ONDEMAND's clock request (RUNSTDBY is now measured and is the load-bearing bit of the whole chain) - and erratum 1.20.2 deliberately not judged, since no controlled edge can reach a muxed WO pad with no wires |
| [samc/tcc.md](samc/tcc.md) | TCC (provisional): the family's richest timer, and THREE INSTANCES THAT ARE NOT COPIES OF EACH OTHER - 24/24/16-bit counters, 4/2/2 channels, 8/4/2 outputs and five optional waveform-extension units apportioned per instance, all of it read out of the device header's TCCn_* constants through the reserve. The pad map is keyed by pad AND FUNCTION (PA08 is TCC0/WO0 under E and TCC1/WO2 under F). Dead-time insertion, the output matrix, swap, pattern generation, ramps, dithering and BOTH fault systems, whose inputs ARE the event inputs. Seven of the chapter's eleven errata are live: 1.21.10 is a compile-time refusal (ALOCK is dead, and the bench proves it), 1.21.6 is code, 1.21.11 is unreachable, three are stated caller obligations - and 1.21.8 DID NOT REPRODUCE. Measured: dead time exact to a stopwatch tick and independent of the prescaler, the dual-slope formula right where the AVR TCD's was wrong, a fault raised from a pin clamping an output, MSYNC moving the CHANNELS and not COUNT - and two facts no chapter carries: a buffered write's SYNCBUSY stands until the UPDATE takes it (so the setters refuse instead of waiting) and a CCx read returns the BUFFERED value while the pad shows the old one; not covered: DMA, sleep, the debug fault, the advanced capture modes |
| [samc/osc32kctrl.md](samc/osc32kctrl.md) | OSC32KCTRL (provisional): the three 32 kHz roots, the RTC's clock select (which lives here and not in the RTC), and the crystal's failure detector - with the fact the chapter insists on and the bench confirms, that OSC32K IS NOT CALIBRATED UNTIL SOFTWARE CALIBRATES IT (44% fast without its production trim, six per mille with it); also the GCLK rule a generator cannot be moved off a STOPPED source. Neither of the two errata touching the chapter applies to this silicon; not covered: XOSC32K (no crystal on the board), the CFD event output |
| [samc/freqm.md](samc/freqm.md) | FREQM (provisional): the hardware ratio counter - f_msr = VALUE/REFNUM x f_ref between two GCLK generators - with erratum 1.24.1 as code (reading CTRLB is a PAC error on every revision, so START is written and never read), the 24-bit overflow budget, and two documentation findings decided by data: CFGA has no DIVREF whatever 44.8.3 draws, and the channels must be routed BEFORE the software reset or its synchronization never completes. Cross-checks OSCULP32K against the watchdog measurement to 3 Hz; not covered: the DONE interrupt as a wake source, GCLK_IO inputs |
| [samc/supc.md](samc/supc.md) | SUPC (provisional): the two brown-out detectors, the regulator that cannot be turned off, and the bandgap every analog block asks for - with the BODVDD whose reset value is a FUSE (cross-checked field by field against nvm.hpp's user row), its enable-protection-plus-synchronization dance, and VREF's VREFOE, which is what ac.md's gap list was waiting for. Measured: this board runs at about 5.1 V, located three ways through the comparator's VDD scaler against the 1.024/2.048/4.096 V references; the BODVDD level step is 48.7 mV, settling table 45-18 against its own stated 60 mV; a store carrying configuration AND enable together leaves the protected fields untouched; a sampled detector never reports ready; SUPC_BODCORE is real and running at an offset ch. 22 marks Reserved; and erratum 1.5.6's spurious comparator flag observed. Not covered: nothing forces a brown-out (the supply is not a program's to dip), standby, BODCORE stays read-only by design |
| [samc/rtc.md](samc/rtc.md) | RTC (provisional): one counter wearing three faces - a 32-bit counter, a 16-bit one with PER as its top, and a clock/calendar with a masked alarm - behind three overlaid register views, with THE CLOCK CHOSEN IN ANOTHER CHAPTER (OSC32KCTRL.RTCCTRL, which this driver never writes) and the read synchronization that is off at reset. Erratum 1.16.3 is live and answered STRUCTURALLY (no verb writes COUNT or CLOCK in pieces); 1.16.1 and 1.16.2 are B and B..E items, and the bench confirms the second by behaviour. Measured: the counter counts its source tick for tick on all four clock selects against the board's crystal, the prescaler exact to 70..300 ppm, PRESCALER = OFF dividing by one while silencing every periodic event, a compare AND a periodic event each moving a DMA block, MATCHCLR raising the compare and the OVERFLOW together, mode 1's period proven to be PER + 1, the calendar's leap rule and its year-63 wrap, an alarm that arrives A WHOLE SECOND after its match, and a synchronized read that trails the counter by a CONSTANT four ticks while an unsynchronized one is simply frozen; not covered: tasks of any kind (deliberate), sleep, and the frequency correction's per-step linearity, which is smaller than this board's RC wander and is declined rather than guessed |
| [samc/adc.md](samc/adc.md) | ADC (provisional): BOTH 12-bit converters over the whole chapter - the reference multiplexer (this target's `brio::Ref` and `ref_mv()`, the vocabulary util/analog.hpp asks every target for), the prescaler and sampling arithmetic with table 45-22's cycle counts as constexpr, the RESSEL/AVGCTRL interplay whose full-scale arithmetic IS util/analog.hpp's `steps`, the window monitor, the digital corrections, the sequencer, both event directions and the DMA trigger. The FACTORY CALIBRATION is copied by init(), keeping the promise nvm.hpp's comment has carried since its own campaign. Five of the ten errata are live: 1.4.4 is CODE (start_on() refuses a non-asynchronous channel), 1.4.5 is why start() waits for nothing, 1.4.6 is five discarded conversions, and 1.4.10 IS LIVE AND WORSE THAN ITS OWN SENTENCE (once ADC1 has run, ADC0 will not enable at all and no reset clears it - the errata's ADC1-first order is the way out). Measured: THE BANDGAP CHANNEL IS DEAD WITHOUT SUPC's VREFOE (a flat zero until the bit is set, which no chapter connects) while the bandgap as a REFERENCE needs no such thing; VDD located at 5201..5276 mV from three bandgap levels, agreeing with the comparator's independent answer to under 2 %; ADC0 and ADC1 EXACT on a shared pad at both rails and 6 counts of 4096 apart on an internal divider; conversion time EXACT TO THE TICK against the crystal in six configurations; MODE4 of the window monitor proven to be the COMPLEMENT of MODE3, settling the device header against 38.8.10's own table; the correction's 13-cycle single-conversion latency NOT OBSERVED though the correction itself is exact; and util/analog_sampler.hpp running inside a real kernel UNCHANGED on its second architecture. Not covered: the host/client pair, the sequencer, sleep, differential mode, VREFA, and INL/DNL as this converter's own numbers |
| [samc/dac.md](samc/dac.md) | DAC (provisional): the whole of a small chapter as a MONOSTATE resource (one instance on every variant), and the session that closes the analog loop - the three references with `DacRef` as this converter's OWN vocabulary, table 41-1's four data placements as arithmetic, the two outputs, the START/EMPTY event pair, the DMA trigger, and the refusals (a Reserved REFSEL, dithering without the start event 41.6.8.3 requires). THE GEOMETRIC GIFT: VOUT, ADC0/AIN0 and the AC's AIN4 are all PA02, so erratum 1.8.9's "wire the DAC output to an ADC input" has ZERO LENGTH and the whole suite is wireless. Measured: the device header's `INT1V` name is the SAM D21's and wrong - the internal reference follows SUPC.VREF.SEL (996/2011/4057 mV at its three levels); neither the DAC's nor the ADC's reference path needs VREFOE, closing adc.md's open question; `Ref::dac` converts and is ratiometric to half a per cent; `AdcInput::dac` reads the DAC to one count in 4096 though 38.8.9's table marks the code Reserved; ERRATUM 1.8.9's OUTPUT noise is real and large (3 counts of spread becoming 71..87 while another converter samples the DAC internally, and 3 again with that converter on any other input - the control that makes it the erratum and not crosstalk) while its READING half is DECLINED as beneath this board's noise floor; `AcNegative::dac` flips a comparator at 255/512/769 against 255/511/767 predicted with gaps of exactly 257 and 257; the transfer curve monotonic with a worst residual of 2.5 counts, reported as the two converters' COMBINED nonlinearity and deliberately not apportioned; startup 5.9 us; a full-scale step crossing mid-supply faster than a comparator poll can resolve (an upper bound of a few hundred ns against the 2857 ns the conversion RATE implies); a no-CPU waveform (timer event in, DMAC feeding DATABUF, EMPTY counted by a second timer) with UNDERRUN when it runs dry; SYNCBUSY.DATABUF proven NOT to be a bus crossing (it stands until a start event consumes the value, and every later write is discarded) which is why buffer() never waits; and erratum 1.9.2 reproduced with a control on both sides. Not covered: dithering, VREFA, the voltage pump, sleep beyond RUNSTDBY, the SDADC's share of everything |
| [samc/vendor/README.md](samc/vendor/README.md) | The datasheets/errata the stratum is written against, the bench chip's silicon revision, the targeted errata pass (PDFs kept local, not in git) |

Target host (`brio/host/`):

| Document | Content |
|----------|---------|
| [host/README.md](host/README.md) | The native test target: HostPlatform, doctest suites |

The bench:

| Document | Content |
|----------|---------|
| [bench.md](bench.md) | The board, the wiring and the apps as they are today (volatile) |

## Rules of this directory

- **Keep it true or delete it.** A design doc that lags the code is
  worse than no doc. Whoever changes a documented decision updates the
  doc in the same change (same commit when practical).
- **Docs hold the WHY and the contracts; headers hold the API.** The
  canonical reference for any type or function is its header comment -
  do not duplicate signatures or parameter lists here, link to the
  header instead. If a browsable API reference is ever wanted, Doxygen
  over the headers generates it without touching this directory.
- **First principles only in design/ and the target folders.** They
  state principles, contracts and tradeoffs - they never describe or
  reference individual apps (apps are disposable and must be free to
  change without touching the foundations). Apps document themselves
  in their own header comment and are listed in `bench.md`.
- **One folder per target, mirroring `brio/<name>/`.** Its
  `README.md` is the operational page - toolchain, probe, debugger,
  quirks, clock fixture; next to it one document per peripheral driver
  and `vendor/` with the datasheets of record. What is
  target-independent (models every target realizes) stays in design/;
  a reader of `docs/<target>/` sees at a glance what is that
  target's.
- **Plain Markdown, ASCII only, English** (project-wide rules). No
  generator-specific syntax: every file must render on GitHub as-is.
- **MkDocs-ready by construction.** If/when a website is wanted:
  `pip install mkdocs-material`, drop a 10-line `mkdocs.yml` at the
  repo root pointing at this directory, `mkdocs serve`. Nothing here
  needs rewriting for that - which is exactly why nothing here may
  depend on it.
- **Every peripheral driver has its document; only incomplete ones
  are marked.** A driver that does not yet cover its chapter's full
  option space opens with a PROVISIONAL banner and closes with "Not
  covered yet" - the chapter's features the driver does not implement
  (driver gaps) kept distinct from what is implemented but not yet
  bench-verified. A complete doc carries no banner and no gap list:
  absence of the banner IS the statement of completeness. Never mark
  a doc complete while it still lists gaps.
- **One document per peripheral, in this shape.** First paragraph:
  the documents of record with their revision (data sheet, errata),
  the driver header, the reference test suite - no chapter lists, no
  history. Then: *what the silicon does* (the behaviour and the
  physical facts that matter to code, measured ones marked as such),
  *types and verbs* (a systematic inventory of the configuration knobs
  - name, values, default, effect - the input/resource types, and the
  verbs by purpose: names and meaning, never signatures), *how to use
  it* (one example per way of using it - what to write, since readers
  want the call, not the header), *bench findings* (the
  facts the test suite established, with its name). Tracks, guiding
  applications and history live in CLAUDE.md and memory, never here.
- **Today's truth only, no change history.** A doc says what is,
  never what it used to be, when it was reorganized or what something
  was called before. Rationale and rejected alternatives are welcome
  (they are the WHY); dates, renames and "since ..." notes are not.
  When the project earns versioning, changelogs will be their own
  documents.

## What does NOT belong here

- Bench diary and hardware bring-up state beyond the current wiring
  (`bench.md`): session memory.
- The assistant's working notes: `CLAUDE.md`.
