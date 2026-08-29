# TCC - Timer/Counter for Control Applications (SAM C21)

> **PROVISIONAL.** The whole chapter is built and bench-verified except
> the advanced capture actions, the debug-fault state and sleep. Two
> errata are recorded as UNREPRODUCED rather than disproved - 1.21.7 and
> 1.21.8 - and 1.21.5 is stated and not exercised. The list is in "Not
> covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 36 - and
errata DS80000740S items 1.21.1 to 1.21.11, of which **seven are this
silicon**, the largest live errata list of any chapter this stratum has
touched. Driver: `samc/tcc.hpp`, with its per-instance and per-pad data
in `samc/device_tables.hpp`. Family fixture `test/family_samc/tcc.cpp`
plus eight negatives under `tools/check_samc.sh`; the bench suites are
`test_samc_tcc` and - for DMA-driven operation, the circular buffers and
the advanced modes - `test_samc_timer_dma`.

## What the silicon does

**Three instances, and they are NOT copies of each other.** That is the
first thing separating this chapter from the TC's, where five identical
timers differ only in which pads they reach.

| | counter | CC channels | outputs | GCLK channel | extension units |
|---|---|---|---|---|---|
| TCC0 | 24 bit | 4 | 8 | 28 | all five |
| TCC1 | 24 bit | 2 | 4 | 28 | pattern, dithering |
| TCC2 | 16 bit | 2 | 2 | 29 | none |

Every number in that table is a `TCCn_*` constant of the device header,
probed in the reserve (`samc/device_tables.hpp`) and consumed by
`Tcc<n>`; none of it is spelled out in the driver. `TCCn_EXT` is the
header's own one-word summary of the five optional units, and the bench
confirms the decoding: OTMX, DTI, SWAP, PG, DITHERING from bit 0 up, so
31 / 24 / 0 for the three instances. **TCC0 and TCC1 share generic clock
channel 28** (36.5.3), so neither can run at a rate the other does not.

**The pad map is keyed by pad AND peripheral function.** A TC waveform
output lives on exactly one function, so `TcWo<Pin>` needs no more than
the pad. A TCC output does not: **PA08 is TCC0/WO0 under function E and
TCC1/WO2 under function F**, PA16 is TCC2/WO0 under E and TCC0/WO6 under
F, and the two are different outputs of different instances. Hence
`TccWo<Pin, PinFunction::f>`, and the header's own
`PIN_P<pad><fn>_TCC<n>_WO<k>` as the only authority - a pair the package
does not bond does not compile.

**The period register is always there**, unlike the TC's, and PER is TOP
in every waveform mode but MFRQ, where CC0 is (table 36-2). So a TCC PWM
channel has an arbitrary period at any counter width, which is what makes
`TccPwm<T, ch, top>` a `PwmChannel` whose full scale the caller chose.
Seven waveform modes: two frequency modes, single-slope PWM, and four
dual-slope flavours differing only in where the interrupt and event land.

**Reading COUNT is a command, not a load** - CTRLBSET.CMD = READSYNC,
then wait out SYNCBUSY.CTRLB and SYNCBUSY.COUNT, then read (36.6.7),
exactly as in the TC - AND ISSUED TWICE, exactly as in the TC: the
COUNT shadow lands about half a counter-clock period after SYNCBUSY
clears with no bit advertising it (measured on both timers by
`tc_readsync_probe`; the mechanism and the numbers are in
[tc.md](tc.md)), so a single-command read returns the PREVIOUS
command's snapshot and `read_sync()` pays the second crossing to
return the count at the call's entry. `count()` does it; `count_raw()`
skips it and says so.

**Enable-protection and write-synchronization split differently here
than in the TC, and the difference is useful.** Enable-protected
(36.6.2.1): CTRLA except RUNSTDBY/ENABLE/SWRST, FCTRLA, FCTRLB, WEXCTRL,
DRVCTRL, EVCTRL - the driver refuses those while enabled. Write-
synchronized and **not** enable-protected: CTRLB, STATUS, COUNT, PATT,
**WAVE**, PER and CC/CCBUF. So the waveform mode, the polarities, the
swaps and the circular-buffer bits can all change under a running timer,
where the TC's WAVE cannot - and the bench uses exactly that to swap a
complementary pair live.

**The waveform extension is four stages between the compare unit and the
pins** (36.6.3.7), in this order: the output matrix (OTMX) distributes
compare channels across outputs; dead-time insertion splits matrix output
*x* into a low side on WO[x] and an inverted high side on WO[x + N/2]
with an OFF time between them; SWAP exchanges the two sides of a pair;
and pattern generation overrides any output with a constant level. Each
is optional per instance, and asking an instance for one it does not have
is a compile error.

**The fault system has two halves, and the fault inputs ARE the event
inputs.** Recoverable faults A and B are the channel event inputs MCE0
and MCE1: they clamp CC0's or CC1's output, and can additionally halt,
restart, blank, qualify, filter and timestamp. Non-recoverable faults are
the counter event inputs TCE0 and TCE1 with EVACT = FAULT: they stop the
counter and force **every** enabled output to a level DRVCTRL names. Both
must be carried on ASYNCHRONOUS event channels - 36.6.3.5 says so for the
fault use and erratum 1.21.9 for every use, and the bench measured both
halves of that.

**Dithering shares the low bits of COUNT, PER and CCx.** With
CTRLA.RESOLUTION = DITH4/5/6 the low 4, 5 or 6 bits stop being part of
the value and become the number of extra clock cycles to spread over a
frame of 16, 32 or 64 PWM cycles (36.6.3.3). `tcc_dither()` packs the
pair; a value written without it while a dithering resolution is selected
is off by a factor of 16, 32 or 64, silently.

**Two facts the chapter does not carry at all**, both measured here and
both shaping the driver's API - a buffered write's SYNCBUSY bit stands
until the update CONSUMES the buffer, and the two halves of CTRLB are not
a set/clear pair for the command fields. See "Bench findings".

## The errata: read the row, not the column

Eleven items, and the E/G/J row at revision F is what decides. **Seven
are live**, which is the most of any chapter in this stratum.

| item | what | here |
|---|---|---|
| 1.21.1 Circular Buffer | needs an APB host awake in standby | rev B only |
| 1.21.2 RAMP2 Mode | double restart on a fault at a ramp end | rev B only |
| 1.21.3 CAPTMARK | cannot mark two faults on one channel | rev B only |
| 1.21.4 Capture Overflow | overflow with no INTFLAG.ERR within 3+3 clocks | rev B only |
| 1.21.5 Advance Capture | CAPTMIN/MAX, LOCMIN/MAX, DERIV0 need every UPPER channel in one of those modes | **live** |
| 1.21.6 SYNCBUSY | clearing STATUS.xxBUFV releases SYNCBUSY early | **live** |
| 1.21.7 Dithering Mode | dithering plus external retrigger events distorts pulses | **live** |
| 1.21.8 PERBUF | LUPD said not to protect PER while counting down | **live**, did not reproduce |
| 1.21.9 EVSYS SYNC/RESYNC | the TCC is not compatible with a synchronous channel | **live**, measured |
| 1.21.10 ALOCK | the auto-lock feature is not functional | **live**, measured |
| 1.21.11 Overflow DMA trigger | OVF in DMAOS mode broken for RAMP2C/RAMP2CS | **live**, unreachable |

What the driver does with each live one:

- **1.21.10 is a compile-time refusal.** `TccConfig::auto_lock` exists so
  that a caller asking for it gets a compile error naming the erratum,
  and `tcc_config_valid()` never lets the bit be written. It is the one
  item in this chapter that can be turned into a compile error, and it
  is.
- **1.21.6 is code**: `clear_buffer_valid()` clears the flag TWICE, which
  is the whole documented workaround and the exact twin of the TC's
  1.20.3.
- **1.21.11 is out of reach by construction**: it applies only to RAMP2C
  and RAMP2CS, which 36.8.17 marks variant-L, and `tcc_wave_valid()`
  refuses RAMP2C for that independent reason.
- **1.21.5, 1.21.7 and 1.21.9 are stated caller obligations.** 1.21.5 is
  a property of the whole channel set (basic capture on the low channels,
  advance capture on the high ones) that no single register write can
  check; 1.21.7 says not to retrigger a dithering TCC, and a retrigger
  may arrive on an event channel this driver cannot see; 1.21.9 belongs
  to EVSYS, which owns the fabric and not the vocabulary.
- **1.21.8 did not reproduce** - see the bench findings.

## Types and verbs

**Vocabulary** - `TccPrescaler` (div1..div1024, with
`tcc_prescaler_divisor()`), `TccPrescalerSync`, `TccResolution`
(none/dither16/32/64, with `tcc_dither_bits()`, `tcc_dither_frame()` and
`tcc_dither()`), `TccWaveform` (the two frequency modes, `normal_pwm`,
and four dual-slope flavours, with `tcc_waveform_is_dual_slope()`),
`TccRamp`, `TccCommand`, `TccRampIndexCommand`, `TccEvent0Action` and
`TccEvent1Action` (with `tcc_event1_is_capture()`), `TccCountEvent`,
`TccOutputMatrix`, `TccFaultSource`, `TccFaultBlank`, `TccFaultHalt`,
`TccFaultCapture` (with `tcc_capture_is_advanced()`), `TccFault` and
`tcc_fault_input_bit()`.

**The configuration structs**, one per enable-protected group plus WAVE:
`TccConfig` (CTRLA and the CTRLB mode bits), `TccWaveConfig` (waveform,
ramp, polarities, swaps, circular buffers), `TccWaveExtConfig` (output
matrix, dead-time enables and the two dead times), `TccDriveConfig`
(inversion, the non-recoverable output state, the two filters),
`TccEventConfig` (both event actions and every input and output enable),
`TccFaultConfig` (one recoverable fault, whole).

**The refusals** - `tcc_config_valid(n, cfg)` (ALOCK, dithering without
the unit, MSYNC off the client, a capture bit past the channels),
`tcc_wave_valid(n, w)` (RAMP2C, swap without the unit, masks past the
channels), `tcc_wave_ext_valid(n, w)` (the matrix and dead time per
instance, a slice that does not exist), `tcc_drive_valid(n, d)`,
`tcc_fault_valid(n, f)`, `tcc_pattern_valid(n, e, v)` and
`tcc_event_config_valid(n, cfg, ev)`, which holds the rules that live
between the structs.

**Geometry**, all from the reserve: `tcc_count()`, `tcc_cc_count`,
`tcc_wo_count`, `tcc_size`, `tcc_gclk_id`, `tcc_pair_role`, the five
`tcc_has_*` probes, `tcc_ext_code`, `tcc_slice_count`, `tcc_max_count`,
and the DMAC and EVSYS codes.

**`Tcc<n>`** - `index`, `cc_count`, `wo_count`, `counter_bits`,
`max_count`, `gclk_id`, `slice_count`, `has_dead_time`,
`has_output_matrix`, `has_swap`, `has_pattern`, `has_dithering`,
`extension_code`, `is_pair_host`, `is_pair_client`, `irq()`;
`overflow_generator`, `retrigger_generator`, `count_generator`,
`match_generator(ch)`, `event_user(which)`, `match_user(ch)`,
`fault_user(f)`, `dma_trigger_overflow`, `dma_trigger_match(ch)`;
`init(generator)`, `bus_clock`, `clock`, `reset`, `enable`, `release`;
`configure`, `wave`, `wave_extension`, `drive`, `fault`, `event_config`
and their readback twins; `command`, `retrigger`, `stop`, `update`,
`ramp_index_command`, `count_down`, `lock_update`, `one_shot`;
`read_sync`, `count`, `count_raw`, `set_count`; `period`, `set_period`,
`period_buffer`, `set_period_buffer`; `cc`, `set_cc`, `cc_buffer`,
`set_cc_buffer`; `pattern`, `pattern_buffer`, `pattern_reg`; `status`,
`stopped`, `ramp_index`, `is_client`, the buffer-valid readers and
`clear_buffer_valid`, `compare_output`; `fault_input`, `fault_state`,
`clear_fault_state`, `non_recoverable_input`, `non_recoverable_state`,
`clear_non_recoverable`, `debug_fault_state`, `update_fault_state` and
their clears; `sync_busy`, `cc_sync_busy`, `period_sync_busy`,
`pattern_sync_busy`; the flag constants,
`flags`/`clear_flags`/`arm`/`disarm`/`armed`/`isr`; `debug_run`,
`fault_on_debug`.

**`TccWo<Pin, function>`** - `timer`, `output`, `claim()` (the given
function with the input buffer on, so a driven level can be read back),
`release()`. `tcc_wo_code()` and `tcc_wo_exists<>` are the map itself.

**Tasks** - `TccPwm<Tcc, ch, top>`, a `PwmChannel` whose `max` is the
period the caller chose, and `TccPairPwm<Tcc, slice, top>`, the
complementary pair with dead time - one duty value, two pads, and
`low_output` / `high_output` naming which two.

## How to use it

**A PWM channel on a pad**, with the function as part of the pad's
identity:

```cpp
using OutPin  = brio::Pin<'A', 8>;
using OutWave = brio::TccWo<OutPin, brio::PinFunction::e>;   // TCC0 / WO0
using Pwm     = brio::TccPwm<brio::Tcc<0>, 0, 1999>;         // max = 1999

brio::Tcc<0>::init(/* generator */ 0);
OutWave::claim();
Pwm::setup(brio::TccPrescaler::div256);   // 48 MHz / 256 / 2000 = 93.75 Hz
Pwm::duty(500);                           // buffered, taken at the next UPDATE
```

**A complementary pair with dead time**, which is what this chapter is
for. The dead times count UNPRESCALED GCLK_TCC cycles, so they are set in
a different unit from the period:

```cpp
using LowWave  = brio::TccWo<brio::Pin<'A',  8>, brio::PinFunction::e>;  // WO0
using HighWave = brio::TccWo<brio::Pin<'A', 22>, brio::PinFunction::f>;  // WO4
using Pair = brio::TccPairPwm<brio::Tcc<0>, 0, 1999>;

brio::Tcc<0>::init(generator);
LowWave::claim();
HighWave::claim();
Pair::setup(brio::TccPrescaler::div1, /* dead low */ 60, /* dead high */ 180);
Pair::duty(1000);
```

**A recoverable fault raised from a pin.** The fault input IS the
channel's event input, so EVCTRL.MCEIx has to be set for it, and the
channel must be asynchronous:

```cpp
Eic::configure_line(line, {.sense = EicSense::high, .event_out = true});

const TccConfig cfg{};
Tcc<0>::configure(cfg);
Tcc<0>::fault(TccFault::a, TccFaultConfig{
    .source = TccFaultSource::event,
    .halt = TccFaultHalt::hardware,
});
Tcc<0>::event_config(cfg, TccEventConfig{
    .match_in = tcc_fault_input_bit(TccFault::a),   // EVCTRL.MCEI0
});
Evsys::connect(Tcc<0>::fault_user(TccFault::a), channel, EventChannelConfig{
    .generator = ExtInt<Pad>::event_generator,
    .path = EventPath::asynchronous,                // 36.6.3.5 and 1.21.9
});
```

**A non-recoverable shutdown**, which is the other half: the counter
stops and every enabled output goes to the level the application chose.

```cpp
Tcc<0>::drive(TccDriveConfig{
    .fault_output_enable = 0x11,   // WO0 and WO4 driven, the rest tri-stated
    .fault_output_value  = 0x01,   // WO0 high, WO4 low
});
Tcc<0>::event_config(cfg, TccEventConfig{
    .action0 = TccEvent0Action::fault,
    .input0_enable = true,
});
// ... and later, after the input has gone away:
Tcc<0>::clear_non_recoverable(0);
```

## Bench findings

From `test_samc_tcc` (12 letters, **143 verdicts, 143/143 three times**).
Nothing to wire. Five of TCC0's eight outputs reach pads this board
leaves free - PA08 (WO0, function E), PA09 (WO1, E), PA22 (WO4, F) and
PA12 (WO6, F) - and PA16 carries EXTINT0, which is the fault stimulus.
Every timer runs from OSC48M, so the expected numbers are exact
arithmetic and not a measurement of an oscillator. A second TC, free
running at 3 MHz, is the stopwatch; a third counts events.

**The two facts the chapter does not have**

1. **A buffered write's SYNCBUSY bit stands until the UPDATE consumes
   the buffer** - not merely for the clock-domain crossing. Waiting
   SYNCBUSY.CC0 out after writing CCBUF0 took **256 us and 480 us and
   1299 us** on three runs of a **1333 us** period - that is, a random
   fraction of a whole waveform period - and with CTRLB.LUPD set it never
   returned at all (the bounded wait spun out its 65535 turns, 21 ms).
   **A second buffered write inside that window is DISCARDED**: with the
   update locked, the first write returns and lands, the second finds
   SYNCBUSY.CC0 standing and CCBUF0 still holds the first value. So every
   buffered setter in this driver - `set_cc_buffer()`,
   `set_period_buffer()`, `pattern_buffer()` - **refuses instead of
   waiting** and returns at once; false means "the last value has not
   been taken yet", which is the honest answer for a duty set faster than
   the waveform can accept one. Waiting would have put a whole PWM period
   inside `TccPwm::duty()`.
2. **A read of CCx or PER while a buffered write is pending returns the
   BUFFERED value, not the one the waveform generator is using.** With
   the update locked, CC0 reads 250 while the pad still shows the 750 it
   was given before - the register and the pin disagree, and the pin is
   right. This is the trap under erratum 1.21.8: PER reads back the
   buffered 555 in all three modes tried, while the counter is still
   using 1000.
3. **The two halves of CTRLB are not a set/clear pair for the command
   fields.** DIR, LUPD and ONESHOT behave as expected, but CMD and IDXCMD
   are written as a VALUE and "writing zero to this bit group has no
   effect" on *both* halves (36.8.2, 36.8.3) - so a command is issued
   through CTRLBSET and can only be CANCELLED through CTRLBCLR. Found
   with a RAMP2 index that stayed held forever after a `HOLD` the driver
   thought it had cleared; `command(none)` and `ramp_index_command(off)`
   now go through CTRLBCLR, and the index toggles again (499 per mille
   of a 50 ms window against 1000 while held).

**The counter and the waveform**

4. **The 24-bit counter is genuinely 24 bits and TCC2's is genuinely 16**:
   0x123456 written to COUNT reads back whole on TCC0 and as 0x3456 on
   TCC2, which is 36.8.15's "the excess bits are read zero" from the
   other side. PER is TOP in every mode: with PER = 999 the highest COUNT
   seen over five counter cycles was exactly 999.
5. **The single-slope PWM frequency is GCLK / (N x (PER+1))**: 937
   overflows in one second against 937.5 predicted.
6. **The dual-slope period is 2 x PER counter ticks, exactly as
   36.6.2.5.6 prints it** - and NOT 2 x (PER+1). Measured over two
   seconds, where the two predictions are five counts apart: **942
   against a predicted 942, where 2 x (PER+1) would have given 937**.
   Worth the two seconds: the AVR TCD campaign found its own datasheet's
   dual-slope formula off by exactly that one, and this chapter's is
   right.
7. **Dithering buys a fractional period.** With DITH6 and PER = 99, the
   measured overflow counts over two seconds were **938 / 933 / 928** for
   DITHERCY 0, 32 and 63, against **937 / 932 / 928** predicted from
   PER + DITHERCY/64. An integer counter with a fractional average.

**The waveform extension**

8. **The dead times are exactly the two registers, in unprescaled
   GCLK_TCC cycles.** At a 200 kHz GCLK_TCC with DTLS = 60 and
   DTHS = 180, the two both-low windows measured **899 and 2701 stopwatch
   ticks** against 900 and 2700 predicted - one tick being a third of a
   microsecond. **Prescaling the counter by 4 did not move them** (900
   ticks again), which is 36.8.7's "GCLK_TCC clock cycles" confirmed
   against the obvious alternative.
9. **The pair is complementary and the two outputs are never high
   together**: 0 coincidences in 400000 paired samples. The two duties
   sum to 888 per mille rather than 1000, and the missing 112 is the dead
   time itself (DTLS + DTHS = 240 of 2000 counter ticks).
10. **The output matrix does what table 36-4 draws**: with OTMX =
    per-channel, WO0 shows CC0's 25 % and WO6 shows CC2's 75 %; with
    OTMX = 0x2 both show CC0's 25 %.
11. **SWAP exchanges the two sides of a pair, LIVE.** 150 and 644 per
    mille before, 646 and 152 after - and the write went to a RUNNING
    timer, because WAVE is the one configuration register this chapter
    does not enable-protect.
12. **Pattern generation beats everything upstream of it**: with
    PGE = 0x11 and PGV = 0x01 the two enabled outputs sit at 1000 and 0
    per mille whatever the compare unit is doing. The BUFFERED pattern is
    held while the update is locked and lands at the next update
    condition, which is what makes a commutation step atomic across the
    pins. **STATUS.PATTBUFV lags its own write** where STATUS.CCBUFVx
    does not: 0 immediately after the write with SYNCBUSY.PATT standing,
    1 twenty milliseconds later.

**The fault system**

13. **A pin level, through the EIC and an asynchronous event channel,
    clamps a compare channel's output**: duty 502 free, 0 with the fault
    held, 501 after release, with FAULTAIN, FAULTA and INTFLAG.FAULTA all
    set.
14. **EVCTRL.MCEIx is the gate on a recoverable fault's input**, and
    36.6.3.5 never says it in those words. With MCEI0 clear the same pad,
    the same generator and the same FCTRLA give FAULTA = 0 and an
    untouched waveform.
15. **Erratum 1.21.9 measured, not taken on trust.** The same generator
    on a SYNCHRONOUS channel connects without complaint and does
    nothing - FAULTA = 0, duty 496. The asynchronous path is the only one
    a TCC hears.
16. **The restart action is a retrigger**: INTFLAG.TRG rises on a fault
    exactly when FCTRLA.RESTART is set, and not otherwise.
17. **The hardware halt freezes the counter while the fault is present
    and lets go the moment it is not; the software halt outlives the
    fault** and is released only by writing STATUS.FAULTA. The witness is
    the OVERFLOW FLAG, because **two READSYNC'd reads of a halted counter
    are not a witness at all** - they gave 0 and then 116 across 30 ms of
    a counter that was demonstrably stopped, READSYNC being a CTRLB
    command that has to cross into the clock domain the halt has stopped.
18. **The capture action timestamps the fault** into the channel
    FCTRLA.CHSEL names (CC2 = 153 of a 200-tick period on one run).
19. **A non-recoverable fault forces every enabled output to its own NRV
    level and stops the counter dead**: 1000 and 0 per mille under the
    fault with NRE = 0x11 and NRV = 0x01, 0 and 1000 with NRV = 0x10, no
    overflow in 30 ms of a 1 kHz counter. **The state is a latch, not a
    level**: dropping the input leaves the drivers shut down until
    STATUS.FAULT0 is written, which is what a safety shutdown should do.

**Ramps, capture and the pair**

20. **RAMP2 interleaves two cycles.** The ramp index toggles (high 504
    per mille of a 50 ms window, against 0 in RAMP1) and each output is
    live in only half of the cycles, so a 50 % compare measures 242 and
    243 per mille on the two pads.
21. **PPW capture through the EIC is exact**: a 20 ms / 50 ms wave read
    2344 ticks of period and 938 of width at 46875 Hz, against 2343 and
    937. Unread captures are dropped and raise INTFLAG.ERR while a reader
    that drains CCx keeps up with none.
22. **A TCC counts another timer's events with the prescaler bypassed**:
    18 of TCC2's overflows in two seconds against a 9 Hz source.
23. **CTRLA.MSYNC moves the CHANNELS, not COUNT** - which is exactly what
    36.6.4 says and no more, and not what a reader expecting two counters
    to track would guess. Linked, TCC1's channel-0 matches jump from
    **47 to 937 a second** (the host's cycle rate), while TCC1's own
    overflows stay at **47** throughout. STATUS.SLAVE follows MSYNC.

**Erratum 1.21.8 did not reproduce.** With the overflow rate as the
witness - a register read cannot be one, see finding 2 - CTRLB.LUPD held
the period at 748 overflows a second in **NPWM counting up, NPWM counting
down and NFRQ counting down** alike, where an unprotected PERBUF would
have given 1350. The item is recorded as unreproduced rather than
disproved: the errata document offers no workaround and names no mode
beyond "down-counting", and what the bench CAN say is that the register
read of finding 2 looks exactly like it.

**Erratum 1.21.10 confirmed.** CTRLA.ALOCK writes and reads back as 1,
and does nothing: CTRLB.LUPD is still 0 after nine overflows, which is
the one thing 36.8.1 says ALOCK should change.

**36.6.6, which is one sentence and one bit.** TCC0 on OSCULP32K, its
overflow published as an event and counted by a timer that runs in
standby, made 16 overflows in a 30 ms window awake, 15 in the same
window spent in STANDBY with CTRLA.RUNSTDBY set, and 0 with it clear.
See [platform.md](platform.md), "Sleep, peripheral by peripheral".

## Bench findings, DMA and the advanced modes

From `test_samc_timer_dma` (10 letters, **101 verdicts, 101/101 three
times - twice warm and once cold**). Wireless: TCC0's WO[0] reaches a TC
capture channel through a combinational CCL LUT and an asynchronous
EVSYS channel, with no pad in the path at all, and `tc.md` carries the
capture side's own findings.

**THE ROUND TRIP, which is what the whole letter set is built around.**
One DMA channel plays an eight-entry duty table into TCC0's CCBUF0 on
the OVF trigger; two more drain the capture meter's CC0 and CC1. Over
**192 judged samples of each stream** the captured widths were the
played table, in order, with a **worst error of ZERO ticks** and a phase
that held across every lap boundary of the loop engine and every block
boundary of the two ping-pong streams - which is what says not one beat
was lost anywhere. The period did not move by a single tick throughout,
no stream overran, and no write-back reading was refused (erratum
1.10.4).

- **A TCC COMPARE REGISTER IS A WORD AND A DUTY STREAM'S BEAT MUST BE
  ONE.** CCBUF is 32 bits on a 24-bit counter, and a HALFWORD write
  lands in the low half alone: 0x00ABCDEF followed by a halfword 0x1234
  reads back **0x00AB1234**. So the element type is `uint32_t`, not
  because the value needs the width but because the register does.
- **One DMA beat per waveform period is exactly one write per update
  window**, which is why the round trip works at all: fact 8's
  SYNCBUSY.CCx stands from a buffered write until the update consumes
  it, and a beat delivered per OVF lands in a window the update has just
  emptied. The beats moved and the periods captured agreed to within a
  block over the whole run.
- **When the DMA outruns the update, the DMAC sees nothing wrong.**
  Flooded with software triggers, the loop played **1229 laps in 20 ms
  against 25** paced by the TCC alone - and `faults()` and the erratum
  1.10.4 refusal count both stayed at zero, because every beat the
  controller moved, it moved. The loss is the peripheral's, in a store
  the silicon discarded. **A discarded buffered write loses a value, it
  does not corrupt one**: the waveform was still playing a table entry
  afterwards, just not the one the beat count named.
- **The hardware circular buffer plays two values for ever with no CPU
  and no DMA.** WAVE.CICCEN0 set UNDER A RUNNING TIMER (WAVE is
  write-synchronized and not enable-protected) made sixteen consecutive
  captured widths alternate between 1200 and 3600 with **nothing
  elsewhere**; **WAVE.CIPEREN** did the same for the PERIOD, alternating
  4800 and 2400 ticks. The same two values through the DMA loop cost
  **one interrupt per lap** - 122 laps in 20 ms, one every two waveform
  periods - and reach exactly the same waveform. So the circular buffer
  wins at two values and loses at three, which is the whole trade.
- **NFRQ toggles on the PERIOD and not on a compare**: PER = 2399 gave a
  captured **4799**, and moving CC0 from 600 to 1800 changed **nothing**
  at all. **MFRQ** makes CC0 the top instead - the same move changed the
  period from 2399 to 3599, both twice CC0 + 1 to the tick, with PER
  left four times too big and ignored.
- **DUAL-SLOPE CRITICAL gives one output two independent edges**, and
  the arithmetic 36.6.2.5.7 does not print is
  **width = (PER - CC0) + (PER - CC2)**: exact to the tick at all three
  settings measured (2399 at 600/1800, 2099 at 600/2100, 1799 at
  900/2100), on a period of exactly 2 x PER.
- **RAMP2A pairs two counter cycles into one waveform cycle.** With
  every register left alone, the captured period **doubled** (2399 to
  4799) while the pulse width did not move - so WO[0] is driven in one
  ramp of the two and idle in the other, not, as the name invites, given
  two duties.
- **Recoverable fault B is the mirror of A**, on channel 1's event
  input, and a pulse on it is a valid fault.
- **FCTRLn.FILTERVAL COUNTS GCLK_TCC CYCLES, NOT PRESCALED ONES** - the
  dead-time unit's story again, and not what 36.8.5's wording or this
  driver's own comment said. On one 93 kHz generic clock, the shortest
  pulse that still made a valid fault was **20 us with FILTERVAL 0 and
  160 us with FILTERVAL 15**, and **a sixty-fourfold prescaler change
  left it at 160** - where fifteen cycles of that clock are 160.9 us.
  The stimulus was calibrated against a 750 kHz stopwatch before it was
  used, and the generator's rate measured rather than computed.
- **A blanking window gates the input, not its edge.** With the pad held
  high for 150 ms, a fault valid with no window was still valid with a
  5.4 ms one, because the window closes long before the period does.
- **FCTRLn.QUAL ties the fault to the waveform, and to THE FAULT'S OWN
  CHANNEL.** Fault B qualifies on channel 1's output: with CC1 at zero
  the same held input raised nothing, and with CC1 at half the period it
  raised a fault. The first version of the letter moved CC0 and measured
  a qualified fault that never fired at any duty - correct behaviour of
  a wrong setup.
- **EVACT0 = INCREMENT counts events and nothing else**: twenty pin
  pulses gave **COUNT = 20** exactly, with the counter's own clock
  slowed to 45.8 Hz so it could not contribute. **EVACT0 = COUNT** turns
  the counter into a gate: 20 ms with the line low advanced it **0**
  counts and 20 ms with it high advanced it **1863** against 1864 at the
  measured generator rate.
- **ERRATUM 1.21.7 DID NOT REPRODUCE.** Dithering is a very sharp
  witness - a dithered duty of 200 + 32/64 shows as **exactly two pulse
  widths**, 199 and 200 - and in the one arrangement where the erratum
  can be told apart from a retrigger's own truncation (a periodic
  hardware RETRIGGER arriving constantly at a rate that never cuts into
  the pulse) the dithered waveform still showed **exactly those two and
  no third**, over forty captures three times over. **And the instrument
  is proved sensitive**: with the retrigger moved to a rate whose
  landing point walks through the pulse, the distinct-width count rose
  at once - from 1 to 2 undithered and from 2 to 3 dithered - which is a
  retrigger's own doing and not the erratum's. Recorded as unreproduced,
  not disproved.

## Not covered yet

Driver gaps (deliberate):

- **CTRLA.DMAOS**, the one-shot DMA trigger of 36.6.5.1: a configuration
  field only. The per-cycle trigger is bench-verified (above).
- **The TCC as a WAKE source.** 36.6.6's counting half is measured (see
  "Bench findings"); no TCC interrupt has ever left a sleep, and the
  fault inputs in a standby are untouched.
- **The debug fault.** `fault_on_debug()` sets DBGCTRL.FDDBD and
  `debug_fault_state()` reads STATUS.DFS, but staging a halted debugger
  is not something a console suite can do.
- **`TccPwm`'s wider periods.** `PwmChannel` scales against a 16-bit
  `max`, so a period past 65535 on a 24-bit instance has to be driven
  through the resource's own `set_cc_buffer()`.
- **A task over the circular buffers.** WAVE.CIPEREN and CICCENx are
  bench-verified (above) and no task wraps them.

Not judged, and deliberately so:

- **Erratum 1.21.7** (dithering plus external RETRIGGER events distorts
  pulses) was STAGED and DID NOT REPRODUCE - see "Bench findings" for
  the arrangement, and for the control that shows the instrument would
  have seen it. Recorded as unreproduced, not disproved.
- **Erratum 1.21.8** - see above. Recorded as unreproduced, not
  disproved.
- **Erratum 1.21.5** (advance capture modes needing the upper channels)
  is stated and not exercised: the bench uses only the plain CAPT action,
  which the item does not touch.

Implemented but not bench-verified:

- **The `alternate` fault source** (FCTRLn.SRC = ALTFAULT, one fault
  taking the other's state at the end of the previous period).
- **The advanced capture actions** (CAPTMIN/CAPTMAX/LOCMIN/LOCMAX/
  DERIV0/CAPTMARK) - see erratum 1.21.5 above.
- **The second non-recoverable fault input (TCE1)**: only the TCE0 half
  has run. Recoverable fault B is bench-verified (above).
- **`TccEvent0Action`'s `stamp`, and `TccEvent1Action`'s `direction`,
  `stop` and `decrement`**: `increment`, `count_while_active`, `count`,
  `retrigger`, `fault` and the PPW capture have all run.
- **The count event output** (EVCTRL.CNTEO with its four CNTSEL choices)
  and the retrigger event output.
