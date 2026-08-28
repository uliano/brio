# TC - Timer/Counter (SAM C21)

> **PROVISIONAL.** The whole chapter is built and bench-verified except
> the N-variant capture modes (which this family's device header does not
> even declare), DMA-driven operation and sleep. One measurement the
> suite deliberately DECLINES to make is listed too: capture on an I/O
> pin cannot be given a controlled edge on a board with no wires. The
> list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 35 - and
errata DS80000740S items 1.20.1 to 1.20.3, of which **one is this
silicon**. Driver: `samc/tc.hpp`. Family fixture
`test/family_samc/tc.cpp` plus four negatives under
`tools/check_samc.sh`; the bench suite is `test_samc_tc`.

## What the silicon does

**Five instances, two compare/capture channels each, three counter
resolutions.** A counter with a prescaler, four waveform modes, one
event input and three event outputs. `Tc<n>` is the resource; `TcWo<Pin>`
is one waveform output reached through its pad; `TcPwm`, `TcPwm8`,
`TcPeriodMeter` and `TcPulseWidthMeter` are the tasks.

**The geometry is the device header's, all of it** - and two facts of it
are worth reading before writing any code.

- **TC0 and TC1 share generic clock channel 30; TC2 and TC3 share 31;
  TC4 has 32 to itself.** 35.5.3 warns that two instances sharing a
  channel "cannot be set to different clock frequencies", and
  `Tc<n>::gclk_id` is what says which do. THE SHARP EDGE OF THE SHARING
  is `release()`: it disconnects the shared channel, so releasing one
  half of a pair SILENTLY STOPS THE OTHER (the TSENS campaign paid half
  a letter to learn it). A caller keeping the sibling alive tears down
  by hand - reset plus bus_clock - and leaves the channel connected;
  the verb's own comment says so.
- **`TCn_MASTER_SLAVE_MODE` is 1 for a pair master, 2 for a client and 0
  for an instance that cannot pair.** TC0+TC1 and TC2+TC3 pair into a
  32-bit counter, TC4 does not - which is exactly 35.6.2.4's sentence,
  read out of the header rather than restated. A 32-bit mode on a client
  or on TC4 is refused at compile time.

**The pad-to-(TC, WO) map is not a formula and it is per-package**: 26
pads on the J, 18 on the G, 8 on the E. PA22 and PB08 and PB12 are all
TC0/WO0; PB23 - the bench board's LED - is TC3/WO1. The device header's
`PIN_P<pad>E_TC<n>_WO<k>` symbols are the whole authority, so `TcWo<>` on
an unbonded pad does not compile.

**Reading COUNT is a command, not a load.** COUNT is read-synchronized
"on demand through the READSYNC command" (35.6.8): write
CTRLBSET.CMD = READSYNC, wait out SYNCBUSY.CTRLB and SYNCBUSY.COUNT, then
read. `count8()`/`count16()`/`count32()` do that; the `*_raw()` accessors
skip it and say so.

**The register map is three overlaid views.** `tc_registers_t` is a union
of COUNT8/COUNT16/COUNT32 structs; everything up to SYNCBUSY sits at the
same offset in all three and only COUNT, PER and CC/CCBUF differ. So the
control surface is written once against the 16-bit view and the
width-carrying verbs come in explicit flavours - nothing dispatches at
run time on a mode the caller knew at compile time.

**The period register exists only in 8-bit mode** (35.6.2.4). In COUNT16
and COUNT32 an NPWM or NFRQ waveform's TOP is fixed at MAX; shortening it
means MPWM/MFRQ, which spend CC0 as the period and leave one channel.

**Enable-protection and write-synchronization are different things and
both matter.** CTRLA (bar ENABLE/SWRST), DRVCTRL, WAVE and EVCTRL are
*enable*-protected, so `configure()` and `event_config()` refuse while the
TC is enabled. CTRLB, COUNT, PER/PERBUF and CC/CCBUF are
*write*-synchronized, so every verb that touches one waits its SYNCBUSY
bit out, bounded, and reports.

**Reading CCx is the acknowledgement.** For capture, CCBUFx and CCx act
as a two-deep FIFO: reading CCx is what lets the buffer move up
(35.6.2.8). A capture that arrives while INTFLAG.MCx still stands is
**dropped** and INTFLAG.ERR is set - measured both ways below.

**Both event directions, published here.** Generators: overflow at
0x34 + 3n and the two match/capture channels after it. User: TCnEVU
(23..27), one event input per instance. 35.6.6 says "the TC requires only
asynchronous event inputs" while table 29-3 lists all three paths for
TCnEVU, and 35.6.2.8's note 2 requires the asynchronous path for capture
specifically - the bench used the asynchronous path throughout and it
works. The DMAC trigger ids are published from the header's own
`TCn_DMAC_ID_*` too.

## The errata: read the row, not the column

- **1.20.1 Capture Overflow** (an overflow within 3 APB + 3 GCLK periods
  of the previous capture raises no INTFLAG.ERR): **revision B only**.
- **1.20.2 I/O Pins** ("the input capture on I/O pins does not work",
  workaround: capture through the event system with the EIC or CCL as
  generator): **revision B only.** This is the item most likely to be
  applied by mistake on this part - and see "Not covered yet" for why
  the bench declines to claim either way.
- **1.20.3 SYNCBUSY Flag**: **every revision of E/G/J, so live here.**
  When STATUS.PERBUFV or STATUS.CCBUFVx is cleared, SYNCBUSY is released
  *before* the buffer register has been restored. The documented remedy
  is to clear the flag twice, and `clear_buffer_valid()` is that remedy:
  two writes, each waiting out SYNCBUSY.STATUS.

## Types and verbs

**Vocabulary** - `TcMode` (count8/count16/count32), `TcPrescaler`
(div1..div1024, with `tc_prescaler_divisor()`), `TcPrescalerSync`,
`TcWaveform` (normal/match x frequency/pwm), `TcCommand` (including
`read_sync`), `TcEventAction` (off, retrigger, count, start, stamp, the
two period-and-width orders, and pulse width) with
`tc_action_is_capture()`.

**`TcConfig`** - mode, prescaler and its synchronization, waveform, the
per-channel `capture_enable` / `capture_on_pin` / `invert` masks,
`run_standby`, `on_demand`, `lock_update`, `count_down`, `one_shot`.
**`TcEventConfig`** - action, `input_enable`, `invert_input`,
`overflow_out`, `match_out`.

**The refusals**: `tc_config_valid(n, cfg)` refuses a 32-bit mode on a
non-master, a capture-on-pin channel that is not a capture channel, and a
channel bit past the two implemented. `tc_event_config_valid(cfg, ev)`
holds the two rules that live *between* the structs - counting events
while generating PWM (35.6.2.5.3), and a capture action with no capture
channel or no source for one.

**`Tc<n>`** - `index`, `cc_count`, `gclk_id`, `can_pair`, `pair_index`,
`irq()`, `overflow_generator`, `match_generator(ch)`, `event_user`,
`dma_trigger_overflow`, `dma_trigger_match(ch)`; `init(generator)`,
`bus_clock`, `pair_bus_clock`, `clock`, `reset`, `enable`, `release`;
`configure`, `event_config`, `ctrla`, `evctrl`, `mode`; `command`,
`retrigger`, `stop`, `count_down`; `read_sync`, `count8/16/32` and their
`_raw` twins, `set_count8/16/32`; `period8`, `set_period8`,
`set_period_buffer8`; `cc8/16/32`, `set_cc8/16/32`,
`set_cc_buffer8/16`; `status`, `stopped`, `is_client`,
`period_buffer_valid`, `cc_buffer_valid`, `clear_buffer_valid`;
`flags`/`clear_flags`/`arm`/`disarm`/`armed`/`isr`, `debug_run`.

**`TcWo<Pin>`** - `timer`, `channel`, `claim()` (function E, input buffer
on), `release()`. `tc_wo_code()` and `tc_wo_exists<>` are the map itself.

**Tasks** - `TcPwm<Tc, ch>` (16-bit NPWM, `max` 65535) and
`TcPwm8<Tc, ch, top>` (8-bit NPWM with PER as TOP, `max` = top), both
satisfying `util/pwm_channel.hpp`'s `PwmChannel`; `TcPeriodMeter<Tc>`
(EVACT = PPW, period in CC0 and width in CC1, both channels capturing)
and `TcPulseWidthMeter<Tc>` (EVACT = PW, width in CC0).

## How to use it

**A PWM channel on a pad:**

```cpp
using LedPin = brio::Pin<'B', 23>;
using LedWave = brio::TcWo<LedPin>;          // TC3, WO1 - from the header
using LedPwm = brio::TcPwm8<brio::Tc<3>, 1, 199>;   // TOP 199, so max = 199

brio::Tc<3>::init(/* generator */ 0);
LedWave::claim();
LedPwm::setup(brio::TcPrescaler::div256);    // 48 MHz / 256 / 200 = 937.5 Hz
LedPwm::duty(100);                           // buffered: taken at the next UPDATE
```

**A capture meter fed by a pin, which is the chapter's own recipe**
(35.6.2.8 note 4 - "possible using EIC and Event System" - and note 2,
which requires the asynchronous path):

```cpp
Eic::configure_line(line, {.sense = EicSense::high, .event_out = true});
Tc<2>::init(0);
TcPeriodMeter<Tc<2>>::setup(TcPrescaler::div1024);
Evsys::connect(Tc<2>::event_user, channel, EventChannelConfig{
    .generator = ExtInt<Pad>::event_generator,
    .path = EventPath::asynchronous,
});
```

```cpp
extern "C" void TC2_Handler() {
    if (brio::Tc<2>::isr() != 0u) {
        Latch::store(TcPeriodMeter<brio::Tc<2>>::period_ticks());   // the read IS the ack
    }
}
```

## Bench findings

From `test_samc_tc` (6 letters, **77 verdicts, 77/77 three times**).
Nothing to wire. Every timer runs from generator 0 - the CPU's own
48 MHz OSC48M - so every expected number below is exact arithmetic and
not a measurement of an oscillator, and SysTick is the independent
witness.

- **The prescaler is exact.** Over a SysTick-timed 250 ms window a /1024
  counter advanced 11677..11710 ticks against an exact 11718 (0.1..0.4 %
  low, which is the window's own rounding), and /256 counted **4.00x**
  as fast - a ratio immune to everything the measurement chain adds.
- **A verdict line is 4 ms of console at 115200**, which is worth
  remembering: the first version of this letter measured 4849 ticks where
  4687 were due, because a `bench.verdict()` print sat between starting
  the counter and starting the clock. The window is now opened and closed
  by two reads with nothing between them but the wait.
- **A measurement window has to fit the counter that reads it**: at /256
  a 16-bit counter wraps in 350 ms, which is why the window is 250.
- **COUNT32 is genuinely 32 bits**: TC0 paired with TC1, no prescaler,
  advanced 9587352 counts in 200 ms against an exact 9600000 - 146 wraps
  past a 16-bit counter's range with no wrap of its own. **TC1 reports
  STATUS.SLAVE = 1** and TC0 does not.
- **One-shot stops itself** at the first overflow with no software in the
  path, and STATUS.STOP is set; the STOP command freezes the counter and
  RETRIGGER clears STATUS.STOP by itself.
- **Two READSYNC'd reads of a running counter differ**, and the raw
  accessor afterwards returns exactly what the command fetched - which is
  what makes the two names worth having.
- **A synchronized read is ONE BEHIND**, which the pair of names above
  does not warn about. Measured by `test_samc_sleep` letter c, on a pair
  clocked at 32 kHz where the effect is big enough to see: four
  consecutive `count32()` calls on a counter that had been running for
  six milliseconds returned **0, 196, 201, 205**. `read_sync()`'s waits
  return before the value THIS command latched is readable, so what comes
  back is the value the PREVIOUS one latched. The lag is one read
  interval - invisible in a stopwatch read every few microseconds, and
  the whole interval when reads are milliseconds apart or separated by a
  sleep. A caller that needs the count at a MOMENT reads twice and keeps
  the second; a caller measuring the interval between two reads of its
  own is unaffected, because the lag cancels.
- **PWM measured two ways at once.** On the LED's own waveform output
  (PB23 = TC3/WO1), TOP 199 at /256: duty 0 reads **0** per mille high on
  the pad, duty 199 reads **994**, duty 100 reads **488** and duty 50
  reads **255** - sampled through PORT.IN, which is what an output pad
  with its input buffer on will tell you. And the FREQUENCY, with no wire
  and no scope: **TC0 counting TC3's overflow EVENTS reported 937 in one
  second** against the arithmetic's 937.5 Hz. A timer counting another
  timer's events is a frequency meter, and 35.6.2.5.3 permits exactly
  that (it forbids only PWM in the same instance).
- **Capture through the event system is exact.** An EIC line sensed on a
  HIGH LEVEL - so its event output is a *copy* of the pad, not a pulse -
  through an asynchronous EVSYS channel into TC2's PPW action: a hand-
  built 20 ms high / 30 ms low wave captured as **2344 ticks period and
  937 width** at 46875 Hz, against 2343 and 937 exact.
- **Reading CCx is the acknowledgement, measured both ways.** Four
  periods with nothing read set INTFLAG.ERR and lost captures; four
  periods with CC0 and CC1 read after each one set no error at all.
- **A pad handed to a DRIVING peripheral function does not move under its
  own internal pull**, where an input-only function's does. The EIC's
  function A leaves the pull in charge (`eic.md`); the TC's function E
  takes the output driver and holds the pin. And **a pad left under PORT
  is not seen by the capture input at all** - a digital peripheral input
  needs the mux, where the AC's analog one reaches the pad without it
  (`ac.md`). Those two facts together are why erratum 1.20.2 is not
  judged here (below).
- **The util contract, live and unchanged.** Letter f runs a
  `MeterSampler` AO inside a real kernel: TC2 captures a pin's period
  through EVSYS, the TC's own ISR reads CC0 and stores it in a
  `MeterLatch`, and the sampler publishes on its own 100 ms time event to
  a subscriber AO. Over two seconds of a 20 Hz wave: **41 ISR captures,
  19 samples published, 19 received, last value 2344 ticks (exact 2343),
  21 captures overwritten in the latch.** That is `meter_sampler.hpp`'s
  whole design measured on a second architecture - the AO paces
  PUBLICATION, not capture - and **not one line of `util/` changed** to
  make it work.

## Not covered yet

Driver gaps (deliberate):

- **The minimum and maximum capture modes** (35.6.3.3, 35.6.3.4): SAM
  C20/C21 **N** variants only, and this family's device header does not
  declare CTRLA.CAPTMODE at all, so there is nothing to gate and nothing
  to write.
- **DMA-driven operation.** The trigger ids are published
  (`dma_trigger_overflow`, `dma_trigger_match`), and nothing wires them
  to a `DmaChannel`; the first user is what builds that.
- **Sleep, half of it.** `CTRLA.RUNSTDBY` is now exercised and it is
  the load-bearing bit of the whole clock chain (`platform.md`): a TC
  with it set counts through a standby, with its generator's and its
  source's own RUNSTDBY clear. `on_demand` is still only a field -
  35.6.7's clock-request behaviour, ONDEMAND stopping the request while
  STATUS.STOP is set, has no owner.
- **Whether `read_sync()` should wait for SYNCBUSY.COUNT to be SET
  before waiting for it to clear.** That is the candidate fix for the
  one-behind read above; it is not made here, because the measurement
  that found it belongs to another chapter's suite and the change wants
  its own pass through this one.
- **The double-buffer surface beyond the two setters.** `CTRLA.ALOCK`,
  the UPDATE command and the buffer-valid flags are all exposed, but no
  task uses `lock_update` to stage a coordinated multi-channel update.

Not judged, and deliberately so:

- **Erratum 1.20.2, capture on an I/O pin.** It is marked revision B only,
  so this silicon should capture from a pad - but a controlled edge
  cannot be presented to a muxed WO pad from inside the chip on a board
  with no wires: function E holds the pin against its own pull, and a pad
  left under PORT is not seen by the capture input. The suite prints what
  it can (CC0 did not stay at zero after the pad was handed over, so the
  path is not obviously dead) and declines the verdict. One wire between
  two pads would settle it in a minute.

Implemented but not bench-verified:

- **The 16-bit `TcPwm` task** and the match waveform modes (MFRQ/MPWM):
  the bench PWM is `TcPwm8` in NPWM, where TOP is the period register.
- **`TcPulseWidthMeter`** (EVACT = PW) and the `stamp` and `pulse_width_
  period` actions: only PPW has run on silicon.
- **TC1 and TC4 as counters in their own right**, `PRESCSYNC` other than
  `gclk`, `count_down` with a capture action, `invert` (DRVCTRL.INVEN) on
  a waveform output, and `debug_run`.
- **Operation on the E and G variants**: compile-checked only. The block
  is identical across the family; what differs is which pads carry a
  waveform output, and that is exactly what the family fixture asserts
  per variant.
