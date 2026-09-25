# RTC and the backup domain (CH32V203 and CH32V303)

A 32-bit counter behind a 20-bit prescaler, the registers that survive
what the rest of the chip does not, and the one pad the block can drive.
Documents of record: the CH32F/V20x_V30x_V31x reference manual V2.3
(chapter 6 for the counter, chapter 4 for the backup registers and the
tamper input, 3.2.3 and 3.4.9 for the domain's reset and its clock
select, 3.3.3 for the low-speed oscillators, 2.4.1 for the write enable -
chapters 4 and 6 apply to the whole family, and the one per-class note
in either is the COUNT of backup data registers), the CH32V203 datasheet
V2.8 (the pin tables of 3.2 for which packages bond PC13, PC14 and PC15)
and the CH32V303/305/307/317 datasheet V3.5 (the same tables, which bond
all three on every CH32V303 package). Driver:
[brio/ch32vx03/rtc.hpp](../../brio/ch32vx03/rtc.hpp). Reference suite:
`test_vx03_rtc`.

## What the silicon does

### What it is, and what it is not

Per the manual, this is not the calendar RTC of the newer parts. There
is no BCD register, no date, no sub-second field and no wake-up timer:
what the silicon keeps is a NUMBER. RTCCLK feeds a 20-bit prescaler
whose underflow is TR_CLK, one edge of TR_CLK raises the SECOND event
and increments a 32-bit counter, the counter reaching the ALARM
register raises the alarm event, and the counter wrapping raises the
overflow event. The calendar is the program's own arithmetic over that
number.

The prescaler, its reload value, the counter and the alarm live in the
BACKUP DOMAIN: per 6.2.2 they "can only be reset by the reset signal in
the backup domain", so they survive a system reset, a Standby wake and,
with a battery on VBAT, the loss of VDD. The CONTROL registers do not -
they are reset by a system or power reset - which is why a program that
comes back finds its count intact and its interrupts disarmed.

### Three gates before a single register answers

Per the manual (4.2), in this order: RCC_PB1PCENR's PWREN and BKPEN
(the power interface's and the backup interface's bus clocks, both
clear at reset), then PWR_CTLR.DBP. With DBP clear, every register of
the RTC and of the BKP block, and the four backup-domain bits of
RCC_BDCTLR, ignore a write. Measured, in both directions: with DBP
clear a backup data register keeps its value and RCC_BDCTLR does not
move, and the same store lands the moment DBP is set again - and every
reset leaves both bus clocks shut and DBP clear, so a program opens the
door at each boot even though what is behind it survived.

### Two disciplines the chapter imposes

Per 6.2.3, and every verb of the driver obeys them.

**Writing is a window.** Wait for CTLRL.RTOFF; set CNF; write one or
more of the four backup-domain registers; clear CNF; wait for RTOFF
again. Between the last two steps the write is crossing into RTCCLK's
domain and a second write started there is lost. The chapter's own
wording is "one or more", which is what lets the prescaler, the counter
and the alarm be set in ONE crossing rather than three.

**Reading needs the domains synchronized.** The registers are clocked
by RTCCLK and read over PB1, so "the RTC register reading through PB1
must go through a RTC rising edge after PB1 is started up. This
situation may occur after system reset and power reset, wake-up from
standby or stop mode." CTLRL.RSF is the flag: clear it, wait for the
hardware to set it, then believe a count. It costs at most one RTCCLK
period - thirty microseconds on a 32.768 kHz crystal, and measured at
fourteen.

AND THE FLAGS CROSS THAT BOUNDARY FASTER THAN THE COUNTER DOES.
Measured on the overflow: at the instant OWF stands, the count the bus
reads is still the one before the wrap, and the wrapped value appears a
moment later. A handler that wants to know where the counter is READS
it, and does not infer it from the event.

### What is write-only, and what is read-only

Per the manual: RTC_PSCRH/L (the prescaler RELOAD) and RTC_ALRMH/L (the
alarm) are write-only and cannot be read back at all; RTC_DIVH/L (the
prescaler's live DOWN-counter) is read-only. So a program that wants to
know what alarm it armed keeps the number itself - and `divider()`,
which is the count of RTCCLK edges left before the next tick, is the
finest time this block offers.

### The carry between two halves

The counter is 32 bits in two 16-bit registers and the chapter says
nothing about the low half rolling over between the two reads. It is a
carry race whatever the silicon intends, so `count()` and `divider()`
read the high half, the low half and the high half again, and take the
whole thing again while the two high reads disagree. That is the
driver's own care and not a rule of the chapter.

Measured at 32768 ticks a second, over four million reads that included
a roll-over of the low half: the retry never fired and no read was ever
torn - the worst step in either direction is a few ticks and never the
65536 a half-read counter would give. BUT THE COUNT DOES STEP BACKWARDS
by up to three ticks now and then (some 700 reads in four million),
which is a read crossing back out of the RTC's clock domain and seeing
a stale copy. A program that reads this counter faster than it ticks
cannot assume the number only grows.

### The alarm reaches two vectors

Per the manual (table 9-2 and 9.4.2's line list): the second, the alarm
and the overflow share the RTC's own line; the ALARM ALONE also reaches
EXTI line 17 and its own line. The second path is what survives a
low-power mode, because an EXTI line is asynchronous and needs no
clock - which is the line the power chapter's timed sleep site arms,
and its alarm is what ends a Stop there ([sleep.md](sleep.md)). A
handler on that line clears TWO flags, the EXTI line's and the RTC's
ALRF, because either one left standing re-enters the handler for ever.

MEASURED, AND NEITHER CHAPTER SAYS IT: the line fires with RTC_CTLRH's
own alarm enable CLEAR. The event leaves the peripheral by itself, so
the wake costs no vector of the RTC's - which is the opposite direction
of this family's other finding about this fabric, that an EXTI line
INTO a peripheral reaches it through that peripheral's EVENT enable and
not its interrupt enable.

AND THE ALARM FIRES AS THE COUNTER LEAVES THE VALUE IT WAS ARMED AT,
on both parts - five ticks after a count set to zero and an alarm at
four. What the handler then READS is the part's: ALR + 1 on the
CH32V203C8T6, ALR itself on the CH32V303VCT6, where the flag reaches
the bus before the count does - as the overflow flag does on both. A
program that wants something done AT a count arms the alarm at that
count and takes the count it reads in the handler for what the part
makes it, one further on or not yet.

### RTCCLK, and the one number that is not knowable from the part number

Per 3.4.9, RTCSEL takes the LSE, the LSI or the HSE divided, and the
selection is ONE-WAY: "once the RTC clock source has been selected
(RTCEN=1), it cannot be changed until the next backup domain is reset."
RTCEN itself is forced to zero by hardware while RTCSEL is `none`.

The first two choices are plain - the crystal's own rate, and an RC
whose spread the part table states as a range. The third is not, because
3.4.9 says the division is 512 on some dies of the CH32V20x_D6 and 128
on others, keyed on "the penultimate 5th digit of the lot number".
Neither number is knowable from the part number, so the part table
states BOTH (`device::rtc_hse_div`), `rtc_hse_divider_known` answers
whether they agree, and a program that clocks the RTC from the crystal
on such a die measures which one it has - against the second event,
which is the only ruler in the room. The CH32V203RB is named among the
parts that divide by 512 and every CH32V303 among those that divide by
128, with no lot rule, so their entries agree. One CH32V203C8T6 measured
on this bench divides by **128**: a reload asked for one hertz out of
15625 Hz ticked at four, which puts RTCCLK at 62500 Hz out of the 8 MHz
crystal. That is a fact of the die's lot and not of the part, which is
why the driver measures instead of choosing.

### The low-speed crystal

Per 3.3.3 and 3.4.9: LSEON starts it, LSERDY says it is stable, LSEBYP
takes an external square wave into OSC32_IN instead and is writable only
while LSEON is clear. After LSEON is cleared "it takes 6 cycles of LSE
clock" for LSERDY to fall. The pads are PC14 and PC15, which only the
three CH32V203 parts with a port C bring out, and every CH32V303.

### The backup registers, and what wipes them

Per 4.3's note under table 4-1, the COUNT is the device class's: ten
16-bit registers on the CH32V20x_D6, forty-two on the CH32V20x_D8 and
the CH32V30x_D8. They are written and read like any register while the
three gates are open, and what clears them is a BACKUP DOMAIN reset
(RCC_BDCTLR's BDRST) or a TAMPER event - explicitly NOT the block's own
RCC reset line, which 4.2.4 says "is not affected by the RCC peripheral
interface control BKPRST bit". Measured, that line moves NOTHING in this
register file: the calibration and the tamper pair survive it as well,
so BKPRST resets the backup INTERFACE and everything behind it is the
domain's. While the tamper event flag stands, every write to a data
register is dropped (4.3.4).

### The tamper input, and the edge the hardware remembers

Per 4.2.2, an edge on the TAMPER pad matching TPAL clears every backup
data register in hardware, and raises an interrupt if TPIE is set. The
trap the section spells out: **the level is sampled whether or not TPE
is set**, and a matching edge is remembered, so enabling the input over
a pad already at the active level fires at once. The chapter's own
advice is to write CTE first. The driver's `tamper()` therefore writes
the level (legal only with TPE clear, 4.3.3's note), clears the
remembered event, and only then enables. 4.3.4 adds that the tamper
interrupt cannot wake the core from a low-power mode.

### The three things the pad can carry

Per 4.2.3 and 4.3.2, PC13 is one pad with four jobs and they exclude one
another: an ordinary I/O; the tamper input (TPE); RTCCLK divided by 64
(CCO), which is what a program measures the oscillator against before it
trims; or a pulse at every alarm or every second event (ASOE with ASOS).
The calibration register CAL[6:0] SKIPS that many RTCCLK pulses every
2^20, so it can slow the clock by up to 121 ppm and can never speed it
up. CCO and ASOE are refused by the driver while the tamper input holds
the pad. PC13 is bonded on three CH32V203 parts of the nine and on all
four CH32V303.

AND THE BLOCK TAKES THE PAD WHOLE. Measured: with TPE set the pad reads
zero and neither its own output stage nor its pull reaches the
detector, so a tamper on this input can only come from OUTSIDE the
chip.

## Types and verbs

`RtcDomain` is the domain. `pwr_bus_clock()`, `unlock(on)` and
`unlocked()` are the write enable, with `unlock()` answering the bit's
READ-BACK so a caller can insist. `reset()` is BDRST, the way back from
every one-way bit here and the only one - at the cost of the counter,
the prescaler, the alarm and the data registers. The crystal is
`lse_enable`, `lse_enabled`, `lse_ready`, `lse_wait_ready(spins)` and
`lse_bypass`, with `has_lse_pins` the part fact; `select(source)`,
`selected()`, `enable(on)` and `enabled()` are RTCSEL and RTCEN, and
`open(source, wipe)` is 4.2's whole sequence in one verb. It does not
start an oscillator: which one to run and how long to wait for it is
the application's. `rtc_hse_clock_hz(hse_hz, which)` and
`rtc_hse_divider_known` are the arithmetic of the third choice.

`Rtc` is the counter. The two disciplines are `write_finished()` /
`wait_write_finished()` and `synchronized()` / `synchronize()`;
`begin_config()` and `end_config()` are the window for a caller that
wants to write the registers itself. `configure(config)` writes the
prescaler and, when the config says so, the counter and the alarm, all
in ONE window; `configure<config>()` is the same with the values
constant, where a prescaler past the twenty-bit field is a compile
error. `prescaler()`, `count(v)` and `alarm(v)` are the single writes,
`count()` and `divider()` the two reads. `interrupts(mask)` is the
three enables, `flags()` / `second()` / `alarmed()` / `overflowed()` /
`clear(mask)` the three events, and `isr()` the body of the handler on
the RTC's own vector. `wake_line`, `arm_wake(on)`, `arm_wake_event(on)`
and `alarm_isr()` are the alarm's second path through EXTI line 17.

`rtc_prescaler_for(rtcclk_hz, tick_hz)` is EXACT OR NOTHING: a ratio
that is not whole, or one the field cannot hold, answers
`rtc_prescaler_none` rather than a rounded reload, because a clock that
is nearly right is a clock that drifts. `rtc_tick_hz()` is the inverse,
for a program that wants to print what it actually got.

`Bkp` is the data registers and the pad. `open()` is the gates,
`data(n)` / `data(n, v)` the run-time pair (an index outside the class's
range answers an empty optional, or false) and `data<n>()` /
`data<n>(v)` the compile-time pair, where such an index is a compile
error. `calibration()`, `clock_output()`, `pulse_output(on, which)`,
`tamper(on, active_low)`, `tamper_interrupt()`, the flags and `isr()`
are the rest of the chapter. `count`, `has_tamper_pad`,
`tamper_pad_port` and `tamper_pad_pin` are the facts a board file reads;
the pad itself is the program's to configure - nothing here claims a
pin.

`PwrRegs` and `pwr_dbp` are device.hpp's, beside RCC's registers,
because two chapters reach that block: ONE of its bits is the door to
the backup domain and the rest - the sleep modes, the PVD, the RAM
retention - are the power chapter's.

## How to use it

A one-second counter on the board's crystal, from a cold boot:

```cpp
brio::RtcDomain::unlock(true);
brio::RtcDomain::lse_enable(true);
if (!brio::RtcDomain::lse_wait_ready()) {
    // no crystal on this board: fall back on the LSI, whose rate is nominal
}
(void)brio::RtcDomain::open(brio::RtcClockSource::lse);
(void)brio::Rtc::synchronize();
(void)brio::Rtc::configure<brio::RtcConfig{.prescaler = brio::rtc_prescaler_for(32768)}>();

const uint32_t seconds = brio::Rtc::count();
```

Coming back from a reset with the count still running: the same
`synchronize()`, and nothing else - the prescaler and the counter are
where they were.

```cpp
brio::RtcDomain::unlock(true);
(void)brio::Rtc::synchronize();
const uint32_t seconds = brio::Rtc::count();
```

An alarm ten seconds out, on the path that survives a low-power mode:

```cpp
(void)brio::Rtc::alarm(brio::Rtc::count() + 10u);
(void)brio::Rtc::interrupts(brio::rtc_alrie);
(void)brio::Rtc::arm_wake(true);
brio::Pfic::enable(brio::Irq::rtc_alarm);
// in the handler: if (brio::Rtc::alarm_isr()) { ... }
```

A value kept across a reset, and the domain wiped when a new clock
source is wanted:

```cpp
(void)brio::Bkp::open();
(void)brio::Bkp::data<1>(0xBEEF);
const auto kept = brio::Bkp::data(1);        // std::optional<uint16_t>

(void)brio::RtcDomain::open(brio::RtcClockSource::lsi, true);   // wipe: RTCSEL is one-way
```

The oscillator on the pad, for a calibration run:

```cpp
(void)brio::Bkp::tamper(false);
(void)brio::Bkp::clock_output(true);         // RTCCLK/64 on PC13
(void)brio::Bkp::calibration(12);            // twelve pulses skipped every 2^20
```

## Bench findings

`test_vx03_rtc` measured the chapter on a CH32V203C8T6 with the board's
two crystals and no wire. THE CORE'S COUNTER IS THE RULER HERE, so the
system clock is rooted in the 8 MHz crystal (96 MHz through the PLL):
a ruler half a per cent off would swamp a measurement of a 32.768 kHz
oscillator, and the clock chapter puts a crystal-rooted rate inside a
twentieth of a per mille. Thirty-three verdicts in `z`; three letters
by name, two of which wipe the domain and one of which reboots the
board.

- **The three gates.** Both bus clocks shut and DBP clear at every
  boot, including after a software reset; with DBP clear a data
  register keeps its value and RCC_BDCTLR's own bits do not move; the
  same store lands the moment DBP is set.
- **The crystal.** It runs, and on a board that has already started it
  it is still running at the next boot - LSEON lives in the backup
  domain and a system reset does not clear it, which is what a
  time-keeping program depends on. LSEBYP is refused while LSEON
  stands.
- **The clock select, one way.** RTCSEL on the crystal with RTCEN
  behind it; `select()` on a DIFFERENT source is refused with nothing
  written, re-selecting the one that stands costs nothing, and the way
  out is a domain reset.
- **The counter.** `synchronize()` came back in 14 us, inside the
  thirty microseconds one period of the crystal would cost. One window
  set the prescaler and the counter together and closed itself with
  RTOFF back; the thirty-two bits read back through two sixteen-bit
  registers as written. RTC_DIV moves and stays inside the reload; the
  counter advanced four counts in sixty milliseconds at sixty-four
  ticks a second. Hammered at 32768 ticks a second - 4.08 million reads
  in four seconds, across a roll-over of the low half - the driver's
  re-read never fired and nothing was ever torn, while the count
  stepped BACK 689 times, the worst by three ticks.
- **RTC_CTLRH takes its write outside the configuration window.** The
  three interrupt enables are not among 6.2.2's four backup-domain
  registers: `interrupts()` waits for RTOFF, stores, and the enable
  reads back with CNF never set - and the alarm that follows fires.
- **THE CRYSTAL'S RATE.** Ten second-events against the core's
  crystal-rooted counter: 10 000 632 us for ten ticks, one tick
  1 000 063 us - which puts the low-speed crystal at **32 765.9 Hz,
  63 parts per million slow** of its nominal 32.768 kHz.
- **The alarm, on both paths.** On the RTC's own vector the handler
  ran 78 ms after a count of zero with the alarm at four - five ticks
  of a 64 Hz counter, the arm falling inside a tick - on both parts,
  and read CNT = ALR + 1 on the CH32V203C8T6, CNT = ALR on the
  CH32V303VCT6 (78091 us and the armed value, five runs of five, with
  the second flag standing beside the alarm's). On EXTI line 17 the
  handler ran with ALRIE CLEAR, 71 ms after arming.
- **The second block, on the CH32V303VCT6** (letter `k`): the thirty-two
  registers BKP_DATAR11..42 hold sixteen bits each under two patterns,
  read back through their own numbers; writing them leaves the first
  block's ten as they were - two sets of registers and not one numbered
  twice - and the forty-third is refused with nothing written.
- **The overflow.** Set two ticks short of 0xFFFFFFFF at 64 Hz, OWF
  stood 31.3 ms later - two ticks exactly - with the counter still
  reading 0xFFFFFFFF at that instant and the wrapped value appearing
  two milliseconds after. The flag is rw0 and stays down once cleared.
- **The backup registers.** All ten hold sixteen bits; the eleventh and
  the zeroth are refused with an empty optional and nothing written;
  CAL takes 0x40 and refuses 0x80; CCO and the alarm/second pulse are
  written and read back, and both are refused while the tamper input
  holds the pad. THE BLOCK'S OWN RESET LINE MOVES NOTHING HERE: after
  BKPRST a data register still read 0xFACE and the calibration still
  read 0x2A.
- **The domain reset, and the HSE division** (letter `w`, wipes the
  domain): BDRST took a data register from 0xDEAD to zero, put RTCSEL
  back to none and stopped the crystal. With RTCSEL then on the HSE, a
  reload asked for one hertz out of 15625 Hz ticked at 250 ms - so
  RTCCLK is 62500 Hz and THIS DIE DIVIDES THE CRYSTAL BY 128. A second
  domain reset was needed to put the crystal back, RTCSEL being
  one-way.
- **Across a system reset** (letter `v`, reboots): the ten data
  registers read exactly what was written, the crystal, RTCSEL, RTCEN
  and the counter all came through, and only the interrupt enables were
  gone - while the two bus clocks and DBP came up shut.
- **The tamper input cannot be raised from this board's own side**
  (letter `t`): with TPE set the pad reads zero, and neither its output
  stage nor its pull raises the event; the data registers were
  untouched and the vector never ran. What would measure this input is
  a wire from another pad. Arming over a pad at the inactive level
  raised nothing, which is 4.2.2's remembered edge being cleared before
  TPE as the driver does it, and CTE puts the flag down and lets the
  registers take a write again.

## Not covered yet

Driver gaps, each with its reason:

- **A tamper event, and what it wipes.** Measured: the block takes PC13
  whole when TPE is set, so neither the pad's own output stage nor its
  pull can raise the event and nothing on this board can. What would
  measure it - the hardware wipe of the data registers, the vector, and
  the write dropped while TEF stands - is A WIRE from another pad or a
  peer board.
- **A power cut.** What the backup domain is FOR is surviving the loss
  of VDD on a battery, and the CH32V203C8T6's board has no VBAT cell
  fitted and no way to cut VDD under program control. What the suite
  reaches is a software reset, which is a weaker claim; a Standby wake
  is a weaker one still, and that one the power chapter measures - the
  domain's own crystal still running the alarm that ended it
  ([sleep.md](sleep.md)).
- **The calibration trimmed against a reference.** CAL is written and
  read back, and it can only SLOW the clock - so trimming it means
  measuring a crystal that is already 63 ppm slow against something
  better than this chip, which is an instrument or a long count against
  a host, and then having nowhere to go.

Implemented but not bench-verified, each with what would measure it:

- **The CH32V203RB's forty-two backup registers.** Its class carries the
  forty-two of the CH32V20x_D8 - the count folds through the part table
  and the eleventh register is a compile error on the CH32V20x_D6 - and
  `test_vx03_rtc`'s letter k writes and reads back the second block,
  BKP_DATAR11..42, under two patterns; it is measured on the
  CH32V303VCT6 (above) and what would measure it on this part is a board
  with it.
- **The LSI as RTCCLK.** The third source is selected by the same verb
  and the watchdog chapter has measured that oscillator at 38.8 kHz on
  the CH32V203C8T6; what would measure this path is a run with RTCSEL on
  the LSI, which costs a domain reset the suite spends on the HSE
  instead.
- **The pad's two outputs on the pad.** CCO and the alarm/second pulse
  are written and read back in BKP_OCTLR, and refused where the chapter
  refuses them; what would measure the waveform is a timer capturing
  PC13, which is a wire between two pads of this board.
- **The six parts with no port C**, which bond neither the 32 kHz pads
  nor the TAMPER pad: `has_lse_pins` and `has_tamper_pad` are false
  there and the whole stratum compiles for all thirteen parts both ways
  the hardware prologue can be built (`brio check ch32vx03`). What would
  measure them is a board.
