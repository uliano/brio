# PWR and the sleep sites (CH32V203 and CH32V303)

Three low-power modes behind one pair of bits, a regulator with two
prices, a supply monitor, a pad that wakes the chip from the deepest of
them - and, on this family, ONE FACT THAT DECIDES THE WHOLE CHAPTER: in
a sleep of any depth no bus master but the core gets a cycle. Documents
of record: the CH32F/V20x_V30x_V31x reference manual V2.3 (chapter 2 for
the modes, the regulator, the PVD and the two registers, with 2.3.4's
note giving each RAM retention bit its address range; 33.2.1 for the
regulator trims that live in the extended register and 33.2.3 for the
die word the PVD's table is keyed on; 34.2.1 for the debug module's
low-power and watchdog bits; 3.4.10's LPWRRSTF for what a Standby leaves
behind; chapter 2 applies to the whole family, and its per-class notes
are on the RAM-retention bits alone), the CH32V203 datasheet V2.8 (the
pin tables of 3.2, where the wake-up pad is PA0 on every package) and
the CH32V303/305/307/317 datasheet V3.5 (2.5's low-power paragraphs,
which keep "32KB of SRAM" through a Standby, and the pin tables of 3.2,
PA0/WKUP on every package). Drivers:
[brio/ch32vx03/pwr.hpp](../../brio/ch32vx03/pwr.hpp),
[brio/ch32vx03/sleep.hpp](../../brio/ch32vx03/sleep.hpp) and
[brio/ch32vx03/bus_activity.hpp](../../brio/ch32vx03/bus_activity.hpp).
Reference suite: `test_vx03_sleep`.

## What the silicon does

### Three modes, one pair of bits

2.3's table 2-1, with the entry conditions of 2.3.2 to 2.3.4: the
machine has three low-power modes and they are named by two bits -
PFIC_SCTLR.SLEEPDEEP, which is the CORE's, and PWR_CTLR.PDDS, which is
this block's:

| Mode | SLEEPDEEP | PDDS | What stops | What ends it |
|------|-----------|------|------------|--------------|
| Sleep | 0 | 0 | the core clock, and nothing else | any interrupt or wake event |
| Stop | 1 | 0 | HSE, HSI, PLL and every peripheral clock; SRAM, the registers and the pin states are kept | an EXTI line, interrupt or event |
| Standby | 1 | 1 | the same, and the regulator too | the WKUP pad's rising edge, an RTC alarm, the NRST pad, an IWDG reset - and, measured, ANY armed EXTI line - each through a POWER RESET |

Neither bit means anything without the other, which is why one verb
writes the pair and one verb reads it back - and although SLEEPDEEP
alone decides whether a sleep is deep, that verb takes PDDS down with it
for a shallow rung too, because 2.3.2 states both as Sleep's entry
conditions and because it makes "nothing armed here reaches Standby" a
fact of the registers. Arming is not sleeping: the
instruction belongs to the kernel's idle path
([platform.md](platform.md)), and that split is what lets
[the power model](../design/power.md) place a mode without a new kernel
hook.

Standby has a second exit the table adds in a note - "any event can also
wake up the system, but the system will not be reset after wake-up".
HALF OF THAT IS TRUE HERE AND HALF IS NOT, measured on both parts: the
alarm's line armed as an interrupt and the same line armed as an EVENT
ALONE ended a Standby, and on the CH32V203 A PAD'S line armed as an
event did too, an exit 2.3.4 does not list at all - and every one of
them came back through the RESET VECTOR and not through the instruction
after the wait. So an ordinary EXTI line IS a way out of this mode, and
there is no way out of it that returns. The independent watchdog is one
of the documented exits and is measured as one on the CH32V303VCT6: its
reset ended a Standby some 200 ms after it was armed, with IWDGRSTF and
PORRSTF both standing at the boot.

### A sleep of any depth starves every other bus master

This is the chapter's governing fact on this family, and it is not in
the chapter: measured on the silicon of both parts and recorded in
[README.md](README.md), in a sleep NO BUS MASTER BUT THE CORE GETS A
CYCLE. A memory-to-memory DMA started just before the sleep moves the
handful of items already in its pipeline and then nothing until the core
wakes; the CH32V203's USB device controller cannot reach its packet
memory, so an armed endpoint loses the host's bytes and an enumeration
dies in its first control transfer; and the CH32V303's host/device
controller, whose buffers are the program's RAM, sends IN packets full
of zeros where it could not read them and never gets an enumeration
past its descriptors ([usbfs.md](usbfs.md)). The clocks run - a timer on
the peripheral bus and the core's own counter count the whole sleep -
and no software mitigation short of staying awake works.

So the rule here is not "a program that uses the bus must not sleep", it
is a MECHANISM. Every driver whose peripheral is a bus master keeps one
count while it is working:

- a DMA channel, of either controller where the part has two, from the
  moment EN goes up to the moment it comes down, counted in the one verb
  every engine and every task goes through - and EN stays SET when a
  non-circular block completes here, so a channel its owner has not taken
  down still counts, which is what the registers say too;
- a USB controller - the CH32V203's device controller or the
  host/device controller of either part - from its pull-up to its
  detach.

The kernel's idle path reads that count and sleeps only at zero; above
zero it returns at once with interrupts enabled, which the Platform
contract allows and which leaves the loop spinning and the bus served.
The sleep sites read the same count and REFUSE to arm any rung over it -
a vote already lost, without a voter having to be written. A program is
free to idle whatever it is doing, and the silicon's own state decides.

What that costs, measured: an `idle()` that returns at once is some
THIRTY TO FORTY HCLK CYCLES against the up-to-a-millisecond a tick-ended
sleep takes, and a memory-to-memory block of 65535 items went on running
across eight of them and completed with its data moved. The USB consoles
are the other half of the proof: with the ordinary kernel loop, idle
path and all, the device enumerates, is configured and carries its
bytes with no receive overrun and no controller error, on both parts.

### What a Stop costs, and what comes back from one

Per 2.3.3, LPDS prices a Stop: with it clear the regulator stays in its
normal mode, with it set it goes low-power, and RAMLV on top of that
(valid, per its own note, only with LPDS set) puts the RAM in its
low-voltage mode for "the lowest power consumption". What either SAVES
is a current, and this desk has no meter: the driver enforces the
interlock, states the bits, and measures neither. What the first of
them COSTS is measurable and is measured: thirty microseconds more at
the wake on the CH32V203C8T6 and sixty on the CH32V303VCT6, which is one
and two periods of the only clock a stopped core still counts (sixteen
wakes on each regulator, and the difference divided).

The alarm that ends a Stop lands one count of the ruler after the value
it was written with, on both parts - but on the CH32V303VCT6 the same
alarm served AWAKE arrives on the value itself, where the CH32V203C8T6's
is one count late awake too. Either way a Stop ends late and never
early, which is the direction the timed site's arithmetic is built for.

What comes back is not what went in. Per 2.3.3 the HSI is the system
clock after the wake, with the PLL and the HSE off - so a program
running at 144 MHz resumes at 8; measured, SWS reads the HSI at every
wake. The plain site is templated on the clock task for exactly this:
SWS is read, and where it is not what the task promised the task is
re-run (under a dynamic clock, its own restore(), for the rate in
force). Putting a PLL rate on the board's crystal back takes ABOUT TWO
MILLISECONDS on the CH32V203C8T6 and ONE AND A THIRD on the CH32V303VCT6,
nearly all of it the crystal's own start-up.

AND SO DOES THE INDEPENDENT WATCHDOG, which is the opposite of what a
reader of 7.1 would expect: measured on both parts, a watchdog armed for
two hundred milliseconds and refreshed immediately before a Stop of two
seconds did not reset the board in the sleep - the reset arrived 208 to
215 milliseconds after the WAKE. So a watchdog is not a way back out of
a Stop on this family, and a program that stops for longer than its
time-out is not reset for doing so. A STANDBY IS THE OPPOSITE: on the
CH32V303VCT6 the same watchdog, armed after an alarm two seconds out,
ended the Standby at 225 ms - and the debug module's watchdog freeze bit
read clear throughout, so neither answer is the probe's.

Kernel time stands still for the whole of a Stop, because the core's STK
counter counts HCLK. Two things follow, and both are answered in the
idle path rather than in the site: the timebase is PAUSED and its
pending bit cleared immediately before the instruction - with the WFE
idiom this core's idle uses, a tick that is merely pending would end the
sleep before it began - and it is resumed immediately after. That costs
ONE TICK, knowingly: a tick already fired when the sleep begins is
dropped instead of served. The timed site repairs it at no cost, its
witness measuring the wall and subtracting only the ticks the counter
served; without a timed site a deep sleep is legal only with no
deadline armed, which is the model's own restriction.

The core's own state comes through a Stop whole: on the CH32V303VCT6 the
thirty-two f-registers and fcsr, loaded before a Stop and compared after
it, came back bit for bit, with mstatus.FS still Dirty. After a Standby
nothing of the core survives - the wake is a reset, and the crt switches
the unit on again.

### What a Standby keeps

Per 2.3.4 four bits decide what of the RAM survives, and the register
description keys them by device class in a way that is easy to misread.
On the CH32V20x_D6 - every part of that series but the CH32V203RB -
R2KSTY and R2KVBAT govern THE WHOLE 20 KB and the other two do not
exist; on the D8 classes - the CH32V203RB and every CH32V303 - the first
pair is a 2 KB bank and the second a 30 KB one, and 2.3.4's own note
gives their addresses: 0x2000_0000 to +2K, and +2K to +32K. The verbs
are therefore named for the banks and not for the sizes, and what the
first one covers is a constant of the part.

MEASURED ON THE CH32V303VCT6, BANK BY BANK: with nearly all of its 64 KB
painted, a Standby with R2KSTY kept the whole first bank and nothing of
the second, R30KSTY the whole second and nothing of the first, both of
them the two banks whole, and neither nothing - and NOTHING ABOVE 32 KB
SURVIVED ANY OF THEM, which is the datasheet's "32KB of SRAM". A word
that was not kept did not come back as zero but as another value, and a
control across a software reset kept every painted word.

AND THOSE BITS ARE WRITE-ONLY ON THAT DIE. Bits 16 to 20 of PWR_CTLR -
the four retention bits and RAMLV - read back zero whatever was written,
there and over the debug link, and they still act: so a read-modify-write
of the register, ANY of them, writes them away, and a Standby entered
after one keeps nothing - measured, every painted word lost in every
bank, until the bits went into the store that sets PDDS. They read back
on the CH32V203C8T6. The driver therefore keeps the program's choice of
them in RAM and every store to PWR_CTLR carries it - the RTC chapter's
DBP included - and its readbacks of those five bits are that copy, the
silicon's being unreadable where it matters. The copy starts at zero at
every boot, so the first store after a boot writes the bits to zero
whatever a reset left in them, and a program states its retention again
before every Standby.

And the D6's number is the CLASS'S LARGEST ARRAY: four parts of the
CH32V203 carry ten kilobytes and not twenty, and the note says nothing
about them. The constant is the manual's number bounded by what the
part has, which is the most that can honestly be said without a
measurement.

Two more facts of the same section. PWR_CTLR's bits 16 to 20 "can only
be reset by backup" while every other bit of the register is reset by a
Standby wake; and PWR_CSR "remains unchanged after woken up from Standby
mode", which is what makes its two flags readable at the next boot.

There is a THIRD flag in another chapter that might have been part of
this story and is not: RCC_RSTSCKR's LPWRRSTF (3.4.10), a "low-power
reset" whose cause the register description never states. Measured on
both parts, A STANDBY WAKE DOES NOT RAISE IT: what the reset chapter's
register carries at that boot is PORRSTF and nothing else (beside
IWDGRSTF when the watchdog ended it), so the power-on flag and a Standby
exit are indistinguishable there and PWR_CSR.SBF is the only thing that
tells them apart. Which is also why SBF is worth reading before
anything clears it.

### The supply monitor, and the table that is the die's

Per 2.2.2 the PVD compares VDD against one of eight thresholds and
reports the answer in PWR_CSR.PVDO - one while the supply is BELOW the
threshold - with about 200 mV of hysteresis (figure 2-3, and the tables
state each level's falling edge). Its output is also wired to EXTI line
16, so a supply crossing the threshold can be an interrupt or a wake
event.

What a level is worth in millivolts is NOT a fact of the part number.
2.4.1 gives two tables, one for dies whose FEATURE_SIGN.VLEVEL reads 1
(a supply specified from 2.4 V) and one for dies where it reads 0 (from
1.8 V), and 33.2.3 adds that the register is to be believed only where
its low byte is the inverse of the next - a die that fails the test
being read as the 2.4 V kind. So the millivolts are a RUN-TIME question
here: the driver reads the die and answers, and the constexpr forms take
the supply level as an argument for a program that knows it.

AND THAT VALIDITY TEST EARNS ITS KEEP. On both bench chips FEATURE_SIGN
reads 0xE339E339 - this family's own ERASED pattern, the word never
having been programmed - whose low byte is not the inverse of the next,
so the driver falls back to the 2.4 V table as 33.2.3 prescribes. A
reader that believed the bit would have taken the other one.

The two datasheets do not quite agree with the manual about that table.
Their tables 4-4 (the CH32V203's) and 4-4-1 (the CH32V303's, for VLEVEL
1) put fifteen of the sixteen thresholds 10 to 30 mV above 2.4.1's -
2.39 V rising at the lowest level against 2.37, 3.31 against 3.29 at the
highest - and the sixteenth, level 6 rising, 10 mV below it (3.17 against
3.18), with the hysteresis stated as 80 mV rather than drawn as about
200; the CH32V303's table 4-4-2, for VLEVEL 0, is the manual's 1.8 V
table to the millivolt.
The driver's numbers are the manual's, the register's own document; no
threshold has been crossed on either board to tell the two apart.

### The wake-up pad, and the flag two sources share

Per 2.4.2, EWUP forces PA0 to an input with a pull-down and makes its
RISING edge a Standby exit; with the bit clear the pad is an ordinary
pin. Both datasheets' pin tables name it PA0-WKUP on every package.

WUF is the flag it raises - and per 2.4.2's own wording an RTC alarm
raises THE SAME ONE, so a program that arms both learns from it only
that it was woken. SBF is the other flag: it says the machine entered
Standby, and it is what a program that came back through the reset
vector reads to know where from. Both are cleared by write-one bits of
PWR_CTLR that read as zero always, and CWUF's own note says the flag
falls two system clocks after the store - measured, both clears land
within a hundred HCLK cycles of the store, verbs and all.

WUF IS THE WAKE'S FLAG AND NOT THE EVENT'S, which the chapter does not
say and the bench does: an alarm served with the core RUNNING leaves it
clear, on both parts, and so do rising edges on the pad with EWUP set and
the core running on the CH32V203C8T6, while a Standby ended by that same
alarm comes back with it standing. So it is read at the boot beside SBF and nowhere else
- and between the two of them a program can tell the DOCUMENTED exits
(the alarm, the pad) from an ordinary EXTI line's and from the
watchdog's, which set SBF alone.

### The debugger's bits

Per 34.2.1 the debug module's configuration register - a core CSR at
0x7C0 on this family, not a peripheral register - carries three bits
that keep FCLK and HCLK running through Sleep, Stop and Standby, and,
further up, the bits that freeze each watchdog and each timer while the
core is in its debug state. Any of the first three set turns a sleep
measurement into a fiction. The driver READS them and offers no writer
at all, because a csrw to that CSR from the running program resets the
part on this silicon ([README.md](README.md)).

The probes on this bench leave the three low-power bits CLEAR, which is
what makes every span below the silicon's own and not the debugger's -
and it is the first thing the suite prints. The register reads 0x300 on
the CH32V203C8T6 behind its WCH-LinkE - the two watchdog freeze bits set
- and 0x0 on the CH32V303VCT6 behind its CH549 link.

### What this chapter has not got

There is no flash power-down bit here (PWR_CTLR [15:9] is reserved,
where the sister family carries FLASH_LP), no low-power RUN mode (2.3
lists three modes and no fourth), and no auto-wake-up unit: the alarm
that ends a Stop on this family is the RTC's, reached through EXTI line
17 (2.3.5). The regulator's two voltage trims live in the extended
register (33.2.1, LDOTRIM and ULLDOTRIM, 1.1 V out of reset), and the
third bit of that register that belongs to low power - whether the
crystal keeps oscillating through one, the CH32V20x_D8's alone - is the
clock chapter's, because it is about the oscillator.

## Types and verbs

### pwr.hpp

`PwrMode` names the three modes and `pwr_mode_resets` /
`pwr_mode_stops_clocks` are the two questions worth asking of one.
`StopRegulator` and `StopConfig` are what a Stop costs, with
`stop_config_valid` the chapter's interlock.

`Pwr` is the block, monostate, and every verb opens its bus gate first
because the register answers nothing through a closed one.

- `bus_clock` / `open` / `ctlr` / `csr`: the gate and the two registers.
- `arm` (with or without a `StopConfig`) and `mode`: the mode pair
  written together and read back off the silicon.
- `stop_config`: LPDS and RAMLV, set or read - RAMLV from the kept copy.
- `wait_for_interrupt`, `wait_for_event`, `enter`: the instruction, in
  its two forms, and the deliberate one-shot that arms and stops.
- `wakeup_flag` / `standby_flag` / `clear_wakeup_flag` /
  `clear_standby_flag` / `clear_flags`: the two flags and their
  write-one clears.
- `wakeup_pin`, `wakeup_port`, `wakeup_pin_number`, `wakeup_pad_bonded`:
  EWUP and the pad it claims.
- `pvd` (a run-time level or a constant one), `pvd_level`, `supply_low`,
  `supply_level`, `pvd_rising_mv`, `pvd_falling_mv`, `arm_pvd`,
  `arm_pvd_event`, `pvd_isr`, `pvd_exti_line`: the supply monitor, its
  two tables and its line. `feature_sign` and `pwr_supply_level` are the
  die's word and the reading of it.
- `retain_ram`, `retain_ram_on_vbat`, `retain_upper_ram`,
  `retain_upper_ram_on_vbat`, `ram_retention_bytes`,
  `has_upper_ram_retention`: what a Standby keeps, per class, the
  readbacks being the driver's kept copy; `retention_readback` is what
  the silicon returns for the five kept bits.
- `core_voltage`, `low_power_voltage`: the regulator's two trims.
- `debug_cr`, `debug_in_sleep`, `debug_in_stop`, `debug_in_standby`,
  `debug_holds_clocks`: read-only, and the reason is above.

The kept copy itself is device.hpp's - `pwr_ctlr_kept`, the five bits
`pwr_ctlr_kept_bits`, and `pwr_ctlr_store()`, the one store to PWR_CTLR
that both this chapter and the RTC chapter's DBP go through - because two
chapters write that register.

### sleep.hpp

`Ch32vx03SleepSite<Clock, config>` is [the power model](../design/power.md)'s
site: `arm`, `disarm`, `armed`, with `resume_clock` the verb that puts
the tree back and `enter_standby` the door the ladder does not have.
The ladder is `light` to Sleep, `standby` to a Stop on the main
regulator, `deep` to a Stop on the low-power one; `armed()` reads the
silicon and answers `none` for the shallow rung, because Sleep is what
the kernel's plain idle does anyway. Every `arm()` clears PDDS whichever
rung it is given, so no rung can reach Standby.

`Ch32vx03TimedSleepSite<P, Clock, config, site_config>` adds the RTC in
both roles - the alarm on EXTI line 17 as the wake, the counter as the
witness of the span slept - and so lifts the restriction that a program
with armed time events must not Stop. Beside the model's three verbs it
has `init` (route the clock, start the oscillator, set the prescaler,
arm the line), `ready`, `alarm_armed`, `place_alarm`, `counts_for`,
`ticks_for`, `resync`, `last_advance`, `last_counts`, and `isr`, the
four-act body the application binds to the alarm's vector. THAT BINDING
IS NOT OPTIONAL: `init()` enables the line in the interrupt controller,
and an unbound vector on this target's crt is a spin. The site also owns
the RTC whole - the domain gate, the clock select, the prescaler and the
alarm - so a program using it does not drive that counter elsewhere.

`TimedSleepConfig` states the RTC's rate and the tick wanted out of it,
`timed_sleep_divider` and `timed_sleep_tr_hz` derive the prescaler and
the ruler, and `timed_sleep_config_valid` refuses what the prescaler
cannot make. THE RATE RULE IS DIRECTIONAL: state a rate NOT BELOW the
true one, and both halves err the way the kernel's time contract allows
- the alarm lands late and the resync advances short, never the
opposite.

### bus_activity.hpp

`BusActivity` is the count itself: `active`, `entered`, `left`. The
platform publishes the same number as `bus_masters_active()`. The count
saturates rather than wrapping and there is no verb to reset it: an
unbalanced increment leaves the program awake, which wastes current,
and a leak is a bug to find rather than a state to paper over.

## How to use it

A Stop under the power manager, with the clock put back on the way out:

```cpp
using Site = brio::Ch32vx03SleepSite<Clock>;
brio::PowerManager<P, Site, brio::PowerConfig{}, Voters...> power;
```

The same with kernel time kept honest across the Stop - the RTC's alarm
placed where the nearest deadline is, its counter read as the witness:

```cpp
using Site = brio::Ch32vx03TimedSleepSite<P, Clock>;   // the crystal, a 1024 Hz tick
if (!Site::init()) { /* no crystal on this board: a plain site is what is left */ }
extern "C" BRIO_CH32_INTERRUPT void rtc_alarm_handler() { Site::isr(); }
```

Standby, which is not a rung and does not come back the way the others
do - the way back armed first, the SRAM to keep named, the flag read at
the next boot:

```cpp
(void)brio::Pwr::wakeup_pin(true);        // PA0's rising edge - or the RTC's alarm
brio::Pwr::retain_ram(true);              // the first bank: 2 KB on a D8 class, all of it on the D6
brio::Pwr::retain_upper_ram(true);        // the 30 KB above it, where the class has it
Site::enter_standby();
// ... at the next boot:
if (brio::Pwr::standby_flag()) { brio::Pwr::clear_flags(); }
```

The supply monitor, as a warning ahead of a brown-out:

```cpp
(void)brio::Pwr::pvd<brio::PvdLevel::level5>(true);
(void)brio::Pwr::arm_pvd(true);           // EXTI line 16, both edges
const bool low = brio::Pwr::supply_low();
const uint16_t mv = brio::Pwr::pvd_rising_mv(brio::PvdLevel::level5);
```

Asking what the sleep path asks, from a program:

```cpp
if (P::bus_masters_active() != 0) { /* a transfer is up: no sleep of any depth */ }
```

## Bench findings

THE RULER for everything a stopped core is timed with is the RTC's own
clock: the crystal over thirty-two makes a count of 976 us, and the
prescaler's live down-counter under it makes the finest span this
silicon can measure while its core is stopped ONE RTCCLK PERIOD, thirty
microseconds. Every letter that stops or stands the chip by arms the
RTC's alarm as its way back first, and a Standby is entered only once
that counter has been seen counting.

### On the CH32V203C8T6

`test_vx03_sleep` at 96 MHz from the board's crystal, with the RTC on
its 32.768 kHz one - letters a to h, `w` (which enters Standby three
times, each ending in a reset, fifteen verdicts) and `g` (which ends in
one, two).

**The debug module holds nothing up.** DBGMCU_CR reads 0x300 with the
probe attached and its three low-power bits are clear, so every span
below is the silicon's own.

**A light sleep is the tick's.** With the shallow rung armed, two
`idle()` calls covered the 808 us to the next tick, and the tick
counted through them.

**The count of bus masters, and what it saves.** At rest the count is
zero. A memory-to-memory block of 65535 items raises it to one at the
EN transition; across eight `idle()` calls - 225 HCLK cycles in all,
some twenty-eight each, against the up-to-a-millisecond a tick-ended
sleep costs - the channel went on moving items, and the block completed
in 4094 us with its data moved. A channel left enabled after its block
still counts one, and `stop()` is what releases it. The USB pull-up
raises the same count on its own (eight `idle()` calls, 212 cycles) and
the detach releases it. And the proof outside the suite: the CDC
console runs the ordinary kernel loop, idle path and all, and
enumerates, is configured and carries its bytes with no receive overrun
and no controller error.

**A Stop, and what the wake costs.** Asked for 205 counts (200 ms) on
the main regulator, a Stop slept 200.8 ms and overshot the count its
alarm named by 1068 us. OF THAT, 1007 us IS NOT THE SLEEP'S: the same
alarm served with the core awake arrives that late too, so this block
raises its event one count of the ruler after the value written. What
is left - SOME SIXTY MICROSECONDS - is the wake itself. The tick did
not advance across the sleep (the STK counts HCLK, which a Stop stops),
SWS read the HSI at the wake, and the first counter read after it owed
30 us of synchronization, which is one RTCCLK period exactly as 6.2.3
asks.

**What the low-power regulator costs at the wake: 30 us.** One period
of the ruler, which is under its resolution for a single sleep - so it
was measured over sixteen Stops of each kind, the tree left on the HSI
throughout: 560 periods of overshoot in all against 576, a difference
of exactly one period a wake.

**Putting the tree back: about 2 ms**, nearly all of it the crystal's
own start-up; measured on the core's counter at the HSI rate it runs at
until the switch.

**The timed site, under a real kernel.** A 500 ms time event, a
`PowerManager` with one voter, the loop's own `idle_if_empty()`: the
site placed the alarm 512 counts out, the Stop ran, the resync handed
the ticker 502 ticks and the event matured 502 ms later BOTH on the
wall and on the kernel's tick - late by two milliseconds and never
early. The first event after the wake ended the round, the manager
disarmed on its own, and the clock came back to the program's rate.

**The vote round.** One not-ok ends it with nothing armed; a standing
lock clamps a deep request to the rung it names and the voter is asked
for the clamped depth; a deadline nearer than the manager's floor
refuses the round before any voter is asked; with every vote yes the
site arms, and SLEEPDEEP reads back set with PDDS clear. AND THE
REFUSAL THAT IS THE SILICON'S: with a DMA channel enabled the site
refuses every rung and the round is lost, while `none` still takes.

**A pad's EXTI line ends a Stop.** With an instrument outside the board
driving a rising edge into PA0, a Stop armed with a three-second RTC
backstop ended on the edge after 436 to 1566 ms - the edge detection is
asynchronous and needs no clock. With nothing driving the pad, the
backstop ended it at its three seconds.

**Standby: three ways in, one way out.** The RTC alarm's line as an
interrupt, the same line as an EVENT ALONE, and A PAD'S line as an
event all ended a Standby - and all three came back THROUGH THE RESET
VECTOR, 2010 ms after the alarm was placed for the first two and 1486
ms for the third, which is when the edge arrived. SBF stood at every
one of those boots; WUF stood at the two the alarm ended and was CLEAR
at the one the pad's line ended. RCC_RSTSCKR carried PORRSTF and not
LPWRRSTF. The `.noinit` token written before the sleep came back intact
with the first RAM bank's retention bit set, and the two write-one
clears put both flags down in 86 HCLK cycles.

**The independent watchdog does NOT count through a Stop.** Armed for
about 200 ms of its own LSI and refreshed immediately before a Stop
asked for two seconds, it did not reset the board in the sleep: the
Stop ran its full two seconds and the reset arrived 215 ms after the
wake, with IWDGRSTF at the next boot. Twice, to a millisecond.

**The supply monitor.** The eight thresholds walked from the bottom up
never read low on this board, which places its rail above the highest
of the die's table (3290 mV rising). The crossing itself therefore
wants a supply that can be varied; the line the detector's output sits
on was proved with 9.5.1's software trigger instead, which reached the
PVD's own vector in 1 us and left the flag cleared by the driver's
handler body.

**The alarm's own write window: 36 to 49 us** against a count of
976 us, which is what makes the timed site's floor of four counts a
margin and not a guess.

**Sizes.** The whole suite is 42860 bytes of text on the CH32V203C8; on
the 32 KB tier it builds as the two group images its header declares,
21684 and 25004 bytes of the 28672 available.

### On the CH32V303VCT6

`test_vx03_sleep` at 96 MHz from the board's crystal, the RTC on the
board's 32.768 kHz crystal: **57 pass, 0 fail** in `z`, fifteen more in
letter `w`, nine in `r` (five resets), three in `x` and two in `g`.

**The debug module holds nothing up**, and here it freezes nothing
either: DBGMCU_CR reads 0x0.

**The retention bits read back zero.** All four set together, and RAMLV
beside them, read 0x0 on the silicon and over the debug link, the BKP
clock open or not; the driver's verbs answer the kept copy.

**The same Sleep and the same count.** A light sleep ended by the tick
after two `idle()` calls and 404 to 644 us; a memory-to-memory block of
65535 items counted one, eight `idle()` calls cost 321 to 330 HCLK
cycles and the channel ran on (the block done in 4095 us, the data
moved, EN still up counting one, `stop()` releasing it); the
host/device controller's pull-up counted one, eight calls 283 to 308
cycles, the detach releasing it. And DMA2's first channel counts one
too and is refused every rung while it runs.

**A Stop, and a count the Stop adds.** The alarm served awake arrives
ON the count it names. A Stop asked for 205 counts slept 200.5 to 201.0
ms and overshot that count by 1037 us - one count of the ruler and
sixty microseconds - so in a Stop the alarm reaches the core one count
later than awake. The tick did not advance, SWS read the HSI at the
wake, the first read owed 21 to 22 us of synchronization, and the tree
came back in 1344 to 1389 us of HSI time. Sixteen Stops of each: 544
periods of overshoot on the main regulator and 576 on the low-power
one - 61 us more a wake.

**The timed site, under a real kernel**: the alarm placed 512 counts
out, the resync handing the ticker 501 ticks, a 500 ms event matured at
501 ms on the wall and on the kernel's tick.

**The vote round**: the same nine verdicts as on the CH32V203C8T6, DMA2's
channel among the refusals.

**No pad's edge on PA0** reached a Stop - nothing drives the board's key
- and the backstop ended it at 3072 ms; with EWUP set and the core
awake, no edge and WUF clear.

**An edge on the board's wire ends a Sleep.** TIM3's one pulse on PA6,
started with the tick paused and due 500 us later, reached PA1's line
through the wire: the Sleep lasted 501 us of the core's counter, and
line 1's handler ran once, 30 to 32 timer cycles (at 96 a microsecond)
after the edge.

**A Stop at every rate of a DynamicClock.** On the crystal's PLL at 96,
144 and 48 MHz and on the bare HSI: each Stop ended by the alarm on the
HSI, the site's `disarm()` - the clock's own `restore()` - put back the
rate in force, and the core's counter measured 95 999, 143 997, 48 000
and 8 027 kHz against the RTC after it.

**The floating-point unit across a Stop.** The thirty-two f-registers
and fcsr (round-up with four flags standing) loaded, a Stop of fifty
counts ended by the alarm on the HSI, and all of them came back bit for
bit, mstatus.FS still Dirty.

**Standby: the same three ways in.** The alarm as an interrupt, the
alarm as an event alone, and the alarm as a backstop with PA0's line
armed as an event and nothing driving it: three resets at 2010 to 2011
ms, SBF and WUF standing, PORRSTF alone, the `.noinit` token back through
R2KSTY - the kept copy carried by every store - and the two clears in 98
to 100 HCLK cycles.

**What a Standby keeps, bank by bank.** 14 592 words painted from just
above the linker's sections - 285 to 287 of them in the first bank,
7680 in the second, 6625 to 6627 above 32 KB: a software reset kept all
of them; a Standby with both bits kept
the two banks whole and nothing above; R2KSTY alone the first bank and
nothing of the second; R30KSTY alone the second and nothing of the
first; neither, nothing - no lost word reading zero. Written by the
driver's separate read-modify-write stores before it kept a copy, the
same Standbys kept not one word anywhere.

**The independent watchdog through a Standby.** The alarm placed two
seconds out, the watchdog armed at /32 with a reload of 249, the
Standby entered: the board came back 225 ms after the alarm was placed,
in two runs, IWDGRSTF and PORRSTF standing, SBF set and WUF clear - the watchdog, not
the alarm - with no watchdog running into the new boot. Across a Stop
the same watchdog did not count: the Stop ran its two seconds and the
reset came 208 and 209 ms after the wake, in two runs.

**The supply monitor**: FEATURE_SIGN reads the erased 0xE339E339 here
too, so the 2.4 V table; never low up to 3290 mV; the software trigger
on line 16 reached the PVD's vector in 2 us.

**The alarm's own write window: 39 to 56 us** against the same 976 us.

**Size.** 46036 bytes of text; the painted array the retention letter
counts is RAM the image leaves unloaded.

## Not covered yet

Driver gaps, each with its reason:

- **What any of it is worth in current.** Every knob in this chapter -
  the low-power regulator, the RAM's low-voltage mode, the core voltage
  trim, the retention bits - is priced in microamps, and this desk has
  no current meter. What the driver can measure is time, flags, counters
  and the RAM's contents, so the bits are exposed, the interlocks
  enforced, and the comparison left to whoever has a meter. THE SAME
  METER WOULD DECIDE THE OPEN QUESTION OF THIS FAMILY: with the bus
  matrix serving the core alone, a program that moves data can either
  stay awake at a lower clock rate or stop between transfers, and which
  of the two is cheaper is not knowable from the documents.
- **The RAM's low-voltage mode, unused by either site's default.** 2.3.3
  offers RAMLV as "the lowest power consumption" and says nothing about
  what it does to the retention a Stop is chosen for; until a meter and
  a pattern test say otherwise, the configurations this stratum ships
  leave it clear.
- **The wake-up pad as a STANDBY EXIT.** EWUP claims PA0 and the bit
  reads back, and edges on that pad were driven from outside the
  CH32V203C8T6 while its core ran - which is how WUF was found to stay
  clear there. What is not measured on either part is the documented
  thing EWUP is for: a rising edge on it ending a Standby. An instrument
  on the pad with EWUP armed instead of the line would settle it; the
  CH32V303 board's pad is its key, which nobody presses here, and the
  CH32V203 board's is shared with USART2's CTS, a wire the serial
  chapter's flow-control letter wants left alone.
- **What the VBAT halves of the retention bits are worth.** R2KVBAT
  and R30KVBAT take a write, but neither board has a separate backup
  supply to remove, so nothing here can tell a bank kept on VBAT from
  one that was never left without VDD.
- **What the first retention bit covers on a 32 KB part.** 2.4.1 names
  "the 20K RAM" for the whole D6 class while the CH32V203C6, F6, G6 and
  K6 carry ten kilobytes; whether the bit reaches the whole of a smaller
  array, or only the low twenty that are not there, is not in the
  document. What would settle it is the suite's letter r - a pattern
  across the array, a Standby, the survivors counted - on one of those
  parts.
- **Remote wake-up and the USB controllers' own suspend.** A controller
  counts itself a bus master from its pull-up to its detach and not from
  packet to packet, which is the conservative reading: whether a
  SUSPENDED controller would let the program Stop is a question for the
  USB chapters and a meter.
- **A supply excursion through a PVD threshold.** The eight thresholds
  are all under both boards' fixed rails, so PVDO never changed state
  and the crossing was proved on the line and not in the analogue: what
  would measure it is a variable supply, or a part run at the bottom of
  its range.

Implemented but not bench-verified, each with what would measure it:

- **Letters i to k, r and x on the CH32V203C8T6** - the edge on the
  wire, a Stop at every rate of a DynamicClock, what a Standby keeps of
  the whole 20 KB bank, the watchdog through a Standby (the
  floating-point letter is the CH32V303's alone): they build for that
  part and its 32 KB tier, and what would measure them is the suite on
  that board, its wire from PA6 to PA1 in place.
- **The retention banks and the write-only bits of the CH32V20x_D8.**
  The CH32V203RB's two banks fold through the part table like the
  CH32V303's, and the kept copy serves whether that die reads the bits
  back or not; what would measure them is the suite's letter r on a
  CH32V203RB.
- **Whether bits 16 to 20 outlive a reset**, as 2.4.1's closing note
  says ("can only be reset by backup"): the driver does not rely on it,
  its copy starting at zero and the first store writing it over them.
  What would measure it is PWR_CTLR read at the boot after a Standby
  that set them, on the CH32V203C8T6, whose die reads them back.
- **The timed site on the LSI.** The arithmetic takes the RTC's rate as
  a stated upper bound and the internal oscillator's is a wide one
  (25 to 60 kHz on both parts), so the site works there at the cost of
  lateness; what would measure it is a letter that arms the same
  deadline on both sources and prints both spans.
- **The regulator's two trims as WRITES.** `core_voltage` and
  `low_power_voltage` are read at every boot and never written by the
  suite, because what a lowered core voltage does to the maximum clock
  rate is nowhere in the documents: what would measure it is a meter
  for the saving and a rate sweep for the price.
- **`Pwr::wait_for_interrupt()`, the bare WFI.** Nothing in this
  stratum uses it - the platform's idle path takes the WFE form, for
  the reason [platform.md](platform.md) gives - so whether this core's
  WFI wakes with the global mask clear is still that chapter's open
  question, and the verb is there for the program that means to ask it.
