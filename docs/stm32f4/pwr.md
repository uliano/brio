# Power control and the sleep sites - the stopping half (STM32F4)

Documents of record: RM0390 Rev 6 ch. 5 (the power controller of the
F446) and 33.16 (the debug support for low-power modes); RM0090 Rev 22
ch. 5, whose 5.4 is the F405 class's register description and whose 5.5
is the F42x/F43x's; RM0383 Rev 4 ch. 5 for the F411; DS10693 Rev 11
table 36 (the low-power wake-up timings the bench compares against);
the errata sheets' 2.2.1 ("Debugging Stop mode and SysTick timer") and
2.2.4 ("Wake-up sequence from Standby mode when using more than one
wake-up source"), both live on every revision of all four parts (the
same two numbers in ES0321).
Drivers: `stm32f4/pwr.hpp` (`Pwr`, `PwrMode`, `StopConfig`,
`VoltageScale`) and `stm32f4/sleep.hpp` (`Stm32f4SleepSite`,
`Stm32f4TimedSleepSite`), over `util/power.hpp`'s model
([../design/power.md](../design/power.md)); the way back up the clock
tree is `stm32f4/clock.hpp`'s `DynamicClock`
([clock.md](clock.md)). The family fixture is
`test/family_stm32f4/power.cpp` with the negatives that refuse a Stop
in the low-voltage mode on a part without the pair, under-drive without
the low-voltage mode it modifies, and a timed site whose RTC cannot
divide a second finely enough. The reference suite is
`test_stm32f4_power`.

## What the silicon does

**The mode is two registers in two places.** The Cortex-M4 has one bit,
SCB->SCR.SLEEPDEEP, which says "the next WFI is a deep sleep";
PWR_CR.PDDS says WHICH deep sleep (Stop or Standby), and PWR_CR's LPDS,
FPDS, MRLVDS/LPLVDS and UDEN say what a Stop costs to leave. Neither
register is the other's and both must agree before a WFI does anything
but Sleep, so `Pwr` owns them both: SLEEPDEEP is written in that file
and nowhere else in the stratum, `Pwr::mode()` is the one answer to
"what is armed", and `Stm32f4Platform::idle()` stays a DSB, a WFI and an
unmask that take whatever somebody else armed.

**Three modes, and only two of them are a ladder a program can resume
from** (5.3). Sleep stops the CPU clock and nothing else. Stop stops
every clock in the 1.2 V domain, disables the PLLs, the HSI and the HSE,
and KEEPS SRAM and every register. Standby powers the 1.2 V domain off:
"SRAM and register contents are lost except for registers in the backup
domain ... and Standby circuitry" (5.3.6), and the wake is a reset.

**What comes back from a Stop is not what went in.** 5.3.5: "When
exiting Stop mode by issuing an interrupt or a wakeup event, the HSI RC
oscillator is selected as system clock"; the PLL and the HSE are off,
over-drive is disabled ("entering Stop mode disables the Over-drive
mode, as well as the PLL") and the regulator's active scale is 3
("when the microcontroller enters in Stop mode, the voltage scale 3 is
automatically selected"). Everything OUTSIDE the clock tree survives:
the flash latency, the APB prescalers, every peripheral's registers and
the whole of SRAM. A program running at 180 MHz therefore resumes at
16 MHz with a SysTick reload and a USART divisor meant for eleven times
that - a fact of the chapter, and what the sleep site answers by
re-running the clock task in the first thing that executes after the
wake.

**The VOS field is not what a Stop changes.** Measured: after a Stop the
PWR_CR.VOS field still holds the value the program wrote (scale 1 on
this board) while PWR_CSR.VOSRDY is DOWN. 5.1.3's sentence is about the
regulator's ACTIVE output, which follows the field only "when the PLL is
ON" - so a Stop does not reset the register, it merely makes the
programmed scale inactive until the PLL comes back. ODEN and ODSWEN, by
contrast, ARE cleared by hardware (5.4.1: "cleared automatically by
hardware after exiting from Stop mode"), which was also measured.

**A Stop is only entered if nothing is pending.** Table 18's note: "all
EXTI Line pending bits ..., all peripheral interrupts pending bits, the
RTC Alarms ..., RTC wakeup, RTC tamper, and RTC time stamp flags, must
be reset. Otherwise, the Stop mode entry procedure is ignored and
program execution continues" - and the Cortex's WFI is itself a no-op
with an interrupt pending. A Stop that does not happen is not an error
condition anywhere in the silicon: the instruction falls through. So
everything that judges a sleep in this stratum judges it by TIME ELAPSED
and never by a flag.

**And it is only LEFT through the NVIC.** 5.3.5's exit table: a
WFI-entered Stop is left by "all EXTI lines configured in Interrupt mode
(the corresponding EXTI Interrupt vector must be enabled in the NVIC)".
Measured, and it costs a reboot when forgotten: a Stop entered with the
RTC's wake-up line configured but its NVIC channel disabled never comes
back, and the watchdog is what ends it.

**One wake-up flag for every source.** Unlike the STM32G0's per-pin
WUFx, this family has a single PWR_CSR.WUF raised by any WKUP pin OR any
of the RTC's five events (5.4.2), so it says THAT the device was woken
and never BY WHAT. SBF beside it says the device was in Standby and is
cleared only by a power-on reset or by CSBF, which is what lets the boot
after a Standby tell itself apart from the boot after a reset.

**Standby needs its flags clear, and an erratum on top of that.** Table
19 requires WUF down and the RTC flag of the chosen wake source down
before the entry, or the entry silently does not happen. ES0298 2.2.4
(ES0206 2.2.4, ES0287 2.2.4) adds that "the various wake-up sources are
logically OR-ed in front of the rising-edge detector"; a source held
high while CWUF clears the flag "may mask further wake-up events on the
input of the edge detector" and the device then never wakes. The
workaround is the erratum's own four steps and `prepare_standby()` is
them: disable every wake-up source, clear the flags, re-enable, enter.

**The wake-up pins have no polarity to choose.** 5.4.2: an enabled WKUPn
pin is "forced in input pull down configuration" and a RISING edge is
the wake, so a program does not configure the pad at all and must not
drive it. The note that costs a sleep: "an additional wakeup event is
detected if the WKUP pin is enabled ... when the WKUP pin level is
already high" - which is why the driver clears the flag after the
enable, and why a pad held high by a board makes Standby unreachable.

**The debug bits decide whether a low-power mode happens.** DBGMCU_CR's
DBG_SLEEP, DBG_STOP and DBG_STANDBY (33.16.1) feed FCLK and HCLK from
the internal RC through the mode so a probe can still reach the core.
The register "is asynchronously reset by the PORESET (and not the system
reset)" and OpenOCD's own `target/stm32f4x.cfg` writes all three at
every connection, so a board that has seen a probe holds them without
any program having asked - and ES0298 2.2.1 is the consequence that
matters: "if the SysTick timer interrupt is enabled during the Stop mode
debug ..., it wakes up the system from Stop mode". A Stop that looks
like it lasts a millisecond is a debugged board and not a silicon fact.

**What is offered and never set.** PWR_CR's FISSR and FMSSR stop the
flash interface, or the flash itself, WHILE THE SYSTEM RUNS - and 5.4.1
says "this bit could not be set while executing with the Flash itself.
It should be done with a specific routine executed from RAM". Nothing in
brio runs from RAM on this family, so the two verbs exist and no code
here calls them. ADCDC1 is AN4073's and is the same kind of offer.

**What is elsewhere.** PWR_CR.DBP, the backup domain's write gate, is
`RtcDomain::unlock()` in `stm32f4/rtc.hpp`: one register bit, one owner,
and the owner is the chapter whose registers it unlocks. The BOR levels
are option bytes and belong to the flash chapter's provisioning verb.

## The ladder, and why it is not the identity

| depth | this target | what it is |
|---|---|---|
| `none` | Sleep | SLEEPDEEP = 0: the CPU clock stops, HCLK, SysTick and every peripheral keep running |
| `light` | Sleep | THE SAME MODE - see below |
| `standby` | Stop, main regulator | every 1.2 V clock stops, SRAM and registers retained, the fastest wake this silicon has |
| `deep` | Stop, low-power regulator + FPDS | the same mode, cheaper to hold and slower to leave |

**`light` and `none` are one code**, and that is the honest answer rather
than a shortcut. `util/power.hpp`'s rule is that a target "maps what it
does not have to the nearest SHALLOWER mode - never deeper than asked".
Between Sleep and Stop this family has NOTHING - 5.3 lists three modes
and no low-power run - so `light` maps to Sleep, and `armed()` answers
`none` for it because that is what the machine will really do.

**The two deep rungs are one mode with two prices.** This family's Stop
is a single mode whose cost is set by LPDS, FPDS and the low-voltage
pair, so the ladder's deep rungs are two `StopConfig`s and not two
modes - and `armed()` cannot tell them apart from the registers alone.
It reports the depth the site last armed, and `deep` for a Stop somebody
else armed through `Pwr`. The two configurations are the site's template
argument, so a program that wants under-drive on the parts that have it,
or a plain main-regulator Stop for both rungs, says so where it names
the site.

**Standby is off the ladder on purpose, and the site ENFORCES it**: every
`arm()` goes through `Pwr::arm(PwrMode::stop, ...)`, which clears PDDS,
so no rung can reach Standby and a PDDS left standing by anything else
is taken down at the next round. The model is built on the program
RESUMING - "the manager's next dispatch, of ANY event, first disarms the
site and publishes a WakeReport" - and after a Standby there is no next
dispatch. Standby stays reachable, as it should be, through
`Pwr::enter(PwrMode::standby)`: a deliberate one-shot whose resumption
is the application's boot path reading PWR_CSR.SBF and the RTC's backup
registers.

**A Stop entered with the kernel's tick armed lasts** - on a board no
debugger has touched. 5.3.3 enters a low-power mode through WFI only if
no interrupt is pending, and a 1 kHz SysTick raises one every
millisecond; but once the WFI is taken HCLK stops and SysTick with it,
so that window is one instruction wide and not a coin toss. `arm()`
PAUSES the ticker for the deep rungs and `disarm()` resumes it: it costs
nothing (a Stop stops SysTick anyway and kernel time was going to stand
still for the whole sleep either way), it closes the pending-tick window
by construction, and it is ES0298 2.2.1's own workaround - "to debug the
Stop mode, disable the SysTick timer interrupt" - applied whether or not
a probe is attached.

## The timed site

The plain site keeps an HONEST RESTRICTION: with kernel time frozen for
the whole Stop, a program with armed time events must not take one.
`Stm32f4TimedSleepSite` lifts it, inside `arm()` and `disarm()` alone,
with the RTC in both roles:

- the ALARM is the periodic wake-up timer (17.3.5), placed on
  `TimeEvents<P>::ticks_to_next()` rounded up. 5.3.7 calls this the
  device's auto-wake-up and it is the one counter on this family that
  runs with every 1.2 V clock stopped;
- the WITNESS is the calendar plus the sub-second counter; the
  difference between the reading at `arm()` and the reading at
  `disarm()` is how long the world moved, and subtracting what SysTick
  itself counted leaves the FROZEN span, which goes to
  `Ticker::advance()`.

**The rate rule is directional**: state an RTCCLK rate NOT BELOW the
true one. Over-estimating makes the prescalers divide too hard, so
ck_spre runs slow, so the witness UNDER-reports the span and the resync
UNDER-advances; and it makes the alarm ask for more counts than needed,
so the wake lands LATE. Both errors land on the side the kernel's time
contract allows: at least, never early.

**The other prescaler split.** 17.3.1 advises a high asynchronous factor
to save current; that is right for a calendar and wrong here, because
PREDIV_S is how finely the sub-second counter divides a second and that
counter is the site's only way of measuring a span the tick did not
count. `rtc_prescalers_for_resolution()` is the deliberate opposite
choice - 0/32767 at 32768 Hz, 30.5 us a step - and the config predicate
refuses a split that divides the second fewer than a thousand ways,
because a resync quantized more coarsely than the kernel tick it repairs
can advance a tick too many and mature an event EARLY.

**The ISR has four acts**, and every one is load-bearing: restore the
clock (so everything after it runs at full speed and the console's
divisor is right again); acknowledge the EXTI line and the RTC flag;
resync the ticker; and hand the machine back to a TICKING sleep, because
the never-early bias guarantees kernel time is still a shade short of
the deadline when the alarm lands and an RTC wake posts nothing to any
queue. A FOREIGN wake does not run this body: its progress is its own
event, the resync happens in `disarm()` instead, and the Stop stays
armed with the alarm still standing - which is why the model's
convention (a wake path with nothing to say sends `SleepRequested{none}`)
is load-bearing with this site.

**What the site owns: the RTC, whole** - the domain gate, the clock
select, the prescalers, the calendar and the wake-up timer. An
application using it must not drive `stm32f4/rtc.hpp` elsewhere, and
must bind `RTC_WKUP_IRQHandler` to `Site::isr()` AND enable that NVIC
line, without which the Stop is never left.

## Types and verbs

- `PwrMode::sleep | stop | standby`, `pwr_mode_resets(m)`,
  `pwr_mode_stops_clocks(m)`.
- `StopRegulator::main | low_power`; `StopConfig{regulator,
  flash_power_down, low_voltage, under_drive}` and
  `stop_config_valid(c)` (the low-voltage pair and the under-drive field
  must exist on the part; under-drive is a modifier of the low-voltage
  mode and never a mode of its own).
- `Pwr` (monostate, every verb opening the APB1 gate first) -
  `bus_clock`, `cr`, `csr`; the regulator half the clock chapter uses
  (`scale`, `scale_ready`, `scale_exists`, `has_over_drive`,
  `over_drive_enter`, `over_drive_ready`, `over_drive_active`,
  `over_drive_exit` - the last one waiting for ODSWRDY to fall, 5.1.4's
  sequence 1 complete); the mode (`deep_sleep`, `sleep_on_exit`,
  `event_on_pending`, `power_down_deep_sleep`, `stop_config` both ways,
  `arm(mode)` and `arm(mode, StopConfig)`, `mode()`,
  `wait_for_interrupt`, `wait_for_event`, `enter(mode, use_wfe =
  false)`); the wake-up pins (`wakeup_pin_count`, `wakeup_pin_present`,
  `wakeup_pad`, `wakeup_pin(n, on)`, `wakeup_pin_enabled`,
  `wakeup_pins(on)`); the flags (`wakeup_flag`, `clear_wakeup_flag`,
  `standby_flag`, `clear_standby_flag`, `clear_wakeup_flags`,
  `prepare_standby`); the detector (`pvd_levels_known`, `pvd_level_mv`,
  `pvd_level` both ways, `pvd_enable`, `pvd_enabled`, `pvd_below`,
  `pvd_exti_line`); the backup regulator (`has_backup_sram`,
  `backup_sram_clock`, `backup_regulator` both ways with the BRR wait,
  `backup_regulator_ready`); under-drive (`has_under_drive`,
  `under_drive_flag`, `clear_under_drive_flag`); the offers
  (`has_flash_stop_while_run`, `flash_interface_stop_in_run`,
  `flash_stop_in_run`, `has_adc_dc1`, `adc_dc1`); and the debugger's
  three bits (`debug_in_sleep`, `debug_in_stop`, `debug_in_standby`,
  `debug_low_power(on)`).
- `SleepSiteConfig{standby, deep}` and `sleep_site_config_valid(c)`.
- `Stm32f4SleepSite<Clock, Timebase = Ticker, cfg = {}>` - the
  `SleepSite` concept's `arm`/`disarm`/`armed`, plus `resume_clock()`,
  `pauses_tick` and `config`.
- `TimedSleepConfig{rtcclk_hz, source, wipe_domain, fast_clock}` and
  `timed_sleep_config_valid(c)`.
- `Stm32f4TimedSleepSite<Platform, Clock, cfg = {}, site_cfg = {}>` -
  the same three verbs plus `init()`, `ready()`, `alarm_armed()`,
  `place_alarm(ticks)`, `last_advance()`, `last_reload()`,
  `last_alarm_was_fast()`, `time_of_hour_ms()`, `elapsed_ms(from, to)`,
  `resync()`, `isr()`, and the constants `prescalers`, `fast_hz`,
  `fast_span_ticks`.
- The reserve (`stm32f4/device_tables.hpp`): `pwr_has_under_drive`,
  `pwr_has_low_voltage_stop`, `pwr_has_flash_stop_while_run`,
  `pwr_has_adcdc1`, `pwr_wakeup_pin_mask(n)`, `pwr_wakeup_pin_count()`,
  `PwrWakeupPad` + `pwr_wakeup_pad(n)`, `PvdLevels` +
  `pwr_pvd_levels()`, `pwr_pvd_exti_line`,
  `pwr_backup_sram_clock_mask()`.

## How to use it

A program that only wants the shallow rung and a clock that survives a
Stop:

```cpp
#include "stm32f4/sleep.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000,
                             8'000'000, brio::HseMode::bypass>;
using Site = brio::Stm32f4SleepSite<SysClock>;

// The manager arms; the kernel's idle path is what stops.
using Pm = brio::PowerManager<P, Site, brio::PowerConfig{}, Engine>;
```

A program whose time events must survive the Stop hands the RTC to the
timed site and binds its vector:

```cpp
using Site = brio::Stm32f4TimedSleepSite<P, SysClock>;   // LSE, 32768 Hz

extern "C" void RTC_WKUP_IRQHandler() { Site::isr(); }

int main() {
    SysClock::init();
    brio::Ticker::init(clock);
    Site::init();                              // the domain, the split, the line
    brio::Nvic::enable(brio::Rtc::wakeup_irq());   // or the Stop is never left
    brio::enable_interrupts();
}
```

A Stop priced differently - the main regulator for both rungs, so the
wake is as fast as this silicon gets:

```cpp
constexpr brio::SleepSiteConfig fast{
    .standby = brio::StopConfig{},
    .deep = brio::StopConfig{brio::StopRegulator::main, false, false, false},
};
using Site = brio::Stm32f4SleepSite<SysClock, brio::Ticker, fast>;
```

Standby, the one-shot off the ladder:

```cpp
brio::Rtc::backup(3, token);                 // what survives
brio::Rtc::set_wakeup(brio::RtcWakeupClock::ck_spre, 1);
brio::Nvic::enable(brio::Rtc::wakeup_irq());
brio::Pwr::enter(brio::PwrMode::standby);    // prepare_standby() is inside
// ... the next boot reads Pwr::standby_flag() and the backup register
```

## Bench findings

On an STM32F446RE (DEV_ID 0x421, REV_ID 0x1000) at 3.3 V, the PLL at
180 MHz in over-drive from the ST-LINK's 8 MHz MCO in bypass, the LSE
crystal fitted and the RTC split PREDIV_A 0 / PREDIV_S 32767 so the
sub-second counter is a 30.5 us stopwatch. `test_stm32f4_power`, 81
verdicts in `z` plus the Standby letter by name. On the 32F469IDISCOVERY
(180 MHz from its 8 MHz crystal, the LSE fitted) the same `z` is 80
verdicts green and the Standby letter comes back through the reset
vector with SBF set and its backup token intact.

**The block at 180 MHz in over-drive** reads PWR_CR 0x3C100 (DBP, VOS
11, ODEN, ODSWEN) and PWR_CSR 0x34000 (VOSRDY, ODRDY, ODSWRDY). Two
wake-up pins are bonded: WKUP1 on PA0 and WKUP2 on PC13.

**Sleep keeps everything but the CPU clock.** 100 WFIs whose only wake
is the 1 kHz tick take 99 ms of kernel tick and 98 ms of wall; 200 of
them take 198.06 ms of wall against the 200 ms the ticks alone would
be, so a Sleep's whole wake-and-loop cost is well inside a tick.

**A Stop stops kernel time.** A 200 ms alarm on RTCCLK/16 (reload 408 at
2048 Hz) gives 199 ms of wall and 0 ms of kernel tick. At the first
instruction after the wake: SYSCLK on the HSI, the PLL off, the HSE off,
over-drive off, and PWR_CR.VOS still reading scale 1 with VOSRDY down.
The site's `disarm()` put the whole tree back in 213 us.

**The wake-up timer starts at WUTE, not at the WFI**: an alarm
programmed before a console drain has already spent that time when the
Stop begins (measured: 5 ms of a 200 ms sleep). Every measured sleep in
the suite arms the alarm after the wall is stamped.

**The regulator variants, 32 Stops each at 16 MHz on the HSI** (where no
PLL restart hides inside the number), the alarm 4394 us apart:

| variant | per lap | above the main regulator | DS10693 table 36, typical difference |
|---|---|---|---|
| main regulator | 4545 us | - | - |
| low-power regulator | 4546 us | 1 us | 8 us |
| main + flash power-down | 4637 us | 92 us | 92 us |
| low-power + flash power-down | 4637 us | 92 us | 100 us |
| low-power + FPDS + low voltage | 4638 us | 93 us | - |
| low-power + FPDS + under-drive | 4638 us | 93 us | 101 us |

The FLASH's own wake is the step that shows on the STM32F446: 92 us
measured against the datasheet's 92 us typical, and 91 us on the
STM32F469 (laps of 4546 and 4637 us, the low-voltage and under-drive
variants at 4638) - and NO step at all on the STM32F411, where every variant leaves at the same 4545 us lap (a
fact of the part: the suite prints the step and judges its direction
only). THE REGULATOR'S IS NOT: the eight
microseconds table 36 puts between the main and the low-power regulator
did not appear above the HSI's own startup here, and neither did the
extra microsecond the under-drive rows promise. PWR_CSR.UDRDY stands
after an under-drive Stop, which is what says the deepest variant really
engaged.

**The SYSCLK restore costs 213 us**, from the HSI a Stop leaves behind
to 180 MHz in over-drive with 5 wait states and the APBs at 45 and
90 MHz - sixteen restores from real Stops, 213 us average and 213 us
worst. It runs WITH the peripheral clocks enabled, which 5.1.4's boot
recipe ("during the Over-drive switch activation, no peripheral clocks
should be enabled") would have had off: the note is a boot recipe and
not a condition of the switch.

**A Stop through a real kernel** (the `PowerManager`, one voter, the RTC
250 ms out, a 500 ms time event): the vote round runs, the site is armed
at `deep`, the first event after the wake ends the round and puts the
clock back, and the event matures after 500 ms of KERNEL TICK and 727 ms
of wall - the plain site's honest restriction, visible as the 227 ms the
tick never counted. One not-ok ends a round before anything is armed; a
standing `PowerLock` at `light` reaches the voter as the CLAMPED depth;
and a deadline nearer than `min_deep_ticks` refuses a deep round before
anyone is asked.

**The timed site meets the deadline on the wall.** Its fast alarm clock
is 2048 Hz and reaches 31999 kernel ticks; a 500 ms deadline places
reload 1023, exactly what the stated arithmetic gives. The round: 500 ms
of wall, 501 ms of kernel tick, and 501 ticks handed back by the resync.
Six shorter rounds of 150 ms all landed at 150 ms of wall, not one of
them early. A deadline-less round places no alarm at all, and a round
that never slept advances at most a tick.

**A deadline of N ticks is N tick BOUNDARIES**, so arming one mid-tick
makes the wall span up to a millisecond short of N - a property of a
1 kHz tick and not of any sleep. The suite arms every wall-judged
deadline from a tick edge, which takes that millisecond out of the
measurement instead of out of the verdict.

**The console's ring being empty is not the wire being empty.** A Stop
taken while the last character is still in the USART's shift register
truncates it: the line's own CRLF was lost and the host resynchronized
several bytes into the next line. The transport's `tx_idle()` reports
the ring; TC reports the wire, and the suite waits for both before every
measured sleep.

**Standby works and comes back through the reset vector.** A token in an
RTC backup register, the wake-up timer 2 s out, `Pwr::enter(standby)`:
the next boot finds PWR_CSR.SBF set and the token intact.

**The debugger's three bits were all SET at boot** - OpenOCD's
`target/stm32f4x.cfg` writes them at every connection and they survive
every reset but a power-on. `main()` takes them down before a sleep is
weighed. A CURIOSITY worth knowing: once the probe has detached, a write
to DBGMCU_CR does not take - the three bits read back 0 after being
written as a set - while DBGMCU_IDCODE beside them still reads DEV 0x421
REV 0x1000. So the register can be read and cleared at boot, when the
debug power domain is still up, and is inert afterwards. With a probe
still attached (an STLINK-V3 over SWD on the STM32F411) a later set
takes and a later clear does not - the register belongs to the debug
domain and the probe, not to the program; what a sleep-measuring program
can rely on is the clear it makes at boot, and the frozen tick is the
proof that it took.

**The PVD** takes all eight of its codes (2.0, 2.1, 2.3, 2.5, 2.6, 2.7,
2.8, 2.9 V on this part class) and never reports VDD below any of them
at this board's supply. Its EXTI line is 16, a CONFIGURABLE line, and it
takes the software trigger - which is how the wake path is proved with
no supply to move.

**Both wake-up pins arm with WUF clear**, so neither PA0 nor PC13 idles
high on this board and neither would block a Standby. ES0298 2.2.4's
sequence disables, clears and re-enables: an armed pin survives it.

**The backup regulator** comes up and reports BRR ready, and the backup
SRAM's own AHB1 gate opens and closes; this part has the array behind
the bit.

## Not covered yet

Driver gaps:
- `FISSR` and `FMSSR` are offered and never exercised: 5.4.1 forbids
  setting them from code executing out of the flash, and nothing in brio
  runs from RAM on this family - the verbs wait for the program that
  brings its own `.ram_text` routine.
- `ADCDC1` likewise: its meaning is AN4073's, which is not on the desk.
- Low-power run and low-power sleep do not exist on this family, so
  nothing maps to them; `light` is Sleep and says so.
- A TICKLESS timebase (the STM32G0's LPTIM shape) that counts through a
  Stop and would make the timed site unnecessary: this family's LPTIM1
  is not driven yet, so the platform's `idle_until()` hook is not
  offered and the sites repair a SysTick instead.
- The VBAT domain's own supply and the switch to it: a battery on VBAT
  is a wire this desk has not got, so `backup_regulator()`'s promise
  (the array's content kept through Standby AND VBAT) is proved for the
  flag and not for the retention.
- Waking from Standby through a WKUP PIN: neither PA0 nor PC13 can be
  driven here without a wire or a human, and the user button's press is
  not something a suite may wait for. The pins are armed and their flag
  read; the press is the gap.
- The current a mode really costs: no meter on the supply, so every
  number here is TIME and none of them is microamperes.

Implemented, not bench-verified:
- The under-drive variant's own wake cost as a NUMBER: it engaged
  (UDRDY) and cost the same as the plain low-power Stop to within a
  microsecond, where the datasheet promises one more - on the STM32F446
  and again on the STM32F469 (92 us over the main regulator against the
  91 of the flash in power-down alone). A colder or hotter part would
  say whether the table's typical is conservative or both parts are fast.
- `Pwr::enter(mode, use_wfe = true)` - the WFE path. Every sleep here is
  a WFI, which is what the kernel's idle hook executes; the WFE entry
  and its SEVONPEND interaction (5.3.3) would want an EXTI line in event
  mode to wake it.
- `sleep_on_exit` and `event_on_pending`: read and asserted clear, never
  set - a brio program's idle path is the kernel loop's, and an ISR that
  never returned to it would starve every queue.
- The one-bit VOS path and the parts without the low-voltage pair or the
  under-drive field (the F405 class): compiled on their headers by
  `brio check stm32f4`, no board.
- Three wake-up pins (the F410, F412 and F413 classes) and the pads of
  any pin past the first on a class whose datasheet is not on the desk:
  `pwr_wakeup_pad()` answers `known == false` there rather than guessing.
- The timed site on the LSI (a board without a crystal states a rate
  above DS10693's upper bound, which makes every deadline late and never
  early): the site is compiled and its deadlines met on the STM32F429,
  but the LSI's real rate there is what the suite WEIGHS against the
  core before it converts a wall reading (33.5 kHz against 47 stated),
  and a site that wanted the wall's accuracy would take the weighed rate
  the same way - not done, no program has asked.
