# POWMAN and the always-on timer (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 6.2 (the core
power domains, the twelve power states and the transitions between
them), 6.3 (the core voltage regulator), 6.4 (the register list and the
password), 12.10 (the always-on timer), 3.5.10 (what the debug port's
RP-AP can see and do to the power sequencers), 7.3 (the chip-level reset
record, which lives in this block). Appendix E carries no erratum against
POWMAN or its timer. The driver: `brio/rp2350/powman.hpp` (`Powman`,
`AonTimer`) over `clock.hpp` (the low-power oscillator's trim and the
frequency counter) and `core.hpp`. The reference suite:
`test_rp2350_sleep`, letters a, b, c, d, e, k - and the by-name letters
q and r, which are the only place a program of this project powers the
switched core down.

## What the silicon does

THE ALWAYS-ON DOMAIN IS A PERIPHERAL. Where the RP2040 had a power
controller that was little more than a regulator and a reset latch, this
chip has a block that keeps running when the processors, the bus fabric
and every other peripheral have no supply at all: POWMAN. In it live the
power sequencers, the core regulator and the brown-out detector, the
chip-level reset record, eight scratch words, the four boot-vector words
the bootrom reads, and a 64-bit timer.

**Five domains.** 6.2.1 splits the digital core into AON (this block),
SWCORE (the processors, the fabric, every peripheral), XIP (the cache and
the boot RAM), SRAM0 (banks 0..3) and SRAM1 (banks 4..9). A power state
is named Pc.m: c is the switched core, m the three memory domains in the
order XIP, SRAM0, SRAM1. STATE.CURRENT and STATE.REQ carry that as four
bits where a ONE means powered DOWN, and `PowerState` is those twelve
codes under the datasheet's own names. A P0.m state is a normal one -
software runs. A P1.m state has no processor in it, and LEAVING ONE IS A
BOOT.

**The password.** Every register up to offset 0xac takes a write only
with 0x5AFE in its top sixteen bits; a write without it is dropped and
sets BADPASSWD, which is itself protected. A READ of a protected register
does not return the password, so an ordinary read-modify-write would
write a zero password back - and the atomic aliases of 2.1.3 are unusable
here for the same reason, since they carry bits and not a password. Past
0xac the eight scratch words, the four boot words and the three interrupt
registers are ordinary.

**The transitions** (6.2.3, and the STATE register's own description,
which is what the silicon obeys): a request may not combine a power-up
with a power-down; it may not leave the switched core powered with the
XIP domain unpowered; and a P1.m state may only be left for a P0.m one.
The prose of 6.2.3 states a fourth rule - that leaving a P1.m state may
not power a powered-off SRAM domain back on - which its OWN table
contradicts, P1.1 -> P0.0 being listed as valid, and which the register
description does not repeat. `power_transition_legal` enforces the
register's three.

**The regulator and the detector are read-only in this tree**, by
decision and not by omission. A wrong VSEL is a voltage on the one
digital core supply this board has, and VREG_CTRL.UNLOCK cannot be undone
once it is set - the chapter's own words are "the regulator can't be
relocked after it's been unlocked". What a program gets is `VregStatus`,
`VregLowPower` and `BodStatus`, decoded in millivolts. What it does NOT
get is any verb that writes a threshold; the family check asserts their
absence.

**A debugger can block a power-down.** The SW-DP's CSYSPWRUPREQ powers
every domain and inhibits software's own transitions (6.2.3.3). A probe
that has attached once is very likely to leave it asserted when it
detaches, and then a power-down request is answered with REQ_IGNORED and
nothing happens. DBG_PWRCFG.IGNORE is the way past it, and on this bench
it is needed every time (below).

**The always-on timer** (12.10) shares the block, the password and the
domain. It counts MILLISECONDS to 64 bits off whichever source TIMER
names - the low-power oscillator, the crystal through clk_ref, or an
external tick on one of four pads - and its one alarm is both an
interrupt and a POWER-UP REQUEST. The tick is made by a fractional
divider whose divisor is A NUMBER SOFTWARE WRITES, not a measurement the
hardware makes: 32.768 for the low-power oscillator, 12000.0 for a 12 MHz
crystal. So the timer is exactly as accurate as the rate it has been
told, and on a die whose low-power oscillator is well away from its
nominal rate the default divisor is simply wrong. `AonTimer::calibrate`
is the answer.

## Types and verbs

- `PowerState` (p0_0..p0_3, p1_0..p1_7 - the value IS the four-bit
  field), `state_code`, `is_low_power_state`, `PowerDomainBit`,
  `power_transition_legal`, `PowerRequest` (accepted / illegal / no_wake
  / busy / ignored / bad_request), `PowerUpSource` and `pwrup_bit`.
- `Powman`, a monostate over the block:
  - the state machine - `current_state`, `requested_state`, `state_word`,
    `changing`, `waiting`, `bad_software_request`, `bad_hardware_request`,
    `powerup_while_waiting`, `request_ignored`, `clear_request_flags`,
    `wait_settled`; `request_state(to)` for a move between P0.m states
    and `power_down(to)` for a P1.m one - the second REFUSING when no
    wake source is armed, which `powerup_armed()` is the question behind;
  - the four GPIO power-up slots - `pwrup(slot, PwrupConfig{source, mode,
    direction})`, `pwrup_disable`, `pwrup_enabled`, `pwrup_source`,
    `pwrup_raw`, `pwrup_status`, `pwrup_clear`, `pwrup_disable_all`;
    `pwrup_source_valid` judges a source number against the package, and
    `PwrupQspiSource` names the six QSPI pads 6.4 numbers past the GPIO;
  - `current_powerup_requests`, `last_powerup_requests`,
    `last_powerup_source` - the boot-side answer to "why am I running";
  - the debugger - `debug_powerup_pending`, `ignore_debug_powerup`,
    `ignoring_debug_powerup`;
  - the sequencer - `sequencer()` returning `SequencerConfig`,
    `use_fast_clock`, `run_lposc_in_low_power`,
    `sram_stays_down_on_powerup`;
  - the supplies, READ-ONLY - `vreg()`, `vreg_low_power_entry()`,
    `vreg_low_power_exit()`, `bod()`, and the two tables `vreg_vsel_mv`,
    `bod_vsel_mv` beside `high_temp_threshold_c`;
  - `scratch(n)` / `scratch(n, value)` for the eight words, `boot_word(n)`
    for the four the bootrom reads;
  - the password - `bad_password`, `clear_bad_password`, and
    `probe_password()`, THE ONE WRITE IN THE TREE THAT CARRIES NO
    PASSWORD, aimed at BADPASSWD's own bit so that the answer costs
    nothing either way;
  - the interrupts - `Powman::Interrupt`'s four bits, `raw_interrupts`,
    `clear_raw`, `interrupt`, `interrupts`, `force`, `forced`, `pending`,
    and the two lines `irq_pow()` / `irq_timer()` (`isr_powman_pow` and
    `isr_powman_timer`, the same names on both architectures).
- `AonTimer`, the counter in the same block: `init(clock)` (the divisors
  written from a MEASUREMENT of the low-power oscillator and from the
  clock task's own crystal rate, the time zeroed, the counter started,
  the alarm left disarmed), `running` / `run`, `now` (12.10.3's read
  procedure) / `now_low`, `set` (refused while running), `clear`;
  `alarm_at` / `alarm_in` / `alarm_time` / `alarm_enable` /
  `alarm_enabled` / `alarm_fired` / `clear_alarm`, `powerup_on_alarm`,
  `interrupt` / `interrupt_enabled` / `pending` / `isr()`;
  `source()` and `AonSource`, `use_xosc` (refused unless clk_ref is the
  crystal) / `use_lposc`, `synchronised_to_gpio_1hz`,
  `time_reference(AonTimeRefPin, drive_low_power_clock)`;
  `lposc_khz` / `xosc_khz` / `lposc_khz_int` / `lposc_khz_frac` /
  `xosc_khz_int` / `xosc_khz_frac` / `declared_hz` / `calibrate(clock)`.

## How to use it

```cpp
brio::AonTimer::init(clock);              // the divisor from a measurement
brio::AonTimer::alarm_in(500);            // 500 ms out
brio::AonTimer::interrupt(true);
brio::Irq::enable(brio::AonTimer::irq());
extern "C" void isr_powman_timer() { (void)brio::AonTimer::isr(); }
```

A power-down of the switched core is a verb of its own, and its return is
a boot:

```cpp
brio::Powman::scratch(0, my_token);              // the only state that survives
brio::AonTimer::alarm_in(3000);
brio::AonTimer::powerup_on_alarm(true);          // the way back
if (brio::Powman::debug_powerup_pending()) {
    brio::Powman::ignore_debug_powerup(true);    // a probe has been here
}
if (brio::Powman::power_down(brio::PowerState::p1_0) == brio::PowerRequest::accepted) {
    for (;;) { brio::Rp2350Platform<>::idle(); }  // the sequencer waits for the cores to halt
}
```

At the next boot, `Powman::last_powerup_source()` says what woke it and
`Reset::causes()`'s `swcore_powerdown` says the boot was a power-up and
not a reset.

## Bench findings

The reference suite is `test_rp2350_sleep`, green on both architectures
on the WeAct RP2350B board (stepping A2), with the always-on timer as the
only ruler that answers in every state.

- **AS FOUND, at rest under a program.** P0.0; no transition in flight
  and neither BAD_* flag standing. The sequencer
  runs the block off clk_ref
  while the switched core is powered, keeps the low-power oscillator
  running when it goes, and is set to move both supplies into their
  low-power modes on the way down and back on the way up. Both SRAM
  domains are set to be powered up again on the way back.
- **THE REGULATOR** reads VSEL 0x0B = 1100 mV with VOUT_OK set, never
  unlocked, the voltage limit on, over-temperature protection at 125 C.
  VREG_STS.STARTUP stands and goes on standing: 6.3.4 says it is high
  until the mode or the voltage is changed, and nothing here changes
  either. The sequencer's own low-power setting is the LINEAR mode at the
  same 1100 mV, and the way out is the switching mode again.
- **THE BROWN-OUT DETECTOR reads 946 mV, WHICH IS NOT THE THRESHOLD
  6.4'S TABLE MARKS AS THE DEFAULT.** The table puts "(default)" on code
  01001 = 0.860 V; the register's own stated reset value is 0x0b1 - code
  01011, 0.946 V - and that is what the silicon reads. The reset value
  wins and the marker is on the wrong row. The same code in the
  regulator's table IS its default (1.10 V), which is probably how the
  two got confused.
- **BOTH POWER-UP REGISTERS ARE A SET AND NOT A NUMBER.** 6.4 lists their
  sources as "0 = chip reset, 1 = pwrup0 ... 6 = alarm_pwrup", which
  reads like an index. The silicon answers 0x01 for a chip reset, 0x20
  for the debug port and 0x40 for the alarm: one bit per source in a
  seven-bit field.
- **THE PASSWORD IS A PASSWORD.** A store of BADPASSWD's own bit with no
  password in it is dropped and RAISES the flag instead of clearing it;
  the clear needs the password too. The same field written through the
  driver takes and raises nothing. The scratch words, past offset 0xac,
  take a plain 32-bit store.
- **THE LOW-POWER OSCILLATOR ON THIS DIE RUNS 27906 Hz** against its
  nominal 32768 - fifteen per cent slow at the factory trim of 32. That
  is the whole reason `AonTimer::init` measures it: with the divisor told
  what the frequency counter found, two seconds of the always-on timer
  took 1 998 735 us of the crystal's ruler, an error of 6 parts in
  10 000. On the crystal, 1000 ms took 999 999 us.
- **THE ALARM** reads back as written, fires once, and the handler enters
  in the same millisecond as the match - which is the timer's whole
  resolution. Its ISR body must DISARM as well as acknowledge: the match
  goes on being true after the event, so an armed alarm whose flag is
  merely cleared raises it again on the next tick. An alarm disarmed
  before its match never fires.
- **BOTH SOURCE SELECTS ARE LIVE**, with no need to stop the counter, and
  each self-clears when the change has been made. A divisor write is
  refused while the counter runs off THAT source and taken for the other.
- **SEQ_CFG.USE_FAST_POWCK TAKES EFFECT AT ONCE.** 6.4 says the setting
  "takes effect when a power up sequence is next run"; measured, the
  report bit USING_FAST_POWCK follows the write immediately, both ways.
  (It does not, however, do what a dormant needs - see
  [sleep.md](sleep.md).)
- **A POWER-DOWN INTO P1.0**, the alarm three seconds out as the power-up
  request: the chip comes back through a full boot, `LAST_SWCORE_PWRUP`
  reading 0x40 (the alarm) and `Reset::causes()` carrying
  `swcore_powerdown` and nothing else. THE ALWAYS-ON BLOCK'S EIGHT
  SCRATCH WORDS CROSS IT; the watchdog's four do not, living in the
  switched core. The `.noinit` SRAM crosses P1.0, which keeps both memory
  domains powered, and does not cross P1.7, which does not.
- **A POWER-DOWN INTO P1.7**, the deepest state this chip has - the
  switched core, the XIP cache and both SRAM domains unpowered, the
  always-on domain and its timer the only thing left - comes back the
  same way and with the same record, and the `.noinit` SRAM does not
  cross it.
- **AND THE DEBUG PORT'S RESCUE DOES NOT REACH A POWERED-DOWN CORE.**
  3.5.8 says the rescue reset works "even when system clocks are stopped
  and the switched core power domain is powered down". The first half is
  measured true - it ends a dormant in about two seconds
  ([sleep.md](sleep.md)) - and the second is NOT reproduced here: against
  a chip in P1.0 the same DAP-only session reads the debug port's DPIDR
  (0x4c013477) and then never finishes `init`, so it never reaches the
  RP-AP's control register at all. Tried three times with the session
  bounded well inside a sixty-second alarm net, the board came back each
  time on its own alarm and named it (`LAST_SWCORE_PWRUP` = 0x40). What
  cannot be told apart from this side of the wire is the RP-AP being
  unreachable and the debugger's own ADIv6 access-port enumeration
  hanging on the mem-APs that have no power; either way the practical
  fact is the same, and it is why `power_down()` REFUSES without an armed
  wake source. That guard is the only way back from a P1.m state on this
  bench. A failure AFTER the wake is a different matter and a safe one:
  the chip is then in P0.0 with the core powered, where the rescue works.
- **THE DEBUG PORT BLOCKS EVERY POWER-DOWN ON THIS BENCH.** After
  `brio flash` has exited, CURRENT_PWRUP_REQ reads 0x20 - the debug
  port's CSYSPWRUPREQ still asserted by a probe that is no longer
  attached. In that condition a power-down request is answered with
  REQ_IGNORED and the core stays up. Writing DBG_PWRCFG.IGNORE clears the
  request from the block's point of view at once, and the power-down then
  proceeds. Every letter of the suite that stops the chip prints which
  condition it ran under.
- **AND REQ_IGNORED CANNOT BE CLEARED WHILE THAT REQUEST STANDS**, which
  is the same rule seen from the other side: the two write-to-clear flags
  live in STATE beside REQ, so acknowledging one means STORING that
  register, and 6.2.3.1's first rule is that a write to REQ with a
  power-up request pending sets REQ_IGNORED and takes no further action.
  Measured: SET at boot, still SET after a clear, and clear after the
  same store with DBG_PWRCFG.IGNORE set. So the flag is a true report of
  the condition rather than a latch a program can tidy away, and a
  program that wants a clean STATE takes the debugger out of the way
  first.

## Not covered yet

Driver gaps, each with its reason:

- **The regulator's and the detector's write sides**, and
  VREG_CTRL.UNLOCK with them: declined permanently on a bench with one
  board, for the reason the chapter gives itself. The absence is asserted
  by the family check rather than promised.
- **The power-mode-aware GPIO outputs** (6.2.3.4, EXT_CTRL0/1): two pads
  the sequencer drives on the way into and out of a low-power state, for
  an external device that must be told. No device on this desk has that
  input.
- **An external clock or tick for the timer** (12.10.5.2, 12.10.5.4,
  12.10.6): `time_reference` writes EXT_TIME_REF and nothing uses it -
  the four pads it can take would each need a generator this bench has
  not got, and the 1 Hz synchronisation would need a GPS.
- **A tick faster than a millisecond** (12.10.8), by scaling the divisor:
  the arithmetic is there in `lposc_khz` / `xosc_khz` and no program has
  wanted 62.5 us of resolution yet.
- **POW_FASTDIV and POW_DELAY**, the sequencer's own step timings: the
  reset values are what every measurement here was made against, and
  changing them changes how long a domain takes to come up. Born with the
  first program that needs a faster sequencer.
- **The boot-vector words** (5.2.3): read and never written. A magic pair
  in BOOT0/BOOT1 diverts the next boot to BOOT2's address, which is a
  bootloader's mechanism and not a driver's.
- **BOOTDIS and DBGCONFIG**: the first belongs to a secure boot chain
  this tree has none of, the second changes the SWD multidrop instance
  id and would cost the board its debug connection if it were wrong.
- **The current of each state.** NO CURRENT IS MEASURABLE ON THIS DESK:
  the board is powered through its USB-C connector with no shunt and no
  meter in the path, and the probe's own 3V3 pad is left open. This
  chapter proves which states are entered and left and what each keeps
  alive; what each draws is not measured anywhere in it.

Implemented but not bench-verified, each with what would measure it:

- **The four GPIO power-up slots as a wake from a P1.m state.** The slots
  are written, read back and refused for a pad the package has not got,
  and the wake this chapter measures is the alarm's. A pad wake out of a
  powered-down core needs an edge from outside while the core has no
  supply - a peer board's output into one of the four, which the standing
  wires of this desk do not provide (both ends of every one of them are
  this chip's own pads, and this chip is off).
- **STATE.BAD_SW_REQ and STATE.BAD_HW_REQ.** Both are decoded and neither
  has been raised: every request this driver will make is one 6.2.3's
  table allows, and it refuses the rest before the store. What would
  measure the first is a raw write of a code with the switched core up
  and the XIP domain down - harmless, since the silicon implements only
  the power-ups of an invalid request - but it wants a verb that bypasses
  the driver's guard, and that guard is what keeps the one board on this
  desk recoverable.
- **The P0.m memory states** (P0.1, P0.2, P0.3, a memory domain powered
  down with software still running). `request_state` writes them and
  nothing has: this project's linker script places the stack at the top
  of SRAM, which is SRAM1, so powering either domain down under a running
  program takes its own stack away. It wants a program whose whole image
  and stack fit in one domain.
- **`sram_stays_down_on_powerup`** (SEQ_CFG's two HW_PWRUP_SRAM bits),
  which would bring the core back up with a memory domain still off:
  measured only at its reset value, where both come back powered. It
  wants the same program as the row above.
- **The block's other three interrupt sources** - VREG_OUTPUT_LOW,
  STATE_REQ_IGNORED and PWRUP_WHILE_WAITING on `isr_powman_pow`. The
  three flags are read directly by every letter that cares; what would
  measure the line is a handler bound to it and a request made while the
  debug port's stands.
