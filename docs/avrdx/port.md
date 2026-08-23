# PORT - the I/O pins (AVR DA/DB)

> **PROVISIONAL.** The pin and port surface is covered in full and
> bench-verified (senses, flags, the multi-pin engine, the one-store
> configuration); what remains is what this bench cannot measure -
> the INLVL thresholds, the slew rate's effect, the fully-async wake
> - and the pin-level bonding deferral. The gaps are in "Not covered
> yet".

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B
(PORT chapter 18, I/O multiplexing chapter 3), errata DS80000915F
(2.9.1 PD0 floating input on 28/32-pin; 2.2.4 VPORT stores after a
store to >= 64 - Pin uses SBI/CBI, unaffected). Driver:
`avrdx/pin.hpp`. Reference test: `test_avr_pin`.

## What the silicon does

Up to seven ports of up to eight pins, each pin with: direction and
output value (atomic SET/CLR/TGL registers, plus the VPORT aliases in
bit-accessible I/O space - SBI/CBI, single cycle), a synchronized
digital input (`IN`), and a `PINnCTRL` byte holding the pin's whole
personality: `INVEN` (inverts input AND output), `PULLUPEN` (only
while the pin is an input), `INLVL` (DB only: Schmitt-from-supply or
TTL input threshold - an MVIO companion), and `ISC` - the input/sense
configuration that is BOTH the interrupt sense (both edges / rising /
falling / low level) AND the input buffer switch (`INPUT_DISABLE`:
`IN` freezes, interrupts and events die, the pad keeps feeding the
analog mux - what an ADC/AC input wants: no mid-rail current in the
Schmitt stage, no digital noise).

Facts that matter to code:

- **One interrupt vector per port** (`PORTx_PORT_vect`): any sensed
  pin raises its bit in `PORTx.INTFLAGS`; several pins can fire
  together, so the ISR reads the MASK. The flags are
  write-1-to-clear: a plain store - an RMW (or SBI, which is one)
  reads every pending flag back as 1 and clears the lot. Measured:
  the clear lands one cycle after the store - a back-to-back read
  still sees the old flags.
- **The sense fires on the SYNCHRONIZED input**: an OUTPUT pin whose
  buffer stays on senses its own edges (how the test suite closes its
  loops); `level_low` re-fires for as long as the pin reads low
  (measured: continuously - it is a level, not an event); edges less
  than ~3 CLK_PER apart are lost on partially-async pins (the
  interrupt dead-time).
- **Px2 and Px6 are FULLY asynchronous** (I/O mux note): they wake
  from every sleep mode on every sense, catch sub-CLK_PER pulses and
  have no dead-time; every other pin needs CLK_PER running (or
  BOTHEDGES/LEVEL held until wake-up completes).
- **INVEN inverts the sense too** (measured: rising + INVEN counts
  the physical falling edges), and toggling INVEN makes an edge every
  peripheral on the pin sees. The datasheet's three write-hazards
  (18.3.3): INVEN toggled in the same cycle as an ISC change loses
  that edge's interrupt; INLVL must change with the pin's consumers
  quiet; an ISC change while an interrupt is synchronizing can fire a
  spurious one or lose one. The driver's answer is the one-store
  `configure()`.
- **The multi-pin engine** (18.3.2.4): `PINCONFIG` holds a PINnCTRL
  value and is MIRRORED across ports; `PINCTRLUPD/SET/CLR` (per
  port) copy/set/clear it into every pin of a mask in one operation.
- `PORTCTRL.SRL` limits the slew rate of the whole port. The PORT
  keeps running while the CPU is halted in debug.
- Errata 2.9.1: on 28/32-pin DB parts PD0 is not bonded but its input
  buffer exists and floats - constant current draw; the datasheet's
  remedy is INPUT_DISABLE on PD0. A board/device-init fact (one
  store), noted here on purpose and NOT wrapped by the framework: a
  kludge for broken silicon stays visible.

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `PinConfig` | `invert`, `pullup`, `input_level` (`PinLevel::schmitt/ttl`, DB only - the field exists only where the silicon has it), `sense` (`PinSense`: none, both, rising, falling, input_disable, level_low) |
| `Pin<'A', 5>` | `output`/`input`, `set`/`clear`/`toggle`/`read`/`is_output` (VPORT, single cycle), `configure(cfg)` (the whole PINnCTRL in ONE store), the single-field RMW verbs `invert`/`pullup`/`sense`, `flag`/`clear_flag` (this pin's W1C bit), `pinctrl()`, `ref()` (runtime PinRef), `fully_async` (Px2/Px6), PwmChannel (max 1), `disable_digital_input`/`enable_digital_input` |
| `Port<'C'>` | the port's own registers: `in`/`dir_set`/`dir_clear`/`out_set`/`out_clear`/`out_toggle` (masks), `flags`/`clear_flags(mask)`, `take_flags` (ISR body of PORTx_PORT_vect: the fired mask, cleared), `slew_limit(bool)`/`slew_limit()`, `configure_mask(pins, cfg)` (the multi-pin engine), `regs()`/`vregs()` |
| `PinSet<Pins...>` | up to 8 pins on ANY ports as one bit mask: `input(pullup)`, `output`, `read`, `write`, `configure(cfg)` (grouped BY PORT at compile time: one mirrored PINCONFIG store plus one PINCTRLUPD per involved port - the set behaves like a single register), `port_mask<'C'>()` |
| `PinRef` | the runtime descriptor (3 bytes): a pin inside a request event; null = no-op |

## How to use it

A button with an interrupt (the classic):

```cpp
using Button = brio::Pin<'A', 2>;                 // PA2: fully async - wakes from standby
Button::input();
Button::configure({.pullup = true, .sense = brio::PinSense::falling});
ISR(PORTA_PORT_vect) {
    const uint8_t who = brio::Port<'A'>::take_flags();
    if (who & Button::mask) post<Ui>(Pressed{});
}
```

A row of keys on two ports, configured as one register:

```cpp
using Keys = brio::PinSet<brio::Pin<'A', 2>, brio::Pin<'A', 3>,
                          brio::Pin<'C', 6>, brio::Pin<'C', 7>>;
Keys::input();
Keys::configure({.pullup = true, .sense = brio::PinSense::both});
const uint8_t raw = ~Keys::read() & Keys::mask;   // active-low
```

An analog pin done right (what the ADC/AC drivers do for you):

```cpp
brio::Pin<'D', 1>::configure({.sense = brio::PinSense::input_disable});
```

## Bench findings

`test_avr_pin` (A5, 22 verdicts): every sense counts exactly (10
rising, 10 falling, 20 both, 0 with none, on self-driven edges 2 us
apart); `level_low` is quiet while high, re-fires continuously while
low (245 ISRs in a ~20 us window) and stops on the rising edge;
rising + INVEN counts the physical falling edges; the W1C discipline
holds (clearing one pin's flag leaves the other's) and the clear
lands ONE CYCLE after the store; a pulled-up input reads 1;
`input_disable` freezes IN at its last value and the buffer comes
back live; the multi-pin engine writes one setting into three pins on
two ports (PINnCTRL readbacks exact) and both ports interrupt; the
Port mask verbs and the slew-limit bit read back as written.

## Not covered yet

Driver gaps:

- Pin-level bonding within an existing port (which Pxn THIS package
  bonds) - deferred to the family device tables, as everywhere.

Implemented but not bench-verified:

- The INLVL thresholds (needs analog levels) and the slew rate's
  electrical effect (needs a scope) - both are configuration
  read-back only today.
- The fully-async wake of Px2/Px6 from standby (queued with the
  RUNSTDBY pass, LOW priority).
- The PA2..PA5 buttons as a human-in-the-loop extra.
