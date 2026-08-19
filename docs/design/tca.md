# TCA - the 16-bit timer/counter type A (AVR DA/DB)

> **PROVISIONAL.** Not the result of a systematic review of the TCA
> chapter and errata; the driver is one task on the peripheral - six
> 8-bit PWM channels in split mode. The exhaustive pass (with the TCB
> tasks, driven by the Multislope needs) is pending. Documents
> consulted: AVR128DB28/32/48/64 data sheet DS40002247B (TCA, PORTMUX
> TCAROUTEA), errata DS80000915F (2.12.1 restart resets the count
> direction in NORMAL/FRQ mode, rev A4/A5 - not used). Driver:
> `avrdx/pwm.hpp` (`TcaPwm`). Reference tests: `traffic2` (twelve PWM
> channels on TCA0/PORTC and TCA1/PORTB).

## What the driver does today

`TcaPwm<n, port>`: TCA n in SPLIT mode, both 8-bit halves in single-
slope PWM with PER = 255, routed to one port (PORTMUX: TCA0 to PORTA/
B/C/D/F, TCA1 to PORTB for six channels), pins 0..5 = WO0..5, one
shared prescaler (`TcaClock`, div16 -> ~5.9 kHz at 24 MHz). `duty<ch>
(v)` is one store; 0 and 255 leave the waveform and drive the pin from
PORT.OUT (clean endpoints, DxCore's policy). `Channel<ch>` is the
`PwmChannel` type generic actuators use (`RgbLamp`). The task owns the
whole timer: no other use of that TCA instance coexists.

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `TcaPwm<n, port>` | `init(TcaClock)`, `duty<ch>(uint8_t)`, `Channel<ch>` (PwmChannel: `max = 255`, `duty(v)`); `channels = 6` |
| `TcaClock` | `div1 .. div1024` |

## How to use it

```cpp
using PwmC = brio::TcaPwm<0, 'C'>;           // TCA0 -> PC0..PC5
PwmC::init();                                // split mode, div16, all dark
PwmC::duty<2>(64);                           // PC2 at 25 %
using Lamp3 = brio::RgbLamp<PwmC::Channel<0>, PwmC::Channel<1>, PwmC::Channel<2>>;
Lamp3::show({255, 40, 0});
```
Clock change: the PWM frequency scales with CLK_PER (not a ClockUser
today - a fact to decide in the exhaustive pass: keep the frequency, or
the prescaler?).

## Bench findings

- Twelve channels on two timers drive four RGB LEDs; colour mixing is
  limited by the LEDs' dies, not by the PWM.

## Not covered yet

Normal 16-bit mode (period and three compare channels, double
buffering), frequency generation, single- and dual-slope 16-bit PWM,
count on event / restart on event / direction from event (`TCAn CNTA/
CNTB` users), the overflow and compare events and interrupts as
generators (`TCAn OVF/CMPn` codes exist in the EVSYS table), the
command register (restart, reset, update), split-mode interrupts,
the 6-channel routes that are partial on some ports, the other
PORTMUX routes for TCA1, RUNSTDBY, errata 2.12.1, the ClockUser
question above. The `Tca<n>` resource handle + task split (a
`TcaHeartbeat` with two waveform outputs and an overflow event is the
Multislope need) is the design step that comes with the exhaustive
pass.
