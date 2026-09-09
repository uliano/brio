# traffic - the pedestrian-call traffic light, brio's first active objects

A breadboard learning testbed, ASSEMBLED ONCE AND DISMANTLED: four RGB
LEDs and four buttons. The three apps are kept as the record of how the
kernel's ideas were first exercised on silicon, in the order they were
learned. This directory is self-contained and nothing under docs/
references it.

- `traffic0` - the over-commented learning testbed: four buttons, four
  RGB lamps, one active object per role, `publish` for the button facts.
- `traffic1` - the traffic light itself: a multi-state FSM with timed
  phases through one re-armed time event and a remembered pedestrian
  call.
- `traffic2` - `traffic1` with PWM lamps (`TcaPwm` in split mode, a
  colour palette): the actuator changes, the active objects do not.

## The wiring it needed (AVR128DB48)

| Signal | Pins |
|--------|------|
| LED1 R/G/B, LED2 R/G/B | PB0/1/2, PB3/4/5 - common cathode, TCA1 WO0..5 (PWM in `traffic2`) |
| LED3 R/G/B, LED4 R/G/B | PC0/1/2, PC3/4/5 - common cathode, TCA0 WO0..5 |
| Buttons 0..3 | PA2..PA5 to GND, internal pull-ups |

## Building

Discovered by the avrdx project like any other app
(`experiments/*/avrdx/*.cpp`): `cmake --build --preset
avr128db48-release --target traffic2` (or `traffic0`, `traffic1`).
