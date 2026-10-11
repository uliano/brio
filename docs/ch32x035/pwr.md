# PWR (CH32X035)

The power controller of the CH32X035 series, as much of it as a program
asks today: the programmable voltage detector, the one witness a board
has of its own supply. The sleep modes and the flash's low-power bits
are mapped in [device.hpp](../../brio/ch32x035/device.hpp) and driven by
nothing ([README.md](README.md)'s gap list).

Documents of record: the CH32X035 reference manual V1.8 (2.2.2 for the
detector, figure 2-3 for its hysteresis, 2.4.1 and 2.4.2 for PWR_CTLR and
PWR_CSR) and the CH32X035/X033 datasheet V1.7 (the electrical
characteristics the manual points to for the thresholds). The header is
[brio/ch32x035/pwr.hpp](../../brio/ch32x035/pwr.hpp).

## What the silicon does

- **Four thresholds, chosen by PLS[1:0]** (2.4.1), each with a rising and
  a falling edge about 20 mV apart (figure 2-3): 2.12 / 2.1 V, 2.32 /
  2.3 V, 3.02 / 3.0 V and 4.02 / 4.0 V.
- **PVD0 says which side VDD stands on** (2.4.2): 1 below the chosen
  threshold, 0 above it. The detector also feeds EXTI line 26, for an
  interrupt on either crossing (2.2.2).
- **There is no enable bit.** PWR_CTLR's bits 4:2 are reserved where the
  STM32F1 keeps PVDE: the manual's three steps are "set the PLS field",
  optionally the EXTI line, and "read the PVD0 bit".
- **The registers answer behind RCC_APB1PCENR's PWREN**, and the
  controller's reset line is one no verb of this stratum pulses
  ([clock.md](clock.md): a pulse on it cut the part off its debug port).

## Types and verbs

| name | what it is |
|---|---|
| `PvdLevel` | PLS's four codes, named by the falling edge: `v2_1`, `v2_3`, `v3_0`, `v4_0` |
| `pvd_rising_mv(level)`, `pvd_falling_mv(level)` | the two edges of a threshold, in millivolts (2.4.1) |
| `Pwr::pvd(level)`, `Pwr::pvd()` | PLS written, every other bit of PWR_CTLR kept; and read back |
| `Pwr::supply_low()` | PVD0 |
| `Pwr::open()` | PWREN, which every verb opens itself (idempotent) |

## How to use it

The check an image makes at boot before it drives a pad wired to another
board, whose protection diodes a sagging rail would otherwise be fed
through:

```cpp
brio::Pwr::pvd(brio::PvdLevel::v3_0);
(void)brio::delay_us(clock, 200);              // the manual gives no settling time
const bool supply_ok = !brio::Pwr::supply_low();
```

## Bench findings

On the CH32X035F8U6 of WCH's evaluation board, fed from its WCH-LinkE's
3.3 V: at the boot of `test_x035_usart` and `test_x035_pin` - each drives
pads wired to the desk's CH32V006K8U6 - the detector at 3.0 V reads NOT
LOW, and both suites run their letters; each says so on its banner.

## Not covered yet

Driver gaps, each with its reason:

- **The detector's interrupt** (EXTI line 26): born with a program that
  acts on a crossing while it runs; the boot check reads the flag.
- **The low-power modes** (PDDS, the flash's LP and LP_REG bits, the
  auto-wakeup): the sleep chapter, born with its first sleep site
  ([README.md](README.md)).

Implemented but not bench-verified, each with what would measure it:

- **A threshold crossed**: only the 3.0 V level has been read, and only
  above it; a bench supply stepped through 3.0 V, or the four levels
  read at a known VDD (the 4.0 V one reading low on a 3.3 V rail), would
  measure the flag's sense and the edges.
