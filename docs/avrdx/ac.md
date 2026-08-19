# AC - the analog comparators (AVR DA/DB)

> **PROVISIONAL.** The systematic review of the chapter and errata is
> done (this page records it); the driver and its bench suite are not
> written yet. Documents of record: AVR128DB28/32/48/64 data sheet
> DS40002247B (AC chapter 32, electricals 39.17, PORTMUX 17.3.10, EVSYS
> 16 generators 0x20-0x22, I/O multiplexing chapter 3), errata
> DS80000915F (no AC item). Complements: TB3211 "Getting Started with
> AC" (see [vendor/README.md](vendor/README.md)). Driver: none yet;
> the reference is `Vref::ac` in `avrdx/vref.hpp` ([vref.md](vref.md)).
> Reference test: none yet.

## What the silicon does

Three comparators, AC0..AC2. Each compares a positive input (one of
four analog pins) with a negative input (one of three pins or
`DACREF`, an 8-bit divider of the comparators' shared reference:
V = DACREF x VREF / 256, VREF = `Vref::ac`), with selectable
hysteresis (none / 10 / 25 / 50 mV typ.) and three power profiles
(response 85 / 250 / 460 ns typ. rising at VDD/2; 17 uA at profile 1).
The output is a level: `STATUS.CMPSTATE`, an event generator (0x20 +
n, async level), a pin (`OUTEN`: PA7 or ALT1 PC6 for all three - the
PA7 default collides with CLKOUT), the CCL input menu (ACn OUT on LUT
input n, [ccl.md](ccl.md)) and one interrupt per instance (`CMP`,
sense both / negative / positive edge). `INVERT` flips the output;
`INITVAL` sets it during start-up (the AC, and VREF if internal,
take their start-up time after enable; a change of input pin or
reference wants settling time too). `RUNSTDBY` keeps it alive in
standby (event, interrupt and pin keep working without CLK_PER).

Input pins on the 48-pin part (chapter 3):

| | AINP0 | AINP1 | AINP2 | AINP3 | AINN0 | AINN1 | AINN2 |
|---|---|---|---|---|---|---|---|
| AC0 | PD2 | PE0 | PE2 | PD6 | PD3 | PD0 | PD7 |
| AC1 | PD2 | PD3 | PD4 | PD6 | PD5 | PD0 | PD7 |
| AC2 | PD2 | PD4 | PE1 | PD6 | PD7 | PD0 | PD7 |

PD6 (AINP3 on all three) is the DAC output pin, so the DAC can drive
any comparator from outside; the pins must be set as analog inputs
(input buffer off) in PORT.

**Window mode**: `CTRLB.WINSEL` pairs ACn with ACn+1 or ACn+2 (the
upper limit on the partner, the lower on ACn, both `MUXPOS` on the
SAME pin - the user configures both MUXCTRLs); `STATUS.WINSTATE`
reports above / inside / below and `INTMODE` (above / inside / below
/ outside) selects which state raises the interrupt and the event;
`CMPSTATE` is then "the window state matches INTMODE".

Errata F has no AC item. Electricals: offset +-5 mV typ. (15 max),
input leakage 5 nA, CMRR 70 dB, input range -0.2 V .. VDD.

## Types and verbs

None yet. The intended shape: a resource `Ac<n>` (config: positive
and negative inputs as typed pins or `dacref`, hysteresis, power
profile, invert, initial value, output pin, run-standby; `threshold_mv`
helper over `ref_mv` for DACREF; `state()`, `cmp()` ISR body with the
sense mode, the OUT event generator) and a `Window<n, m>` over two;
tasks: a threshold detector that posts a kernel event on crossing,
a window watcher - the analog counterpart of the ADC window watcher
named in [design/analog.md](../design/analog.md).

## How to use it

Nothing to show yet.

## Bench findings

None yet.

## Not covered yet

Everything above; the suite (`test_avr_timer` or a `test_avr_analog`
extension: DAC on PD6 against DACREF thresholds through AC0 AINP3,
hysteresis widths measured by sweeping the DAC, response time by a
TCB capturing the AC event against a DAC step, window mode with two
comparators on PD6); the EVSYS vocabulary for AC.
