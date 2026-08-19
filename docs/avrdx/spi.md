# SPI - the serial peripheral interface engine (AVR DA/DB)

> **PROVISIONAL.** Not the result of a systematic review of the SPI
> chapter and errata; the driver is the host-mode engine the bus AO
> needed for the display, the touch controller and the MCP3550. The
> exhaustive pass is pending. Documents consulted: AVR128DB28/32/48/64
> data sheet DS40002247B (SPI, electricals 39.15), errata DS80000915F
> (2.11.1: SPI1 ALT2 pin position non-functional on 48-pin, rev
> A4/A5 - not used). Driver: `avrdx/spi.hpp`; the arbiter above it:
> [spi-bus.md](../design/spi-bus.md) (`util/bus_master.hpp`, `util/spi_bus.hpp`).
> Reference tests: `spi_loopback` (PA4 -> PA5 jumper), `spi_duo`,
> `spi_paint`, `dac_adc` (MCP3550).

## What the driver does today

`Spi<n>`: host mode, MSB first, default pins (SPI0 PA4/PA5/PA6, SPI1
PC0/PC1/PC2), SSD (no client-select input). One transaction = a
`Request` descriptor: CS and DC as `PinRef`, a command phase (DC low)
and a data phase (DC high, full duplex: tx and rx spans), per-request
clock (`SpiClock` div4/16/64/128 of CLK_PER) and mode (CPOL/CPHA),
`cs_setup_us` before the first SCK (MCP3550), `park_sck` level, and a
completion style: per-byte ISR pump (`isr()` body) or a polled loop
for bulk at fast clocks. CS is asserted and released by the engine
around the whole transaction. A `ClockUser`: `rebase` re-derives the
cs_setup timing base (the SCK prescalers are relative to CLK_PER:
the client's choice may need revisiting after a clock change).

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `Spi<n>` | `init(clock)`, `rebase(hz)`, `start(const Request&)` -> bool (true = completed synchronously), `isr()` -> bool (done; the app posts TransferDone), `status()` |
| `Spi<n>::Request` | `cs`, `dc` (PinRef), `cmd`/`cmd_len`, `tx`, `rx`, `len`, `reply` (ReplyTo<SpiDone>), `clock`, `mode`, `polled`, `cs_setup_us`, `park_sck` |
| `SpiClock` | `div4` (6 MHz at 24 MHz) .. `div128` |

## How to use it

Through the bus AO, never directly from an app: `post<SpiBus>(request)`
with `reply_to<Me, SpiDone>()` - see [spi-bus.md](../design/spi-bus.md). The
engine itself is bound once:
```cpp
using SpiHw = brio::Spi<0>;
ISR(SPI0_INT_vect) { if (SpiHw::isr()) brio::post<Bus>(brio::TransferDone{SpiHw::status()}); }
SpiHw::init(clock);
```

## Bench findings

- Per-byte ISR pump costs too much at 6 MHz for 960-byte rows: the
  polled completion style exists for that (the DMA's slot on other
  targets).
- MCP3550: needs `cs_setup_us` and the SPI mode latched at CS fall
  (mode 1,1 = SCK parked high): `park_sck`; a CPOL change is applied
  with the peripheral disabled.

## Not covered yet

Client mode, LSB first, buffer mode (BUFEN, BUFWR), the remaining
prescalers and double speed (CLK2X), write-collision flag, the SS
input (SSD off), wake-up from idle, the pin routing alternatives
(PORTMUX ALT1/ALT2 beyond the default), the electricals' timing table,
DMA-shaped engine boundary on targets that have it.
