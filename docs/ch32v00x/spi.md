# SPI (CH32V00x)

The one SPI of RM ch. 16 - both roles, 8- or 16-bit frames, the four
modes, a software or hardware chip select, hardware CRC, two DMA
requests - and the host engine util/spi_bus.hpp's arbiter drives on
this silicon as on the other three. Documents of record: the CH32V00X
reference manual V1.5 (16.2.1 for the pins and the select, 16.2.2 and
16.2.3 for the two roles, 16.2.5 for the CRC, 16.2.7 for the errors,
16.3 for the registers), the CH32V006 datasheet V2.0 (table 2-1-1 for
the pads).

## What the silicon does

- **The STM32F1's SPI**, with the I2S half absent and one register of
  its own: HSCR, a "high-speed read" mode whose SCK the manual gives
  as HCLK/(BR + 2).
- **No FIFO.** One transmit buffer, one receive buffer, one shift
  register (figure 16-1): TXE means the buffer moved into the shifter,
  RXNE that a frame has been shifted both ways.
- **Frames of 8 or 16 bits** (DFF), SCK from HCLK/2 to HCLK/256 (BR),
  MSB or LSB first, the four CPOL/CPHA modes of table 16-1.
- **The chip select has three arrangements** (16.2.1): software (SSM,
  the level from SSI, the pad free), hardware output (SSOE, driven
  low while SPE is set), hardware input (the multi-host arrangement:
  a low level raises MODF and demotes the host).
- **NOTHING IS ENABLE-PROTECTED** (measured): 16.3.1 says DFF and CRCEN
  "can only be written when SPE is 0" and that BR, CPOL, CPHA, MSTR
  and LSBFIRST "cannot be modified during communication" - and a raw
  write to each of the seven lands with SPE set. The driver's
  refusals and its disable/enable pair are the only protection there
  is.
- **The DMA requests are channels 2 (receive) and 3 (transmit)**,
  table 8-2 - the channel is the request on this family.
- **The default pads on the CH32V006** are SCK PC5, MOSI PC6, MISO PC7,
  NSS PC1 (table 2-1-1); the remaps are AFIO's, which the stratum does
  not touch yet.
- **Two of the chapter's sentences are slips**: DFF's two codes are
  both described as "16-bit" (0 is 8-bit), and 16.2.1 describes CPHA
  with the same words for both values (table 16-1 and the figures
  settle it).

## Types and verbs

[brio/ch32v00x/spi.hpp](../../brio/ch32v00x/spi.hpp), three layers:

- `Spi<1>` is the resource: `configure(SpiConfig)` (refused by
  `spi_config_valid()`: CRC outside full duplex or with a zero
  polynomial, SSOE on a client), `enable()`, `disable()` - the drain
  procedure, TXE then not BSY, bounded -, `data()` both ways,
  `rxne()`/`txe()`/`busy()`, `flush_rx()`, the three errors with their
  clearing sequences (OVR: DATAR then STATR; MODF: STATR then a CTLR1
  write), the CRC verbs (`crc_next()`, `tx_crc()`, `rx_crc()`),
  `software_select()`, `high_speed_read()`, `dma_requests()`, the
  three interrupt enables and `isr()`, the raised-and-enabled sources.
- `SpiHost<1, pins, TxEngine, RxEngine>` is the engine `SpiBus` (=
  `BusMaster`) drives. Its `Request` is the other strata's verbatim:
  a chip select `PinRef` and a D/C line, a command phase (D/C low), a
  data phase with an optional out buffer (null = 0xFF dummies) and an
  optional in buffer (null = discard), the per-request `mode`, `clock`
  and `bits`, and `polled` - false runs the frames on the RXNE
  interrupt with the kernel free between them, true spins inside
  `start()` and completes synchronously. `init(clock, max_sck_hz)`,
  `rebase()`, `clock_for(hz)`, `prime()` for a caller framing its own
  select, `bit_order()`, `isr()`, `dma_isr()`, `status()`,
  `recover()`, `release()`, `claim_nss_pad()`. The engine slots are
  `DmaTxEngine<3>` and `DmaRxEngine<2>`, both or neither, on those
  two channels and no others (refused at compile time otherwise); a
  16-bit request falls back to the pump. Frames in a byte buffer: one
  byte per 8-bit frame, two bytes low-first per 16-bit frame.
- `SpiClient<1, pins>`: the target side, thin - `init(clock, Config)`,
  `enable(first)` with the first answer loaded before the host's
  clock, `write()` on TXE (ONE frame ahead: no FIFO), `poll()`,
  `selected()` read off the NSS pad, `drive_output()` for a dark
  listener, `isr()`.

[brio/ch32v00x/pin.hpp](../../brio/ch32v00x/pin.hpp)'s `PinRef` (the
runtime pin the request carries) and `Pad` (the compile-time pad name
the pin tables use) were born with this driver.

## How to use it

```cpp
using Bus = brio::SpiHost<1>;                         // the default pads
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<'C', 3>;

Bus::init(clock, 8'000'000);                          // a ceiling for the whole bus
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClock::div4;                       // 12 MHz at 48
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
brio::post<Spi>(r);

extern "C" BRIO_CH32_INTERRUPT void spi1_handler() {
    if (Bus::isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

With the engines: `SpiHost<1, spi1_default_pins, DmaTxEngine<3>,
DmaRxEngine<2>>`, and `dma1_channel2_handler` / `dma1_channel3_handler`
both calling `Bus::dma_isr()` the same way.

## Bench findings

The reference suite is `test_ch32_spi` on the CH32V006K8U6 at 48 MHz:
its wire letters run on ONE JUMPER, MOSI (PC6) to MISO (PC7), and
decline without it. What the desk has measured so far is its wireless
letter:

- **The reset values are table 16-2's** (STATR 0x0002 with TXE up,
  CRCR 0x0007), and a control word lands as spelled.
- **Seven of seven CTLR1 fields take a write under SPE** - DFF, CRCEN,
  CPOL, CPHA, LSBFIRST, BR and MSTR - so the enable protection the
  chapter describes does not exist in the silicon.
- **Every path completes with nothing on MISO**: the ISR pump (sixteen
  interrupts for sixteen frames), the polled path, the DMA engines
  ISR-completed and polled, each transaction ending and releasing the
  select.

## Not covered yet

Driver gaps, each with its reason:

- The simplex modes (BIDIMODE/BIDIOE, RXONLY) beyond the resource's
  configuration bits: no user.
- The hardware select arrangements as the ENGINE's select: the engine's
  select is a GPIO on purpose (docs/design/spi-bus.md); the resource
  has SSOE and the multi-host input for a program that wants them.
- The CRC as part of a Request: the resource's verbs exist, born into a
  tenure shape with a device that checks one.
- A client against a foreign host: the peer protocol (the other
  strata's spi_link) on a second board.

Implemented but not bench-verified, each with what would measure it:

- **The loopback letters**: the four modes at both widths through the
  pump, every BR code on the polled path timed, the engines with and
  without a command phase, the CRC against a bitwise reference, the
  arbiter's replies and votes - the suite's letters b..f on the one
  jumper.
- HSCR's high-speed read mode: its rate formula against a scope on
  SCK.
