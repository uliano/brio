# SPI (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 4.4 (the
ARM PL022: 4.4.2 the functional description - the prescaler, the two
FIFOs, the DMA interface -, 4.4.3 the operation - the clock ratios
of 4.4.3.4, the bit rate of 4.4.3.6.1, the frame formats of 4.4.3.7
-, 4.4.4 the registers), 2.19.2 (table 279: the pins), 2.15.3.1
(clk_peri); docs/design/spi-bus.md for the Request and the two
completion styles. The driver: `brio/rp2040/spi.hpp` (`Pl022<n>` the
resource, `SpiHost` the engine `SpiBus` drives, `SpiClient` the other
end) over `pin.hpp`, `resets.hpp`, `dma_engine.hpp` and
`util/spi_bus.hpp`. The reference suite: `test_rp2040_spi`, on the
loop-back and on four wires between the chip's two instances.

## What the silicon does

Two PL022s on clk_peri, each a host or a client in Motorola SPI, TI
or Microwire framing, frames of 4 to 16 bits, two FIFOs of eight
frames. The bit rate is clk_peri / (CPSDVSR x (1 + SCR)), the
prescaler even in 2..254 and SCR in 0..255: 62.5 Mbit/s at the top of
a 125 MHz clk_peri for a host; a CLIENT needs clk_peri at least
twelve times its clock (the input is double-synchronized), 10.4
Mbit/s here. Control registers are written with SSE clear. Four
interrupt sources on one line per instance: the receive FIFO at or
above half, the RECEIVE TIMEOUT (a frame waiting and no clock for 32
bit periods), the transmit FIFO at or below half, the receive
overrun; the last two clear by writing, the first two by moving
data. A DMA request per FIFO. LBM loops the transmitter into the
receiver with nothing on the wire. The PL022's own select output,
SSPFSSOUT, pulses BETWEEN FRAMES in Motorola format - which is why a
multi-frame transaction under one select is a GPIO's business - and
the PL022 as a client frames on SSPFSSIN. The pins are fixed per
instance under function 1: SPI0 clocks on GPIO 2, 6, 18, 22,
transmits on 3, 7, 19, 23, receives on 0, 4, 16, 20, selects on 1, 5,
17, 21; SPI1 on 10, 14, 26 / 11, 15, 27 / 8, 12, 24, 28 / 9, 13, 25,
29.

Two facts of the PL022 as a client, measured: with SPH = 0 (modes 0
and 2) the slave takes ONE frame per select window and ignores the
rest while the select stays low, so a host holding a GPIO select for
a whole transaction reaches it for more than one frame only in modes
1 and 3; and SOD does not release the transmit pad on this chip - the
line reads as a driven level, not as the host's pull-up.

## Types and verbs

- `SpiMode` (the four Motorola modes), `SpiFormat` (motorola, ti,
  microwire), `SpiClock` (`cpsdvsr`, `scr`; `divisor()`, `valid()`) with
  `SpiClocks::div2 .. div256` the named divisors the other strata's
  requests speak, `spi_sck_hz(pclk, clock)`, `spi_clock_for(pclk,
  max_hz)` (the fastest setting under a ceiling, nothing under the
  slowest), `SpiDataSize` (bits8, bits16: what the Request speaks;
  the resource takes 4..16), `SpiRole`, `SpiPins` (`sck`, `tx`, `rx`,
  `cs`, each the instance's own or none) with `spi_sck_pin` and the
  three others as table 279 and `spi_pins_valid`, `SpiConfig` (role,
  mode, format, bits, clock, loop-back, output disabled) with
  `spi_config_valid`, `spi_cr0_of` / `spi_cr1_of`, `SpiFlag`,
  `SpiInterrupt`.
- `Pl022<n>`: `reset` / `hold` / `released`, `enable(on)`, `configure`
  (refused while enabled), `loopback` and `output_disabled` live, the
  rate and width read back, the flags, `write_data` / `read_data` /
  `flush_rx`, the interrupt trio through the atomic aliases, `isr()`
  (the raised-and-enabled sources, the two clearable ones cleared),
  `dma_requests`, `dreq_tx` / `dreq_rx`, `irq()`.
- `SpiHost<n, pins, TxEngine, RxEngine>`: the engine `SpiBus`
  (= `BusMaster`) drives. Its `Request` is the other strata's: a chip
  select `PinRef` and a D/C line, a command phase (D/C low), a data
  phase with an optional out buffer (null = 0xFF dummies) and an
  optional in buffer (null = discard), the per-request `mode`, `clock`
  and `bits`, `cs_setup_us`, and `polled` - false runs the frames on
  the receive interrupt in batches of up to eight in flight, true
  spins inside `start()`. `init(clock, max_sck_hz)`, `rebase(pclk,
  sys)`, `clock_for(hz)`, `sck_hz`, `ceiling_clock`, `prime()` for a
  caller framing its own select, `loopback(on)` (kept through every
  re-application: the wireless instrument), `isr()`, `dma_isr()`,
  `status()`, `recover()`, `release()`. The engine slots take
  `dma.hpp`'s `DmaTxEngine` / `DmaRxEngine` on any two channels, both
  or neither; the data phase moves as one block each way with the
  requests raised around the engines; a 16-bit request falls back to
  the pump. Frames in a byte buffer: one byte per 8-bit frame, two
  bytes low-first per 16-bit frame.
- `SpiClient<n, pins>`: `init(clock, Config)` (mode, format, bits,
  drive_output), `enable(first)` with the first answer in the FIFO
  before the host's clock, `write()` on TNF up to `frames_ahead` = 8,
  `poll()`, `selected()` read off the select pad, `drive_output(on)`
  (the transmit pad handed over or released: the dark listener),
  `sod(on)` (the block's own bit, for a program that wants to see
  it), the overrun flag, `isr()` and the interrupt enables.
- `pin.hpp`'s `PinRef` (a pin named at run time: set, clear, toggle,
  read; `Pin<n>::ref()`) was born with this driver, as on the other
  families.

## How to use it

```cpp
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bus = brio::SpiHost<0, pins>;
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<17>;

Bus::init(clock, 8'000'000);                          // a ceiling for the whole bus
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClocks::div4;                      // 31.25 MHz at 125
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
brio::post<Spi>(r);

extern "C" void isr_spi0() {
    if (Bus::isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

With the engines: `SpiHost<0, pins, DmaTxEngine<4>, DmaRxEngine<5>>`
and `isr_dma_0` calling `Bus::dma_isr()` the same way. A client:

```cpp
constexpr brio::SpiPins client_pins{.sck = 10, .tx = 11, .rx = 8, .cs = 9};
using Client = brio::SpiClient<1, client_pins>;
Client::init(clock, {.mode = brio::SpiMode::mode1});
Client::interrupts(brio::SpiInterrupt::rx | brio::SpiInterrupt::rx_timeout, true);
Client::enable(first_answer);
extern "C" void isr_spi1() {
    (void)Client::isr();
    while (const auto f = Client::poll()) { heard(*f); }
    while (Client::writable() && have_answers()) { Client::write(next_answer()); }
}
```

## Bench findings

The reference suite is `test_rp2040_spi`, green on the Pico and the
WeAct board: five letters on the loop-back (LBM, no wire), five on
four wires between SPI0 and SPI1 of the same chip - SPI0's TX, SCK
and GPIO select into SPI1's RX, SCK and select pad, SPI1's TX back
into SPI0's RX - with the client served from its own interrupt and
the roles inverted on the same wires for the last letter.

- The rate chooser: clk_peri / 2 at the top (62.5 MHz), 8.93 MHz under
  a 10 MHz ceiling, 992 kHz under 1 MHz, nothing under 1 kHz. The
  block's reset state: both FIFOs empty, disabled; a 3-bit and a
  17-bit frame, an odd prescaler and a host with its output disabled
  are refused, a 12-bit TI configuration is taken, nothing is
  configured while enabled.
- THE LOOP-BACK ON THE PUMP: the four modes at 8 and 16 bits, sixteen
  frames each, byte-exact in three or four interrupts (the FIFOs batch
  them); a two-frame command phase then eight data frames, the echo
  discarded; a read with no out buffer clocks 0xFF; the select
  released after every transaction; an empty request completes on the
  spot.
- THE POLLED PATH at every named rate carries 64 frames byte-exact,
  from 1.6 us a frame at clk_peri / 2 (the wire alone 128 ns) to 19.6
  us at / 256 (the wire alone 16.4 us): the per-frame cost is the poll
  until the divider is the larger of the two.
- THE ENGINES on the loop: a 128-byte block at 31.25 MHz in 296 us,
  ISR-completed and exact; a command frame on the pump then 32 data
  frames on the engines; a polled request on the engines completing
  inside start() at 62.5 MHz; 16-bit frames falling back to the pump;
  a read with no out buffer through the transmit engine's fixed 0xFF
  cell.
- THE KERNEL over it, unchanged: four transactions through SpiBus
  with four replies, pumped and polled interleaved on one bus; six
  posted into a four-deep queue, one rejected at once and every
  request answered once; an idle bus votes for the sleep, a busy one
  against.
- ON THE WIRE, modes 1 and 3: sixteen frames both ways pumped at 1.95
  MHz and polled at 3.9, at 8 and at 16 bits, four frames delivered by
  the receive timeout, the client's select pad reading not-selected
  between transactions. MODES 0 AND 2: the PL022 client takes ONE
  frame per select window (sixteen clocked, one heard), the fact a
  multi-frame transaction under a held select must know.
- THE LADDER on the wire, the host polled: exact both ways at 15.6
  MHz (clk_peri / 8) and below, wrong at 31.25 and 62.5 - the client's
  ceiling is clk_peri / 12.
- THE DARK LISTENER: a client with its transmit pad released reads as
  the host's pull-up (0xFF) and still hears every frame; SOD set with
  the pad on the peripheral reads 0x80 repeated - the line is driven,
  not released - and the transmit FIFO is not consumed under it (the
  answers loaded before it are still there after sixteen frames). A
  client nobody reads overruns its FIFO, says so, and the flag clears
  by writing one.
- THE ENGINES ON THE WIRE at 1.95 MHz, 64 bytes each way, the client
  on its interrupt: both directions exact. THE ROLES INVERT on the
  same four wires: SPI1 hosting with its select on GP9, SPI0 listening
  on GP17, both directions exact.
- ONE CORE SERVING BOTH ENDS is the suite's arrangement and its
  limit: a pumped host at 3.9 MHz fills its next four frames before
  the client's interrupt has refilled its answers (the client's FIFO
  runs dry at the ninth frame and overruns), and the first interrupt
  of a cold client pays the cache fill - so the pumped and engined
  wire rounds run at 1.95 MHz after one warming round. Two cores, or
  two boards, would not compete.

## Not covered yet

Driver gaps, each with its reason:

- The TI and Microwire framings on the wire: configured and read back
  in the resource, no device on the desk speaks them.
- Frame widths other than 8 and 16 in the Request: the other strata's
  Request speaks two, the resource takes 4..16; a device with a
  12-bit frame is the first user.
- The client on the DMA engines: the suite's client answers from its
  interrupt; a client that streams is born with a program that needs
  one.
- A client in modes 0 and 2 beyond one frame per select: the PL022
  needs the select pulsed per frame, which a host's own SSPFSSOUT
  does and a GPIO select does not; a program that wants it uses the
  peripheral's select on the host side.

Implemented but not bench-verified, each with what would measure it:

- The per-bus timeout with `recover()`: a lost completion staged as
  the other targets' suites stage it (the ISR body run, the reply
  never posted), then four transactions to spi_ok on the same bus AO.
- `cs_setup_us`: a device that needs it, or a scope on the select and
  the first clock.
