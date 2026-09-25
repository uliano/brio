# SPI and I2S - one block, two faces (STM32F4)

Documents of record: RM0090 Rev 22 ch. 28 (RM0390 Rev 6 ch. 26 and
RM0383 Rev 4 ch. 20 are its twins: one register description, three
manuals, and the memory maps differ on which instances wear the audio
face), the datasheets' alternate-function tables for the pads, RM0090
tables 43 and 44 with their RM0390 and RM0383 twins for the DMA request
mapping, and the errata's SPI/I2S items - ES0206 2.12, ES0287 2.11,
ES0298 2.14: "BSY bit may stay high when SPI is disabled", "anticipated
communication upon SPI transit from slave receiver to master", "wrong
CRC calculation when the polynomial is even", "corrupted last bit of
data and/or CRC received in Master mode with delayed SCK feedback",
"BSY flag may stay high at the end of a data transfer in Slave mode".
Driver: `stm32f4/spi.hpp` (`Spi<n>` the resource, `I2s<n>` and
`I2sExt<n>` the audio face, `SpiHost<n, pins, TxEngine, RxEngine>` the
bus engine, `SpiClient<n, pins>` the other end, the vocabulary
`SpiConfig`, `SpiMode`, `SpiClock`, `SpiDataSize`, `SpiNss`,
`SpiDirection`, `SpiFrameFormat`, `SpiFlag`, `SpiPins`, `I2sConfig`,
`I2sPins`, and the two arithmetics), over the reserve's instance,
audio and request-mapping facts (`stm32f4/device_tables.hpp`), with
the audio PLL and the I2S clock selector on `Rcc`
([clock.md](clock.md)) and the streams from [dma.md](dma.md). The
family fixture is `test/family_stm32f4/spi.cpp` with the negatives
that refuse an absent instance, one pad twice, a host with no MOSI, a
pad on an absent port, an engine off the request map, an engine on a
part class whose manual was not read, one engine of two, two engines
on one stream, the audio face on an instance that has none, on a part
class that is not known, and an extension block that does not exist.
Bench: `test_stm32f4_spi` on the STM32F429I-DISC1, against the
gyroscope and the display controller the board carries on SPI5.

## What the silicon does

**The F1 lineage, as the USART of this family is.** CR1/CR2/SR/DR/
CRCPR/RXCRCR/TXCRCR/I2SCFGR/I2SPR, no FIFO and no frame size beyond
eight or sixteen bits: one transmit buffer, one receive buffer, one
shift register. TXE means "the buffer moved into the shifter" and RXNE
"a frame has been shifted BOTH ways" - which is why a host's pump runs
on RXNE and never on TXE: a frame that came back is the one witness
that the bus is idle for the next.

**Up to six instances, one vector each, and the bus decides the rate.**
SPI1 on every part; SPI2 on every part but the 36-pin F410Tx; SPI3 on
every part but the F410 line; SPI4 on the F401, F411, F412, F413,
F42x/F43x, F446 and F469; SPI5 on the F410's larger packages, the F411,
F412, F413, F42x/F43x and F469; SPI6 on the F42x/F43x and F469 classes
alone. SPI1, SPI4, SPI5 and SPI6 sit
on APB2 and SPI2 and SPI3 on APB1, so BR[2:0] divides ITS bus's clock
and not SYSCLK - 90 against 45 MHz at 180 MHz of core
([clock.md](clock.md)). Every instance has a vector of its own, shared
with nothing.

**Enable-protected and not.** DFF and CRCEN "should be written only
when SPE is 0" (28.5.1), and BR, CPOL, CPHA, LSBFIRST and MSTR "should
not be changed when communication is ongoing" - so a request that
changes the frame size, the mode or the rate costs a disable/enable
pair, and one that changes nothing costs nothing. BIDIOE is the
exception the chapter allows while the block runs, which is what makes
a three-wire turnaround possible at all.

**Three chip-select arrangements** (28.3.1). SSM with SSI is the
software one, and it leaves the NSS pad free - which is what a bus AO's
GPIO select needs; a host under it must hold SSI HIGH, because a low
SSI on a master IS the mode fault. SSOE drives the pad low while SPE is
set and gives up multi-master. The hardware input is the multi-master
arrangement, where another master pulling NSS low demotes this one.

**The flags are cleared by sequences, not by a clear register**
(28.3.10): the overrun by reading DR and THEN SR - the DR read alone
does not do it; the mode fault by an SR access while MODF stands and
then a CR1 write; the CRC error by writing a zero into it, the one
rc_w0 bit of the register; the TI framing error and the I2S
desynchronization by reading SR; the I2S underrun by reading SR.

**Four errata are live on every part with a manual on the desk**, and
three of them are answered as code:
- *BSY may stay high when the SPI is disabled* (2.12.1). The disable
  procedure waits for TXE and then for BSY in the transmitting
  configurations, and in MASTER RECEIVE-ONLY - RXONLY, or bidirectional
  with the output off - it does not look at BSY at all: the errata's own
  instruction, and 28.3.7 already says the flag is kept low there.
- *Anticipated communication upon a transit from slave receiver to
  master* (2.12.2): the clock starts on the MSTR write even with SPE
  clear. `configure()` pulses the RCC reset line before writing a HOST
  configuration over a client receive-only one - the errata's first
  workaround, free on the ordinary path.
- *A wrong CRC when the polynomial is even* (2.12.3): an even
  polynomial is refused, so the unit is never armed with a divisor that
  computes nonsense. The reset value 0x0007 is odd.
- *The last received bit may be wrong when the SCK feedback is slow*
  (2.12.4). The internal loop back from the SCK PAD must return inside
  two APB periods, and the errata's table gives the ceiling per pad
  class at a 30 pF load: 84 MHz of APB at `high` or `very_high`, 75 at
  `medium`, 25 at `low` - halved for I2S. It is a PAD fact and not a
  rate fact: the SPI's own bit rate does not enter it. Every pad this
  driver hands over goes out at `very_high`.

**The audio face is one bit of I2SCFGR**, and which instances have it
is the manual's memory map and not the header - every SPI_TypeDef of
the pack declares I2SCFGR and I2SPR, on instances the manual gives no
I2S at all. It is SPI2 and SPI3 on the F405 class, the F42x/F43x class
and the F446; every instance from SPI1 to SPI5 on the F411. Full duplex
is a SECOND BLOCK on the classes that have one (I2S2ext, I2S3ext -
always slaves, on the instance's own clock and word select) and two
whole instances paired on the F446, which has none.

**The I2S kernel clock is not the APB's**: the audio PLL's R output, an
external clock on the I2S_CKIN pad, and on the parts with the
RCC_DCKCFGR pair the main PLL's R or the root itself. THE PAIR IS
NUMBERED BY BUS AND NOT BY INSTANCE - I2S1SRC is the APB1 instances'
and I2S2SRC the APB2 ones' (RM0390 6.3.24) - a trap the names invite.
On the F42x/F43x class the audio PLL has no input divider of its own
and shares the MAIN PLL's M, so its VCO input is whatever the system
clock already fixed.

**The clock generator's arithmetic** (28.4.4): the whole divisor is
`2 x I2SDIV + ODD`, and a frame is two channels of 16 or 32 bits -
except with the master clock out, where the divisor becomes 256 x that
whatever the channel width, which is the MCK = 256 x FS rule. I2SDIV 0
and 1 are forbidden. A channel is 32 bits by hardware whenever the data
is wider than 16, whatever CHLEN was written with.

**A request reaches one, two or three cells of the fabric.** The DMA
request mapping gives each instance a short list of (controller,
stream, channel) triples and no others, and the lists differ by part -
the F411's SPI1_TX, SPI4_RX and SPI5_TX have a third cell no other
manual shows, which is why this chapter's placement list is three deep
where the serial and analog ones are two.

## Types and verbs

`Spi<n>` - the resource, one instance in its SPI face. Facts:
`number`, `on_apb2`, `irq`, `has_i2s`. The gate and the reset:
`bus_clock`, `reset`. The configuration: `configure` (the whole
`SpiConfig` with SPE clear, refusing what `spi_config_valid` refuses),
`enable`, `enabled`, `disable` (the errata-aware drain),
`master_receive_only`, `bidirectional_output`, `bidirectional`. The
data: `data` in both directions, `data8`, `data_address`, `rx_ready`,
`tx_empty`, `busy`, `status`, `flag`, `flush_rx`. The errors:
`overrun`/`clear_overrun`, `mode_fault`/`clear_mode_fault`,
`crc_error`/`clear_crc_error`, `frame_error`/`clear_frame_error`. The
CRC unit: `crc_polynomial` both ways, `crc_enable`, `crc_enabled`,
`crc_next`, `crc_next_pending`, `rx_crc`, `tx_crc`. The select:
`software_select`, `software_selected`, `nss_output`. DMA and
interrupts: `dma_requests`, `dma_transmit`, `dma_receive`,
`rxne_interrupt`, `txe_interrupt`, `error_interrupt`, and the ISR body
`isr()` which reports the raised-and-enabled sources and consumes
nothing.

The configuration knobs (`SpiConfig`): `role` (host, client - default
host), `mode` (the four Motorola modes, default 0), `clock` (`SpiClock`
div2..div256, default div16), `bits` (8 or 16, default 8), `lsb_first`
(default false), `nss` (software, hardware_output, hardware_input -
default software), `direction` (full_duplex, receive_only,
half_duplex_out, half_duplex_in - default full duplex), `frame_format`
(motorola, ti - default motorola), `crc` with `crc_polynomial` (default
off, 0x0007), `dma_transmit` and `dma_receive` (default off). The
refusals: an even polynomial, SSOE on a client, TI mode beside a
software select or LSB-first.

The arithmetic, all constexpr: `spi_sck_hz(pclk, code)`,
`spi_rate_for(pclk, max_hz)` (the coarsest code at or below a limit,
nothing when even PCLK/256 is too fast), `spi_frame_is_halfword`,
`spi_data_bits`, `spi_frame_mask`, `spi_mode_cpol`, `spi_mode_cpha`,
`spi_cr1_of`, `spi_cr2_of`, `spi_master_receive_only`, and
`spi_errata_apb_ceiling_hz(pad_speed, for_i2s)` - the errata's own
table as a function.

`I2s<n>` and `I2sExt<n>` - the same block in its audio face, the second
being the full-duplex extension where the part has one. `configure`
(the whole `I2sConfig` with I2SE clear, a master mode refused on an
extension block), `select_spi_mode`, `enable`, `enabled`, `disable`,
`mode`, `i2s_mode_selected`, the generator (`prescaler`,
`prescaler_div`, `prescaler_odd`, `master_clock_out`), the data
(`data`, `data_address`), the flags (`tx_empty`, `rx_ready`, `busy`,
`status`, `right_channel`, `underrun`, `overrun`, `frame_error` with
their clearing verbs), the DMA and interrupt enables, and `isr()`.

The audio knobs (`I2sConfig`): `mode` (client/host x transmit/receive),
`standard` (philips, msb_justified, lsb_justified, pcm), `pcm_long_frame`
(only under PCM), `data` (16, 24, 32 bits), `channel` (16 or 32, forced
to 32 by hardware above 16-bit data), `clock_idle_high`,
`master_clock_out`, `div` (2..255) and `odd`, and the two DMA enables.
The arithmetic: `i2s_frame_factor`, `i2s_fs_hz`, `i2s_prescaler_for`,
`i2s_channel_is_32`, `i2s_config_valid`, `i2s_i2scfgr_of`,
`i2s_i2spr_of`.

`SpiHost<n, pins, TxEngine, RxEngine>` - the engine
[the shared SPI bus](../design/spi-bus.md) drives, with the other
strata's `Request` field for field. `init(clock, max_sck_hz)`,
`rebase(sysclk)`, `clock_for`, `sck_hz`, `max_sck_hz`,
`ceiling_clock`, `reference_hz` (the instance's APB clock), `prime`,
`bit_order`, `lsb_first`, `start`, `isr`, `dma_isr`, `status`,
`recover`, `release`, `claim_nss_pad` - and this stratum's two of its
own, `sck_speed` and `errata_apb_ceiling_hz`/`within_errata_ceiling`,
because on this family the SCK pad's slew class is a correctness
parameter and not a taste.

`SpiClient<n, pins>` - a polled surface with an ISR body: `init`,
`enable(first)`, `disable`, `write`, `writable`, `poll`, `selected`,
`select`, `drive_output` (the dark listener), the error verbs,
`isr()`, `release`, and `frames_ahead` = 1, this silicon's answer to
the one number a portable client would need.

`SpiPins` and `I2sPins` name the pads with the alternate function the
DATASHEET gives each signal there; the device header carries no pin
table, so nothing can check an AF and the bench is the check.

## How to use it

A bus with one device on it, no arbiter, polled - the shape a device
that owns its bus wants:

```cpp
constexpr brio::SpiPins spi5{.sck  = {'F', 7, brio::PinFunction::af5},
                             .miso = {'F', 8, brio::PinFunction::af5},
                             .mosi = {'F', 9, brio::PinFunction::af5}};
using Bus = brio::SpiHost<5, spi5>;
Bus::init(clock);                       // no ceiling: every rate legal

const uint8_t cmd[1] = {0x8F};          // read, register 0x0F
uint8_t who = 0;
Bus::Request r{};
r.cs      = brio::Pin<'C', 1>::ref();
r.cmd     = brio::lend<brio::Lease::reply>(cmd);
r.cmd_len = 1;
r.rx      = brio::lend<brio::Lease::reply>(&who);
r.len     = 1;
r.mode    = brio::SpiMode::mode3;
r.clock   = brio::SpiClock::div16;
r.polled  = true;
(void)Bus::start(r);                    // true: it is done, `who` is filled
```

The same bus shared, through the arbiter - the client posts and gets a
`SpiDone` back, and the app's ISR glue is the one vendor line:

```cpp
using Arb = brio::SpiBus<Bus, Platform, 4>;
extern "C" void SPI5_IRQHandler() {
    if (Bus::isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}
r.polled = false;
r.reply  = brio::reply_to<MyAo, brio::SpiDone>();
brio::post<Arb>(r);
```

With the data phase on the DMA streams - the cells are the request
mapping's and the reserve checks them at compile time:

```cpp
using Tx = brio::DmaTxEngine<2, 4, 2>;   // SPI5_TX: DMA2 stream 4, channel 2
using Rx = brio::DmaRxEngine<2, 3, 2>;   // SPI5_RX: DMA2 stream 3, channel 2
using FastBus = brio::SpiHost<5, spi5, Tx, Rx>;
extern "C" void DMA2_Stream4_IRQHandler() { (void)FastBus::dma_isr(); }
extern "C" void DMA2_Stream3_IRQHandler() { (void)FastBus::dma_isr(); }
brio::Dma<2>::init();
FastBus::init(clock);
```

A three-wire device, whose answer comes back on the pad the command
went out on. The pad must be let go for the receive window and taken
back after - the peripheral does not do it:

```cpp
brio::Spi<5>::configure({.mode = brio::SpiMode::mode3,
                         .direction = brio::SpiDirection::half_duplex_out});
brio::Spi<5>::enable();
cs.clear();
brio::Spi<5>::data(0xCF);                       // the command, one line out
while (!brio::Spi<5>::tx_empty()) {}
while (brio::Spi<5>::busy()) {}
Mosi::function(brio::PinFunction::af5,          // let the output stage go
               {.pull = brio::PinPull::up, .open_drain = true});
brio::Spi<5>::bidirectional_output(false);      // the clock starts here
// ... 28.3.8's stop: drop SPE after the second-to-last frame
```

The audio face, as a master transmitter at 48 kHz:

```cpp
constexpr auto pll = brio::plli2s_config_for(8'000'000, 76'800'000, SysClock::pll.m);
brio::Rcc::plli2s_configure(pll);
brio::Rcc::plli2s_enable(true);
brio::Rcc::plli2s_wait(true);
brio::Rcc::i2s_source(brio::I2sSource::plli2s_r);

constexpr auto pre = brio::i2s_prescaler_for(76'800'000, 48'000, false, false);
brio::I2s<3>::bus_clock(true);
brio::I2s<3>::configure({.mode = brio::I2sMode::host_transmit,
                         .standard = brio::I2sStandard::philips,
                         .div = pre->div, .odd = pre->odd});
Ck::function(brio::PinFunction::af6);
Ws::function(brio::PinFunction::af6);
Sd::function(brio::PinFunction::af6);
brio::I2s<3>::enable();
brio::I2s<3>::data(sample);          // and one per TXE from there
```

## Bench findings

`test_stm32f4_spi` on the STM32F429I-DISC1, SPI5 at PCLK2 = 90 MHz with
the board's gyroscope (chip select PC1, four wires, mode 3) and its
display controller (chip select PC2, D/CX PD13) on the same three pads.

- **The reset values are the chapter's but one.** CR1 0x0000, CR2
  0x0000, SR 0x0002, CRCPR 0x0007, I2SCFGR 0x0000 - and I2SPR reads
  **0x0000**, at boot and again behind an RCC reset pulse, where 28.5.9
  states 0x0002. Printed and not judged: one part, one instance.
- **The eight rates, and what the polled pump costs.** SCK 45000, 22500,
  11250, 5625, 2812, 1406, 703 and 351 kHz off PCLK2. Thirty-two frames
  timed at each: the measured frame time exceeds eight bits at the
  stated rate by **239 to 295 ns (43 to 53 core cycles) at every code
  of the ladder** - a constant, which is what says it is the pump's own
  cost (one write, one RXNE spin, one read) and not a share of the
  rate. At PCLK2/256 that cost is 12 parts per thousand of a frame and
  the measurement IS the wire: 23036 ns against the arithmetic's 22755.
- **A mode fault needs no pad and no second master**: SSI driven low
  under software management raises MODF, and the silicon has cleared
  SPE and MSTR with it - the host is a client until the sequence is
  run. **The flag is not up on the bus access that follows the store**:
  it appeared **50 core cycles** later, and the clearing sequence's
  effect landed **46 core cycles** after its CR1 write. A peripheral on
  a half-rate APB answers a beat late, and a handler must read the flag
  rather than assume it from what it just wrote.
- **The overrun's sequence is two reads and both are needed**: three
  frames clocked with nothing reading DR leaves SR at 0x43 (OVR, TXE,
  RXNE); a DR read alone leaves OVR standing; the DR read followed by
  an SR read clears it.
- **The device answers.** WHO_AM_I reads **0xD3** at 5625 kHz in mode 3,
  eight times running, where a register the part does not implement
  reads 0x7E - so the byte is the device's and not the line's.
- **The rate ladder against a real device**: exact at 22500 kHz and
  below, wrong at 45000 kHz (the device's own datasheet stops at 10 MHz,
  and what an over-clocked device returns is its business - 0xDB with an
  occasional correct read).
- **Two of the four modes read it**: mode 3 (the datasheet's) and mode
  0, the pair with CPOL and CPHA both flipped. What modes 1 and 2 return
  is printed and not judged - 0xD3 and 0xFF here, a timing accident of
  one specimen.
- **Auto-increment is byte-exact**: five control registers read one at a
  time and the same five in one transaction agree; a register written
  and read back agrees for every pattern; a two-register auto-incremented
  WRITE lands in both.
- **The arbiter is unchanged on this architecture.** Four transactions
  posted from one dispatch come back four `spi_ok` replies with an
  ISR-pumped and a polled one interleaved; a two-phase write-then-read
  Request reads the control block; six posted into a four-deep queue are
  all answered and the surplus rejected; an idle bus votes for a sleep.
- **The DMA engines carry the data phase** both ways: a polled request
  completes inside `start()` with the block byte-exact, and an
  ISR-style one answers off the streams' vectors with the select
  released by the completion. Sixty-five frames (a command byte pumped,
  sixty-four streamed) cost 1640 ns a frame at 5625 kHz against the
  wire's 1422, and 22998 against 22755 at 351 kHz - the same constant
  as the pump's, spent once on the command frame and not per frame.
- **The SCK pad's slew class does not decide the answer here.** PCLK2 at
  90 MHz is above ES0206 2.12.4's ceiling at every class, and the
  driver says so; the device nevertheless reads exactly at all four
  classes, `low` included, at SCK 5625 kHz on this board's short
  traces. One board, one load: printed.
- **THE BIDIRECTIONAL LINE NEEDS THE PAD TURNED ROUND BY HAND.**
  Measured three ways against the gyroscope moved to its own three-wire
  interface: with the MOSI pad left an alternate-function PUSH-PULL
  output the read comes back all ones whatever the device drives -
  BIDIOE stops the shifter, not the pad's output stage; with the pad
  moved to a plain INPUT it comes back all ZEROES, because the
  peripheral's input is not taken from there; with the pad kept in its
  alternate function but made OPEN DRAIN with a pull-up the device's
  answer arrives exactly (0xD3 then 0x7E, the same two bytes the
  four-wire path reads). The same one-line write moves the device back,
  and the full-duplex bus is intact after it.
- **The display controller's read path is not reachable on this board.**
  The identity command answers 0xFF on every frame - the pull-up on a
  line nobody drove, through the same transaction that had just read
  the gyroscope, so it is the device and not the driver: the board
  straps the controller's interface mode so that its replies leave on a
  pad the MCU is not wired to.
- **The audio PLL and the word select.** PLLI2S from the same 8 MHz
  crystal, its input divider the MAIN PLL's M (4, so a 2 MHz VCO input),
  N = 192 over R = 5: 76.8 MHz, locked; PLLI2SCFGR refused while it
  runs. I2SDIV 25 with ODD clear puts the arithmetic's 48000 Hz on the
  word select, and the pad's own rising edges counted over 200 ms give
  **47805 Hz** - within a per cent, the window's resolution. CHSIDE
  alternates as the frame does.
- **The master clock output** moves the divisor to 256 x (2 x I2SDIV +
  ODD): I2SDIV 3 gives 50000 Hz by the arithmetic and **49840 Hz**
  measured on the word select, with MCK at 256 x that - 12.8 MHz, too
  fast for a polling loop to count, and its pad read high on 1974 of
  4000 samples, which is what says it is a clock and not a level.

- **On the STM32F411CE's SPI1 at PCLK2 = 96 MHz, against an ILI9481
  panel on a breadboard: THE INTERRUPT-PUMPED REQUEST AT PCLK2/8 NEEDS
  THE SCK PAD AT `medium` OR `low`.** Polled requests at 12 MHz are
  byte-exact at every slew class, interrupt-pumped ones at 6 MHz too;
  at 12 MHz with the pad at `high` or `very_high` the device takes
  neither the command nor the data of an interrupt-pumped request (a
  block written reads back as what was there before, a read comes back
  all ones), and with the pad at `medium` or `low` every byte lands -
  through the software pump and through the DMA engines alike, whose
  interrupt-style request pumps its command byte the same way. IT IS
  THE WIRE AND NOT THE DRIVER: slowing the MOSI pad alone to `medium`
  with SCK left at `very_high` lands every byte too, the data pattern
  makes no difference (all zeros fail as the pattern does), and with a
  logic analyser's probes hung on the lines every case passes at every
  slew class - the probes' capacitance slows the edges the way the pad
  setting does. The analyser then said which wire: with its probe on
  SCK alone the case still fails and the clock at the connector is
  CLEAN (eight rising edges a byte, 82 to 84 ns periods, no pulse under
  40 ns, no extra edge); with its probe on MOSI alone the case passes.
  MOSI's fast edge is the actor and the damage is beyond the connector
  - in the module's traces or the controller's input - where a logic
  analyser cannot look. ES0287 2.11.4 is not involved (it asks for the
  FASTER pad at a high APB). The rule for a breadboard: at PCLK2/8 keep
  SCK or MOSI at `medium`; `sck_speed()` is the lever the driver has,
  and MOSI's slew is set at init to `very_high` with no verb of its own
  yet - the finding argues for one. Why the polled loop survives the
  same edges on long requests and not on short ones was not resolved,
  and does not need to be: the fix is on the pads, or on a printed
  board's short traces.

## Not covered yet

Driver gaps:
- The CRC unit beyond its verbs and the even-polynomial refusal: no
  device on the bench speaks a CRC-checked SPI, and a CRC measured
  against nothing is a register read-back. What would close it is a peer
  board running `SpiClient` with the same polynomial - the AVR and the
  SAM strata's suites do exactly that over a wire this board has not
  got.
- TI frame format: the configuration is written and refused where the
  chapter says it must be, but nothing on this board frames its
  transactions that way. Born with its first device.
- **`SpiClient` is driven on SPI2 against a second board.** The peer
  instrument this stratum carries answers a far board's host ONE FRAME
  AHEAD over PB12..PB15 at 281.25 kHz of SCK: a dark client that drives
  MISO only inside an answer window, the four transfer modes and both
  bit orders byte-exact in both directions, a client that never drains
  keeping ONE frame and raising the overrun, the roles inverted with
  this board clocking the far one as host, and a ten-second stress of
  23 exchanges with no error at either end. ES0206 2.12.5's BSY is
  never looked at on this side - RXNE is the witness. And THE ANSWER
  LINE'S SLEW CLASS IS A CORRECTNESS PARAMETER: at the driver's
  `very_high` the far board's falling-edge modes slip and an engined
  exchange comes back nine bytes of sixteen wrong; at `low`, with the
  far board's own pads left at its driver's fastest, everything is
  byte-exact and the rate ladder is unchanged.
- The DMA engines on 16-bit frames: the engines carry bytes, and a
  16-bit request falls back to the pump - the other strata's rule, and
  the transfer-granularity question the first portable example is meant
  to settle.
- The I2S beyond a master transmitter: no codec on the board, so
  reception, the slave modes, the full-duplex extension block on the
  wire and the DMA-fed audio stream have no peer. The extension block's
  refusals and its register surface are exercised; its data path is not.
- The I2S clock sources other than the audio PLL's R output: the
  I2S_CKIN pad has no clock on it, and the F446's four-way selector
  (with the main PLL's R and the root among them) is a part this
  stratum has one of, on another desk position.
- A display driver over this bus: the board's controller is a peer whose
  reads cannot answer, and what a display wants above the transaction
  descriptor is the first portable example's business and not this
  chapter's.

Implemented, not bench-verified (each with what would measure it):
- The instances other than SPI5, SPI2 and SPI1 (SPI3, SPI4, SPI6 -
  compiled on every header that has them, none driven): a wire between
  two of them, or a device on one. SPI1 is driven on the STM32F411CE
  against a display and a touch controller (the finding above).
- The receive-only and half-duplex-in configurations as a Request's
  `direction`: the engine's transactions are full duplex by
  construction, and the resource's simplex modes were driven by hand in
  the bidirectional letter alone.
- 16-bit frames end to end: the frame size travels per request and the
  engine packs two bytes low-first, but the device on this bus speaks
  bytes. A converter with 16-bit registers would measure it.
- The hardware NSS arrangements as the ENGINE's select (`claim_nss_pad`,
  SSOE, the hardware input): the bus AO's select is a GPIO on purpose,
  and a multi-master bus is what would exercise the input.
- `recover()` after a wedged transaction, and the arbiter's per-bus
  timeout with it: a lost completion has to be staged, as the other
  strata's suites stage it, and that wants a second device to hold the
  bus.
- `rebase()` and the SCK ceiling across a clock change: this family has
  no dynamic clock yet ([clock.md](clock.md)).
- The error interrupt (ERRIE) and the transmit interrupt (TXEIE): the
  engine arms RXNE alone, and the errors were measured by polling.
- The frame error (FRE): it is a TI slave's and an I2S slave's, and
  neither role has a peer here.
