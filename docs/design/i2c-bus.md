# The I2C bus

This page decides how a program drives I2C devices through the same
arbiter as the SPI bus: one request is one bus tenure from START to
STOP (a write, a read, a write-then-read or a probe), the requester
posts it and receives an `I2cDone` with a status that says what the
wire did, and the bus active object runs one tenure at a time on the
family's engine. It also decides the arbiter itself, `BusMaster`,
which both buses share, and its per-bus timeout. It is written for an
embedded programmer who has read a register over I2C with a vendor
library and wants the same from several active objects without
collisions. [spi-bus.md](spi-bus.md) is the reference form of the
pattern; this page repeats nothing it says.

The smallest use: an `I2cHost<n, ...>` of the family, an
`I2cBus<Host, P>` in Tenuto's list, and a client that posts
`Host::Request{addr, tx, rx, reply, speed}` and handles `I2cDone` in
its own dispatch. The prerequisite that decides correctness, as on
SPI: **the request's buffers are lent until `I2cDone` arrives**, and
the client must not touch them before.

## BusMaster: the arbiter generalized

The arbiter never looks at a byte. It owns the pending FIFO, the
reject-when-full policy, the `ReplyTo` return channel and the engine
handshake (`Bus::start`, then `TransferDone`). Nothing in that is
specific to a bus, so one class serves both: `BusMaster<Bus, P,
pending_depth>` in `util/bus_master.hpp`, with `BusDone{status}`,
`bus_ok`, `bus_rejected` and `TransferDone` as its own vocabulary.

The per-bus names are aliases that cost nothing, on purpose:
`util/spi_bus.hpp` (`SpiBus`, `SpiDone`, `spi_ok`, `spi_rejected`) and
`util/i2c_bus.hpp` (`I2cBus`, `I2cDone`, `i2c_*`). A display client
that reads `SpiDone` says which wire its bytes took; a `BusDone` in
the same place would be less information for no gain. Measured: the
aliases cost zero flash.

Status codes are one byte, split by ownership: `bus_ok = 0` and
`bus_rejected = 1` are the arbiter's, `bus_timeout = 255` is the
timeout's, and every value from `bus_engine_status = 2` up belongs to
the engine's vocabulary and travels untouched from `TransferDone` to
the requester's reply. The I2C engine borrows `i2c_rejected` (which is
`bus_rejected`) for the refusals it makes on its own, a speed the
clock in force cannot produce, an address no request may name and, on
a host that takes `Request::engine`, `dma` with no engines bound
(below): the same meaning for the requester either way, nothing was
executed and no byte moved. SPI has no
wire-level outcome to report (no ACK, no arbitration), so the codes it
adds to the arbiter's are its hosts' own ([spi-bus.md](spi-bus.md)); I2C
adds the four an I2C master can observe:

| Code | Meaning |
|------|---------|
| `i2c_nack_addr` | no ACK on the address: nobody home (the probe result an address scanner reads) |
| `i2c_nack_data` | no ACK on a written byte: the device refused or finished early |
| `i2c_arb_lost` | lost arbitration to another master: not our bus, no STOP sent |
| `i2c_bus_error` | protocol violation on the wire: peripheral forced back to idle |

## The stack

```
     client AO                          scanner / DAC client
        \  post(Request{...})              /
         v                                v
     I2cBus<Bus, P>          util/i2c_bus.hpp   = BusMaster: arbitration + replies
         |  Bus::start(req)
         v
     I2cHost<n> engine       <stratum>/{twi,i2c}.hpp   START/Sr/STOP, ACK policy, the per-byte interrupt
         |
     ISR glue in the app     posts TransferDone{I2cHw::status()} on completion
```

Layering as for SPI: `I2cBus` and `BusMaster` are `util/`'s, pure and
tested on the host through the SPI alias in `test_spi_bus` (the same
class); the `I2cHost<n>` engine belongs to a target stratum, as a task
over that silicon's own resource. The application's interrupt handler
binds the vector.

### Realizations

Common to all: the `Request` field for field (`addr`, `tx`, `tx_len`,
`rx`, `rx_len`, `reply`, `speed`); `I2cSpeed` word for word (the
CH32V00x's, the CH32V203's, the CH32X035's and the STM32F4's first
block stop at `fast_400k`, those peripherals having no Fm+); the
engine's functions `init`, `start`, `isr`, `status`, `rebase`,
`recover`, `release`, `unstick`; the empty probe (both lengths zero)
served by every row, on the DesignWare block as a one-byte read, its
command FIFO having no address-only entry; `start()`'s refusal of an
address outside 0x08..0x77 (below); and the vocabulary `I2cDone` and
`i2c_*`, produced on the wire. Each is measured against a second
chip where a peer board was on the desk, against the chip's other
instance on the two DesignWare families, against the touch controller
its board carries on the STM32F4 and, on the second design of that
family, against a bus with nobody on it. What differs is the resource
under the task and the shape of the rate arithmetic each chapter
imposes. In the table below a pump is the host's interrupt moving data
byte by byte, an engine slot a DMA channel bound to the host ("Which
engine moves a half") and smart mode the block's acknowledge riding the
data read ("Engine behaviour on the wire").

`addr` is a 7-bit address from 0x08 to 0x77, the range the I2C
specification leaves to devices (NXP UM10204 3.1.12), and every host's
`start()` answers any other value `i2c_rejected` at once, before
anything reaches the wire - one compare against the one definition of
the range, `i2c_address_in_range()` in `util/i2c_bus.hpp`. The sixteen
values outside it are reserved: 0x00 is the general call, 0x01 to 0x07
the START byte, CBUS, other bus formats, a future purpose and the
Hs-mode codes, 0x78 to
0x7B the first byte of a 10-bit address, 0x7C to 0x7F the device ID and a
future purpose. The reason is the wire and not tidiness: the STM32F1's
event machine, which the STM32F4's classic I2C and the three WCH families
carry, has no addressing mode of its own as a controller and sends an
address byte of the form 1111 0xx0 - a 7-bit address of 0x78 to 0x7B -
as a 10-bit header, where every other block sends it as a plain 7-bit
address, so the same request would mean two different things on two
families. The general call is refused with the rest until it is a
feature of its own, and 10-bit addressing is offered by no host. Behind
the contract, each F1-lineage engine is also total in its address
phase: an event that phase does not serve, ADD10 above all, ends the
tenure with `i2c_bus_error` instead of re-entering the vector for ever
(measured on the STM32F446, the CH32V303, the CH32X035 and the
CH32V006 with the refusal compiled out, each family's I2C page).

| stratum | realization | beyond the contract |
|---|---|---|
| avrxt | `I2cHost<n, route>` over `Twi<n>` (`avrxt/twi.hpp`), a resource that carries the client half too | the rate readback is `actual_scl_hz(t_rise_ns)`, what the register in force gives, under the chapter's rise-time budget; `speed_ok(speed)` as on every row that caches its rates, plus `speed_ok()` for the speed in force; `quick_command(bool)` is the TWI's own hardware mode (QCEN: every request an address-only frame); `bus_state()` is the wire's state from the bus monitor; `speed()`, `baud()`; the host's handler calls nothing, every register function it touches inlined, since one call in an AVR vector saves the whole call-clobbered set, and tests the two common cases with one masked compare each; smart mode the engine's default (`Options::smart`), ACK armed once a read tenure; a read to an absent address answers `i2c_nack_addr`; the client's SDASETUP is eight clocks in every default, four losing a smart client in a long read on the AVR EA ([the AVR TWI page](../avrxt/twi.md)) |
| samc21 | `I2cHost<n, pads, generator>` over `I2cm<n>` (`samc21/i2c.hpp`; `I2cs<n>` is the client's resource, two over one SERCOM) | `init` takes the stated core rate (`reference_hz()`); the rates are a per-speed cache: `speed_ok(speed)`, `scl_hz(speed)`, `baud_of(speed)`; `idle()` is the engine's phase, not the wire's; smart mode always on (the pump's last read byte closed STOP+NACK first, ACK reset at every read phase, since a read's closing NACK persists on this block); no engine slot, every tenure on the pump, one interrupt a byte, the family's DMA serving its Uart's transmitter alone ([the SAM C21 DMAC page](../samc21/dmac.md)); `configuration(speed)` for code that reconfigures the peripheral behind the engine |
| stm32g0 | `I2cHost<n, pins, TxEngine, RxEngine>` over `I2c<n>` (`stm32g0/i2c.hpp`, one TIMINGR word) | the same stated rate and per-speed cache (`speed_ok`, `scl_hz`, `timing_of`, `kernel_hz()` for the kernel-clock multiplexer); two optional DMA engine slots with `dma_isr()`; `fast_plus_drive()` (SYSCFG's Fm+ pad drive); `spurious_bus_errors()` (an erratum's counter); `idle()` as the SAM's; the byte counter with AUTOEND carries the tenure, the first data byte preloaded into TXDR before the START, a read's last byte taken at STOPF, a NACK named at STOPF with no NACKIE armed (the address's when the preloaded byte never left TXDR); a tenure's interrupt and DMA enables written in one CR1 store at `start()` and put back in one at the end (`enables()`), so an engined tenure leaves no enable behind; `Request::engine` and `has_dma` (below, "Which engine moves a half"), `dma` carrying a half at any length and a half with no buffer staying on the pump |
| stm32f4 | `I2cHost<n, pins, TxEngine, RxEngine>` over `I2c<n>` (`stm32f4/i2c.hpp`, the F1's event machine with the rise-time register the CH32V00x's has not) | the stated bus rate and the per-speed cache as the SAM's (`speed_ok`, `scl_hz`, `timing_of`, `reference_hz()` = the instance's own APB1 clock); an `I2cHostConfig` at init (fast mode's duty shape, the wire's two rise times, the noise filters); `digital_filter_max()` (table 122's ceiling, advice and not a refusal) and `repeated_start_setup_at_risk()` (an erratum's window a write-then-read sits in); `spurious_bus_errors()` as the G0's, and here a bus error never ends a tenure at all, the manual and the erratum agreeing that a controller's transfer is unaffected; two vectors, `isr()` and `error_isr()`; the engine slots on the request mapping's own cells with `dma_isr()`, `Request::engine` and `has_dma`, `dma` carrying every half at any length, the one-byte read included (ACK cleared before ADDR's clear and the STOP set after it, RM0090 27.3.8), its receive stream started at the read's SB; `unstick()` returns the pulses it took; `start()` waits, bounded, for the last STOP to leave, as the CH32V00x's does, about one bit period when a tenure is started at the last one's completion, nothing in the chapter signalling the STOP's leaving; the event interrupt raised by `start()` and dropped at the completion in one CR2 store (`control2()`), so the BTF a write's STOP leaves standing no longer re-enters the vector; on the pump, the first two data bytes loaded in the ADDR entry (TxE rises within twenty cycles of the ADDR clear), so a one- or two-byte write and a register index take no TxE interrupt; the repeated START of a write-then-read requested on the last byte's TxE with a dummy store into DR (RM0090 27.3.3's note), and a NACK of that last byte answered `i2c_nack_data` with no CR1 access while START stands (27.6.1 forbids one; a withdrawal there hung the event vector with the core at 48 MHz, measured), the Sr the controller generates closed with a void write and its STOP; `claim_smba_pad()` for the SMBus alert; the address phase total, an ADD10 or any event it does not serve ending the tenure `i2c_bus_error` with the STOP and then a DR store that clears ADD10, which the STOP alone does not (measured) |
| stm32f4 (2) | `FmpI2cHost<n, pins, TxEngine, RxEngine>` over `FmpI2c<n>` (`stm32f4/fmpi2c.hpp`, the second I2C design of this family: the STM32G0's register file under the name FMPI2C1, on the F410, F412, F413/F423 and F446) | the STM32G0's surface almost entire (`speed_ok`, `scl_hz`, `timing_of`, `kernel_hz()` for the kernel-clock multiplexer, `fast_plus_drive()`, `spurious_bus_errors()`, `idle()`, `unstick()` returning the pulses it took, `recover()` = 23.4.6's PE cycle and nothing more), and three speeds where every other realization of this family has two; `reference_hz()` is the APB clock the registers are on while `kernel_hz()` is the one the arithmetic divides, which here are two different rates, and `transmit_stall_risk()` publishes the erratum's band on the ratio between them; two vectors, `isr()` and `error_isr()`, with a NACK on the event one (it is not an error on this block) - the address's told from the data's by what of the half reached the wire, the byte pump's position or under the engines the transmit stream's count - and a controller's BERR counted and never acted on; the engine slots on the mapping's single cell per direction with `dma_isr()`, `Request::engine` and `has_dma`, `dma` carrying both phases, a half with no buffer apart, with the engines armed before the START (23.4.16's order, and an erratum's workaround a tenure under `interrupt` does not take); `start()` does not wait for the last STOP, since this block queues a START behind one itself; `claim_smba_pad()` for the SMBus alert |
| ch32v00x | `I2cHost<1, pins, TxEngine, RxEngine>` over `I2c<1>` (`ch32v00x/i2c.hpp`, the F1's event machine with no rise-time register) | the stated bus rate and the per-speed cache as the SAM's (`speed_ok`, `scl_hz`, `timing_of`, `reference_hz()` = HCLK); a `duty` at init (fast mode's 2 or 16/9 shape); two vectors, `isr()` for the events and `error_isr()` for AF/ARLO/BERR; the engine slots fixed to DMA channels 6 (TX) and 7 (RX) with `dma_isr()`, `Request::engine` and `has_dma`, `dma` carrying every half at any length, the one-byte read and the write half of a write-then-read included, the transmit block's completion handing the tenure back to the event vector; `unstick()` returns the clocks it took; `start()` waits, bounded, for the last STOP to leave (the F1 lineage cannot queue a START behind one), the one engine whose `start()` spends dispatch time on the bus; the event line (ITEVTEN) raised by `start()` and dropped when the tenure ends, so the BTF a requested STOP leaves standing (15.10.6) does not re-enter the vector; a START with POS and ACK down in one CTLR1 store; the repeated START of a write-then-read requested on the TxE that says the last written byte went into the shifter (15.10.1, 15.3), the order a polled CH32 target needs; a NACK of that last byte answered as `i2c_nack_data` without a CTLR1 store at the NACK (the controller generates the requested Sr anyway, and a store there races it: a withdrawal wedged the bus at 400 kHz, measured) but by closing the tenure behind that Sr with a void write and its STOP, one address byte on the wire when a byte is refused; `unstick()` clears the STOPF its own STOP leaves; the address phase total as the CH32V303's, an ADD10 or any event it does not serve ending the tenure `i2c_bus_error` with the STOP and a DATAR store (measured on the CH32V006) |
| ch32vx03 | `I2cHost<n, pins, TxEngine, RxEngine>` over `I2c<n>` (`ch32vx03/i2c.hpp`, the F1's event machine with a rise-time register, two instances on PB1) | the stated bus rate and the per-speed cache as the SAM's (`speed_ok`, `scl_hz`, `timing_of`, `reference_hz()` = PB1) with a `duty` and the wire's rise time in nanoseconds at init (the third timing register, RTR); the bus clock is refused outside 19.12.2's 4..60 MHz window, which is narrower than the stratum's own PB1 ceiling; two vectors, `isr()` and `error_isr()`; the engine slots fixed to the instance's own channels (I2C1 on DMA1's channels 6 and 7, I2C2 on its 4 and 5) with `dma_isr()`, `Request::engine` and `has_dma`, `dma` carrying every half at any length, the one-byte read included; `unstick()` returns the clocks it took; `start()` waits, bounded, for the last STOP to leave, as the CH32V00x's does, and then, if BUSY still stands and both lines read high, takes 19.12.1's SWRST out of the way and counts it (`unwedges()`), the one realization that answers a peripheral-side stuck BUSY inside `start()`; `busy()` for a sleep site; the client half takes `I2cClientOptions` (a polled client with its three enables down) and has `flush()`, the PE cycle for a byte the controller never clocked; the repeated START of a write-then-read requested on the TxE of the last written byte, while it shifts, and not at BTF, since a polled CH32 target loses that byte otherwise (measured on the CH32V303 and again on the CH32V203's self-link), and a NACK of that last byte answered as the data's without a CTLR1 store at the NACK (the controller generates the requested Sr anyway, and a withdrawal there races it: lost with the core at 8 MHz, measured) but by closing the tenure behind that Sr with a void write and its STOP; the event line raised by `start()` and dropped at the completion in one CTLR2 store, so the BTF a write's STOP leaves standing no longer re-enters the vector; on the pump, the first two data bytes loaded in the ADDR entry, so a one- or two-byte write takes no TxE interrupt; the address phase total, an ADD10 or any event it does not serve ending the tenure `i2c_bus_error` with the STOP and then a DATAR store that clears ADD10, which the STOP alone does not (measured on the CH32V303) |
| ch32x035 | `I2cHost<1, pins, TxEngine, RxEngine>` over `I2c<1>` (`ch32x035/i2c.hpp`, the F1's event machine with no rise-time register, one instance, every bus at HCLK) | the stated bus rate and the per-speed cache as the SAM's (`speed_ok`, `scl_hz`, `timing_of`, `reference_hz()` = HCLK) with a `duty` at init; the block asked first: the datasheet withdraws the I2C from some lots and no register names one, so `I2c<1>::present()` and both `init()`s check the block holds its timing; six columns, the part's first off the debug port the default, the debug port's refused while the probe owns it and the USB pads while USB_IOEN does; two vectors, `isr()` and `error_isr()`; the engine slots fixed to DMA channels 6 and 7 (ORs shared with USART2) with `dma_isr()`, `Request::engine` and `has_dma`, `dma` carrying every half at any length, the transmit block's completion handing the tenure back to the event vector (a plain write then ends on BTF, a write before a read requests the repeated START on its last byte's TxE); on the pump, the first two bytes loaded in the ADDR entry; `start()` waits, bounded, for the last STOP to leave and takes 15.9.1's SWRST for a BUSY standing over an idle wire (`unwedges()`); the repeated START on the last written byte's TxE and a refused last byte closed with a void write; the event line raised by `start()` and dropped at the completion in one CTLR2 store; `unstick()` drives a line low push-pull and releases it as an input, this series having no open-drain GPIO; the client polled (`I2cClientOptions`) or on its vectors, with `flush()`; the address phase total as the CH32V303's, an ADD10 or any event it does not serve ending the tenure `i2c_bus_error` with the STOP and a DATAR store (measured on the CH32X035F8U6) |
| rp2040 | `I2cHost<n, pins, TxEngine, RxEngine>` over `DwApbI2c<n>` (`rp2040/i2c.hpp`, the family's aliases of the IP stratum's templates in `dw_apb_i2c/i2c.hpp`: the Synopsys command FIFO, where an entry carries its RESTART and STOP bits and one TX_ABRT carries every failure) | the stated rate (clk_sys, `reference_hz()`) and the per-speed cache as the SAM's (`speed_ok`, `scl_hz`, `timing_of`); `idle()` as the SAM's; `start()` answers a second refusal synchronously, `i2c_bus_error` for a block that would not disable to take the address (IC_TAR takes a write only with ENABLE clear); the probe is a one-byte read; `Request::engine` and `has_dma`, `dma` putting the read phase alone on the engine slots at any length (the receive engine collecting every byte, a transmit engine of `uint16_t` entries pouring the plain read commands between the first and the last from a fixed cell, so a read of one or two takes the receive engine alone) with `dma_isr()`, the write half on the pump (a byte beat lands in the command register replicated across the word, its copy in bit 8 a read command, RP2040 2.1.4); the pump reads at half the FIFO (RX_TL 7, written once) and pours the read commands the returned bytes free, so a tenure of eight bytes or less is one interrupt and a longer one one per eight; every tenure ends on its own STOP_DET - its own when the transmit FIFO is empty, the host state machine idle and no TX_ABRT standing, the block raising STOP_DET for every STOP on the wire - and the disable / IC_TAR / enable cycle is skipped while the address and the speed stand and the last tenure ended on its STOP (an abort, a fault, `recover()` force it), which takes the block's bus-timed disable (1.5 us at 400 kHz, 6 at 100) out of the dispatch; `unstick()` returns the clocks it took; `recover()` resets the block through the reset controller, the one way out of a host holding SCL with no STOP in sight |
| rp2350 | `I2cHost<n, pins, TxEngine, RxEngine>` over `DwApbI2c<n>` (`rp2350/i2c.hpp`, the same IP stratum's templates) | the same Synopsys block, register for register, and therefore the same driver: every word of the row above holds, and this family writes a traits type, the write half on the pump there too, though an alias that zero-fills the unused byte lanes (RP2350 2.1.5) could carry it (not built). What is the chip's, and all of it around the block rather than in it: one function column over a longer bank where its UART has two; pads that come up isolated, so the pad functions the unstick drives an open-drain line with must drop the latch as they write; a GPIO bank of two words, which those same functions must tell apart; a request table and a reset controller whose bits have all moved from the RP2040's; and a bus clear by hand paced by the one microsecond ruler both of this chip's processor architectures can read ([the RP2350 I2C page](../rp2350/i2c.md), [the DesignWare I2C page](../dw_apb_i2c/README.md)) |
| host | none | `I2cBus` = `BusMaster`, host-tested through the SPI alias (`test_spi_bus`, `test_bus_master`, the same class); the IP stratum's own driver against a DW_apb_i2c made of RAM (`test_dw_apb_i2c`, `host/sim_dw_apb_i2c.hpp`) |

None of those differences is a spelling of the same function:
`actual_scl_hz` and `scl_hz` answer different questions (the register
in force against a cached speed), `bus_state()` and `idle()` look at
different things (the wire against the engine), and `quick_command`
is a hardware mode the other families do not have (their empty probe
is the same tenure with no data, which the AVR serves too).
`speed_ok(speed)` is one function on every row that caches its rates.

The client side is a surface per position (`I2cClient` on each
stratum, the application's protocol over `Twi<n>`, `I2cs<n>`,
`I2c<n>`, `I2c<1>` or `DwApbI2c<n>`). Every stratum but the AVR speaks
one vocabulary back (`addressed`, `data_ready`, `stop_seen`,
`give`/`take`, `host_reads`; the CH32V00x, the CH32V203, the STM32F4
and the DesignWare families add `service()`, one `I2cClientEvent` per
interrupt body; the DesignWare block has no `addressed`, raising no
address-match event, so a tenure is known by its first byte or its
first read request, and the clock stretch of a read request is the
block's own). The AVR's is its own (`addressed`, `data_ready`,
`receive`/`respond`, `complete`). A convergence, not a task.

The peripherals share almost nothing below the contract: a TWI with
its own baud three-step, a SERCOM with a select-then-use register
file, an I2C with one timing word, the F1's event machine with a
clock register and no rise-time one, and a command FIFO whose entries
carry the bus conditions as bits. That one descriptor serves them all is
the point worth recording. Where two families carry the same block the
descriptor stops being a claim: the DesignWare's driver is one file
both of them alias ([the DesignWare I2C page](../dw_apb_i2c/README.md)).

## The transaction descriptor (`I2cHost<n>::Request`)

`{addr, tx span, rx span, reply, speed}`, and `engine` in the padding
after `speed` on a host that takes it: 10 bytes on the AVR and 28 on a
32-bit target, over the kernel's guideline of about 8 bytes for an
event, legal as on SPI because the request is the arbitration token. One request is one bus tenure from
START to STOP, in the shapes I2C devices actually use:

| Shape | Fields | On the wire |
|-------|--------|-------------|
| write | tx set, rx_len 0 | S addr+W data... P |
| read | tx_len 0, rx set | S addr+R data... P |
| write-then-read | both | S addr+W tx... **Sr** addr+R rx... P |
| probe | both empty | S addr+W P -> `i2c_ok` / `i2c_nack_addr` |

Write-then-read is the register-access idiom (index, repeated START,
value) and it must be one request: the repeated START is what keeps
another client from slipping in between, so the SPI rule "the request
is the complete script of one bus tenure" holds word for word. The
probe is what a scanner sends and, unlike the SPI zero-length request,
it does touch the wire (its address phase is the transaction), so
`start()` is asynchronous on I2C whenever the wire moves: every such
request ends in a `TransferDone`. What completes inside `start()` is
what never reached the wire. The refusals: a request naming a speed the
clock in force cannot produce (`speed_ok(speed)` says so), an address
outside the range above, or `dma` with no engines bound is answered
`i2c_rejected` inside `start()`, through the engine's `status()`, with
no byte moved; a tenure is never run at a rate nobody asked for and
never approximated in silence. And on the blocks where `start()` can
find the peripheral not its own, `i2c_bus_error` the same way: a START
still standing from a tenure never closed on the STM32G0 and the
STM32F4's FMPI2C1, a system operation that never settled on the SAM
C21, a block that would not disable on the two DesignWare families. The
arbiter replies with `status()` on every synchronous
return (`util/bus_master.hpp`'s contract), so the requester sees these
in the same `I2cDone` as any outcome of the wire; the
completion policy does not judge them, and no retry would change a
refusal. Every engine completes them the same way: status set, nothing
armed, true returned.

The `tx` and `rx` buffers are `Lease::reply` loans and say so in their
field types: the requester keeps them alive and untouched until its
`I2cDone` arrives.

Bus speed travels per request (`I2cSpeed::standard_100k`,
`fast_400k`, `fast_plus_1m`), like clock and mode on SPI: a shared bus
can carry a 100 kHz sensor and a 400 kHz DAC. A speed change costs an
enable cycle and a forced idle at `start()` (the AVR's MBAUD, for one,
may only be written with the host disabled), paid only when the speed
actually moves and never per byte. How MBAUD is solved, and why the
bus's rise and fall times are arguments rather than assumptions,
belongs to the engine ([the AVR TWI page](../avrxt/twi.md)).

## Which engine moves a half (`Request::engine`)

A tenure has up to two data halves, the bytes it writes and the bytes it
reads. Everything else in it - the address, the repeated START, the end -
belongs to the host's interrupt on every block, and a host may move the
data the same way: the interrupt pump, one entry per byte or per FIFO
level, is present on every host. Where DMA channels are bound to the
host (its engines), each half can ride them instead, and the program
names which, per request:

| `I2cEngine` | what it asks |
|---|---|
| `interrupt` | the pump, at any length; what a request that does not name the field gets |
| `dma` | the engines, at any length, for every half they can carry |

**The host never chooses.** Neither value hides a threshold: a request
naming `dma` puts a one-byte half on the engines as readily as a long
one. The host could count only its own core cycles, while the program
knows what else must run, and whether the core should sleep, while the
wire does. A wrong choice costs core cycles or a sleep, never correctness.

**A half the engines cannot carry completes on the interrupt**, under
`dma` too, and is not refused. Here I2C parts from SPI, whose host refuses
a request its engines cannot carry ([spi-bus.md](spi-bus.md), "Two ways to
run a request"): there the fallback would be the polled loop inside
`start()`, holding the arbiter's dispatch for the request's wire time, so
the place a buffer happens to sit would decide whether the dispatch is
held. On I2C the interrupt path is itself asynchronous - `start()` returns
false and the tenure ends on an interrupt either way - so carrying a half
on the pump changes the core's cycles and nothing of the program's shape.
The halves the engines leave to the pump: on every host with engines, a
half whose channel refuses the launch (one another owner holds, and on the
CH32V303's DMA1 a run across a 64 KB boundary), found before the half
moves; on the STM32G0 and the STM32F4's FMPI2C1, a half with no buffer; on
the DesignWare block, the write half, the read half's first and last
commands and the probe ([the DesignWare I2C page](../dw_apb_i2c/README.md)
says which of those the silicon decides). Each family's document names
its own.

**`dma` on a host with no engines bound is refused**, a request that
cannot be served as asked: `start()` returns true with `i2c_rejected` in
`status()`, nothing armed and nothing on the wire, the path the
out-of-range address and the unproducible speed take. Every host that
takes the field publishes `static constexpr bool has_dma`, so a program
that depends on the engines can say so in a `static_assert`.

**Which to name, by the cycles.** As guidance the host never reads, each
family's document states from which length the engines cost the core
fewer cycles than the pump, for a write and for a read: the smallest
length N with

```
E - F <= N x b
```

E is the engines' fixed cost to the core for that half: programming and
starting the channel, any interrupt its completion needs, and completing
the channel at the tenure's end. F is what the pump spends on the same
half beyond its work per byte. b is the pump's cost of a byte: its entry,
the byte, its exit. A block whose pump takes no interrupt for a half's
first byte says so in the arithmetic, N - 1 in place of N. The figures may
be measured, or estimated by counting cycles in the release listing,
whichever is cheaper on the family: the right order of magnitude decides
a count this small.

Speed does not move that count. A pump whose service of a byte outlasts
that byte on the wire stretches the clock, which costs the wire's time and
no core cycle, so the tenure runs slower than the speed asked. A family
says, with the arithmetic, whether this happens at its top speed, and a
program that needs the rate names `dma`.

What the cycles do not count is time awake. A platform whose idle cannot
sleep while a DMA channel works keeps the core awake for the whole half
under `dma`, and then the time awake is the engines' cost, far above any
count of cycles. The case is the CH32V003, the CH32V203 and CH32V303 and
the CH32X035, whose `idle()` asks the chip traits'
`bus_masters_working()` before it sleeps (`qingke/platform.hpp`); on the
CH32V006, whose Sleep lets the DMA work, that answer is a constant. On
those parts `interrupt` lets the core sleep between bytes, and `dma` is
how a program that wants the core free for other work during the half
asks for the engines anyway.

**Which hosts take the field**: every host with engine slots - the
STM32G0's, both designs of the STM32F4, the CH32V00x's, the CH32V203's and
CH32V303's, the CH32X035's and the DesignWare host of the RP2040 and the
RP2350. The AVR's and the SAM C21's hosts do not carry it, and a request
that names it does not compile there.

## Engine behaviour on the wire

- On any NACK the engine still issues STOP: the bus is released before
  the failure is reported.
- Arbitration lost: the flag is cleared, no STOP (the other master
  owns the bus), `i2c_arb_lost` reported.
- Bus error: the flag is cleared and the bus state forced to idle,
  `i2c_bus_error` reported - but on the STM32G0 and the STM32F4's two
  blocks, where a controller's bus error is no evidence (an erratum's
  spurious flag): counted in `spurious_bus_errors()`, the tenure going on.
- The ACK policy is the engine's: each received byte is answered with
  ACK and receive-next, or NACK and STOP. Smart mode, where the
  silicon has it, is an option of the engine (the acknowledge then
  rides the data read), invisible here.
- Pull-ups are external, their value the board's: the internal ones are
  far too weak for I2C edges. The peripheral drives the pins
  open-drain by itself; `init` only routes them.

## The per-bus timeout

A client holding a line low for ever (SDA, or the clock in the middle
of a byte) leaves a tenure in flight: the kernel keeps running, since
nothing waits, but the bus active object stays busy and later requests
pile up until rejected. Loud, but not recovered. Of the families
measured, one silicon answers a held clock by itself: the SAM C21's
SERCOM has SMBus time-outs which, armed, bound a client's hold of SCL
as 33.6.3.1 reads, a 40 ms stretch ending the tenure at 29 ms with
`i2c_bus_error` (measured, [the SAM C21 I2C page](../samc21/i2c.md)); a
held SDA they do not see. The AVR's TWI has none at all. The STM32G0's
time-outs, which RM0444 32.4.12
describes in words that read otherwise ("if SCL is tied low for longer
than ..."), do not answer it. Measured with a control on each side, the
host's own unserved hold trips TIMEOUTA, a peer's 6 ms
hold trips neither TIMEOUTA nor TIMEOUTB
([the STM32G0 I2C page](../stm32g0/i2c.md)). On that part a wedged SDA
does not even raise an error: the START simply parks with BUSY
standing, which is the AVR's and the SAM's behaviour met a third time -
unless the block was disabled and enabled first (a PE cycle: a speed
change, or the arbiter's own
`recover()`), which clears BUSY, and then the START goes out into the
held line and loses its address: `i2c_arb_lost` at once (measured under
the preemptive kernel, [the STM32G0 I2C page](../stm32g0/i2c.md)). The
STM32F4's FMPI2C1, the same register file, does the same; its classic
I2C does not forget the held line at a PE cycle: its START parks at the
speed in force and after a speed change alike, and the arbiter's 20 ms
answers both (measured under the preemptive kernel, [the STM32F4 I2C
page](../stm32f4/i2c.md)). On the SAM C21 a forced IDLE does the same - the
arbiter's `recover()`, or the enable a speed change costs - and the
START into the held line loses its address, `i2c_arb_lost`, or
`i2c_bus_error` where BUSERR stands beside the loss (measured under the
preemptive kernel, [the SAM C21 I2C page](../samc21/i2c.md)).
The CH32V00x's F1-lineage I2C has no clock-low timeout either (a client
holding SCL for 45 ms stands until the arbiter answers at its 20), and
it is the one that answers a wedged SDA by itself: its START into a
busy bus comes back ARLO at once, so the requester reads
`i2c_arb_lost` well before any limit
([the CH32V00x I2C page](../ch32v00x/i2c.md)). The CH32X035's copy of
the block reaches the same reply another way: its START into the held
line goes out and the address loses at its first one bit, and a block
brought up over a line already held keeps its START pending, the
per-bus timeout's case ([the CH32X035 I2C page](../ch32x035/i2c.md)).
The CH32V203's and CH32V303's copy answers a wedged SDA the same way
once: the first START into the held line goes out at the controller's
tick and loses (`i2c_arb_lost` in under 0.4 ms), and a START set after
that loss inside the same hold parks until the per-bus timeout answers
it (measured under the preemptive kernel, [the CH32V203/CH32V303 I2C
page](../ch32vx03/i2c.md)).
The RP2040's DW_apb_i2c has no clock-low timeout either: a client
holding SCL through a read request it never serves stands until the
arbiter answers at its 20 ms ([the RP2040 I2C page](../rp2040/i2c.md)).
So one held SDA has two honest replies, parked and timed out on some
strata and refused by the silicon on others. A requester reads both as
"the bus is not mine", and the recovery ladder ("Not built, noted")
stays the application's. So the timeout is the arbiter's: the one object that
knows a completion is owed, living in the kernel that has time events.

`BusMaster`'s `timeout_ticks` template argument (surfaced here as
`I2cBus`'s) arms a one-shot `TimeEvent` for every tenure that goes
asynchronous. If it matures first, the engine is declared dead:
`Bus::recover()` puts the peripheral back where `start()` is legal
(the AVR host's enable-cycle errata workaround, the SAM's cached
re-init, the G0's PE cycle, which RM0444 32.4.6 offers for exactly
this, "restores the normal operation ... by toggling the PE bit", the
RP2040's block reset through the reset controller, because a host
holding SCL with no STOP in sight never disables); the requester is
answered `i2c_timeout` in its place, and the queue moves on. The races
with the real completion are closed by construction (a sequence number
and a drain state; the whole story is in `util/bus_master.hpp`, staged
deterministically in the host suite).

Three decisions worth their line:

- **Per bus, not per request.** One wedged device starves every client
  of the wire, so the limit is a property of the bus. It must sit
  above legal clock stretching, which is flow control and not a fault,
  so size it to a whole worst-case tenure at the slowest device, and
  convert with `ticks_from_ms<P>()`.
- **The wire stays the application's.** The timeout guarantees only
  that the bus active object and its queue survive to be asked;
  whether to `unstick()`, power-cycle a client or re-probe the bus is
  the recovery ladder below, still policy. After a fault, re-verifying
  the clients one by one is the application's decision.
- **A timeout is not a completion.** The retry `Policy` is never
  consulted, since the engine never spoke.

With `timeout_ticks = 0` (the default) none of this exists: the
generated code is byte-identical to the untimed arbiter's.

## Not built, noted

The arbiter carries the hook a recovery policy would hang from:
`BusMaster`'s `Policy` template argument, whose `on_done(status,
attempt)` can ask for the same request to be started again
(`util/bus_master.hpp`). The concrete I2C ladder below and the
multi-host backoff remain on demand.

- **The recovery ladder.** The mechanical functions exist at engine
  level (`unstick()` clocks SCL up to nine times until a stuck client
  releases SDA, [the AVR TWI page](../avrxt/twi.md)) and the timeout
  above notices the wedge and reports it; the policy connecting them,
  when to unstick, when to retry, when to take the bus out of service,
  is the application's, or a future policy type's, born with the
  device that needs it.
- **Multi-host policy.** The engine reports `i2c_arb_lost` and the
  arbiter passes it to the requester untouched. A bus active object
  that knows it shares the wire (retrying a lost tenure, backing off,
  refusing to start while another host is in the middle of a
  transaction) is not designed.
- 10-bit addressing: the arbiter's descriptor has no shape for it.
- A DAC or sensor client active object with word semantics:
  `util/wire.hpp` already gives the big-endian load and store; the
  MCP47CVB22 driver is the first candidate.

Device-specific facts (the DAC's address strapping, the pull-up value)
live in the page of the board that carries the device
([docs/boards/](../boards/README.md)) and in the header comment of the
application that talks to it, never here.
