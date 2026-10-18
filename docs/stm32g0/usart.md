# USART (STM32G0)

Documents of record: RM0444 Rev 6 - USART ch. 33 (the implementation
tables 183/184, the baud generator 33.5.7, the tolerance tables 188/189,
auto-baud 33.5.9, mute mode 33.5.10, the character match 33.5.11, LIN
33.5.13, synchronous 33.5.14, single-wire 33.5.15, the receiver time-out
33.5.16, smartcard 33.5.17, IrDA 33.5.18, flow control and driver enable
33.5.20, low-power 33.5.21, the registers 33.8), RCC 5.4.21 (the CCIPR
multiplexers) and 5.4.1 (HSIKERON), the EXTI's table 65 (lines 24, 25,
26), the DMAMUX's table 55. Errata ES0548 Rev 3: 2.11.1 STAGED AND
REPRODUCED from 2400 baud to 1 Mbaud, 2.11.2 applied, 2.2.4 STAGED AND
REPRODUCED on a USART wake from the host and from a far chip (see "Bench
findings"). Driver: `stm32g0/usart.hpp` (`Usart<n>` resource
+ `UartTask` and its names `Uart<n, pins, ...>`, `Rs485`, plus
`SyncHost`, `IrdaLink`, `AutoBaud`, `Smartcard`); the instance, vector,
bus-clock, capability and EXTI facts come from
`stm32g0/device_tables.hpp`. The LPUARTs are
[lpuart.md](lpuart.md) - the SAME task over a different resource. Bench
suite: `test_stm32_serial` (17 letters in `z`, 120 verdicts, wireless;
four more outside `z` through `brio stress`; thirteen under the second
all-key `t`, 171 verdicts, against the desk's STM32F446RE running
`usart_peer` - "Against a far board" below - and letter `W` beside
them, outside both all-keys). The instrument for a far
board's suite is this stratum's `usart_peer` ("The peer" below). Family
fixture `test/family_stm32g0/usart.cpp` plus SIXTEEN negatives under
`brio check stm32g0` (an instance nowhere, an instance off a part on
two parts, a pad used twice, two engines on one channel, FIFO mode on a
BASIC instance, a Reserved prescaler code, a Reserved threshold code, a
wake on PCLK, DE without a pad, DE together with RTS, a Uart a dynamic
clock does not list, a receive level with no receive engine, a receive
ring past the counter, a receive threshold under an engine, and the
driven single wire on a full-duplex link).

## What the silicon does

**Six instances on the G0B1, four on the G071, two on the G031 - and
they are not copies of each other.** RM0444 table 183: USART1 is FULL
everywhere; USART2 is FULL on the G071 class and up, BASIC on the G031;
USART3 FULL on the G0B1, BASIC on the G071; USART4..6 BASIC. Table 184
says what that buys: the 8-deep FIFOs, the PRESC prescaler, the
kernel-clock multiplexer and the wake from Stop, smartcard, IrDA, LIN,
auto-baud, the receiver time-out and Modbus are the FULL instances'.
Which instance is which is a POINTER-COMPARISON macro in the device
header (`IS_UART_FIFO_INSTANCE` and nine siblings), which is a perfectly
good expression and a perfectly useless constant one - so
`usart_is_full(n)` in the reserve is a STATED TABLE read off the device
select macro with table 183 cited, the `has_*()` verbs are the header's
own answer at run time, and the bench asks the silicon. Three
authorities, and letter a of the suite prints all three side by side.

**ONE ROW OF TABLE 184 IS NOT THE FULL/BASIC SPLIT.** "Synchronous mode
(Master/Slave)" is given to the FULL column AND the BASIC one and taken
from the LP one; 33.8.3's note on CR2.CLKEN says the same thing
backwards ("if neither synchronous mode nor smartcard mode is supported,
this bit is reserved"); and the device header's `IS_USART_INSTANCE` -
whose comment reads "USART Instances : Synchronous mode" - lists all six
USARTs. So `has_synchronous_mode` is a flag of its own and not another
reading of `is_full`, and the silicon settles it (below).

**Configuration is written with UE clear.** BRR, CR1's frame fields,
CR2, CR3, GTPR and PRESC are all "can only be written when the USART is
disabled". Every resource verb that touches such a field RETURNS FALSE
AND STORES NOTHING while UE stands rather than trusting the silicon to
ignore it - on this family's LPTIM a forbidden write lands anyway
([lptim.md](lptim.md)), so the refusal is the driver's.

**The baud generator has two encodings and OVER8's is not a divisor**
(33.5.7). With OVER8 = 0, BRR = USARTDIV = usart_ker_ck_pres / baud.
With OVER8 = 1, USARTDIV = 2 x that and then BRR[15:4] = USARTDIV[15:4],
BRR[2:0] = USARTDIV[3:0] >> 1, BRR[3] = 0 - the low bit of the fraction
is DROPPED and bit 3 must be clear. USARTDIV >= 16 either way, which
puts the ceiling at kernel/16 and kernel/8. **OVER8 buys range and no
resolution**: with USARTDIV's last bit dropped, the rate is
2 x kernel / (an EVEN USARTDIV) - kernel / n for a whole n, as at
OVER16 - so the two encodings make the same lattice of rates, OVER8
alone reaching n = 8..15 (33.5.7's first example writes 1667 and the
generator divides by 1666). A rate between two rungs of kernel / n is
not reachable by choosing the oversampling: at 64 MHz, 7.5 Mbaud is
n = 8.53, 5 % from either neighbour. And what the divisor divides
is `usart_ker_ck_pres`: the KERNEL clock the CCIPR multiplexer chose,
AFTER the PRESC prescaler - never PCLK by assumption.

**Four kernel clocks, and two of them do not move with SYSCLK.** CCIPR's
USARTnSEL picks PCLK, SYSCLK, HSI16 or LSE for USART1..3 (and for both
LPUARTs); the rest run on PCLK, full stop. A console on HSI16 or LSE
keeps its rate through a `DynamicClock` change and `rebase()` rewrites
NOTHING for it, which is the whole point of naming one.

**TXE is a condition and ORE is a storm.** TXE reads 1 whenever the data
register is empty, so its interrupt is armed only while the ring holds
something and disarmed from the handler when it runs dry. ORE raises the
interrupt whenever RXNEIE is set (33.8.9) and is cleared ONLY through
ICR.ORECF: a handler that reads RDR and leaves ORE standing re-enters
for ever. **WUF IS THE SAME SHAPE ONE FLAG ALONG**: with UESM and WUFIE
set, a WUF nobody clears makes the vector re-enter until the watchdog
reboots the board (IPSR 44 with ISR bit 20 standing). Every error flag
has its ICR twin and every handler here clears what it can see.

**RDR holds the last GOOD byte when ORE is set**; FE/NE/PE belong to the
byte in RDR, so a framed or parity-failed byte is dropped precisely and
a noisy one is kept and counted. **In FIFO mode 33.5.4 stores those
three flags WITH EACH ENTRY**, so a draining handler must read ISR
BEFORE every RDR or the attribution slides by one and a good byte
inherits its neighbour's framing error. **And the flags are cleared
BEFORE the RDR read that pops their entry**: a flag the next entry
carries too is the same ISR bit, still standing, and a clear after the
pop takes the next entry's with it - measured from a foreign sender, six
frames of the wrong parity drained from the RXFIFO read three PE with
the clear after the read and six with it before (letter `4`); the task
clears first.

**The FIFOs are the transport's default, where the instance has them.**
A FULL USART and every LPUART carry an 8-deep transmit FIFO and an
8-deep receive FIFO behind CR1.FIFOEN (33.5.4, table 184), and three
conditions pace the transmit side instead of one: TXFNF (not full), TXFE
(empty) and TXFT, the threshold TXFTCFG names - code 101 being "TXFIFO
becomes empty" (33.8.4), which rises as the shift register takes the
last entry, so a handler refilling all eight places has a whole frame
before the wire would idle. All three are conditions, like TXE, so the
interrupt is still armed only while the ring holds something. TC changes
meaning with them: it waits for the TXFIFO AND the shift register
(33.5.5, step 8), up to nine frames after the last byte left the ring
where the one-register transmitter left two. Two exclusions come with
the bit: 33.8.1's note forbids it in LIN and IrDA mode, and 33.5.21 makes
a wake from Stop on an ADDRESS MATCH need mute mode once the FIFO is on.

**The receiver has a level, a time-out and an idle line, and no flag
needs RDR to clear it.** RXFT rises when the RXFIFO holds the RXFTCFG
level (33.8.10); RTOF when the line has been silent for RTOR.RTO bit
times past the last stop bit (33.5.16, a FULL instance's), within two
samples of it; IDLE after a whole idle frame, once a burst - 33.8.10's
note keeps it from rising again until a character has. Every one of
them, and FE, NE, PE and ORE, is cleared by its ICR bit (33.8.11): on
this block no clear is a read sequence, so a receive channel is never
robbed of a byte by an error being cleared. Under DMAR, EIE raises the
vector on FE, NE and ORE and PEIE on PE, and with CR3.DDRE clear the
character an error belongs to is NOT transferred and the next good one
is (33.8.4).

**TE sends an idle frame first** (33.5.5), which is why the pads are
handed to the peripheral BEFORE UE/TE are raised - and why the RX pad
gets a pull-up, so an unconnected line reads idle instead of noise.

**Single-wire half duplex is this family's loop-back.** There is no
LBME here; CR3.HDSEL connects TX and RX internally, drops the RX pin and
RELEASES the TX pin whenever nothing is transmitted, so the pad is
alternate-function OPEN DRAIN with a pull-up - 33.5.15 asks for an
external one and the internal 40 k serves - and the instance hears every
byte it sends. The whole bench suite is built on it. Where the line has
no other driver - the loop of a port that hears itself, or a link whose
far end only listens - the option `single_wire_push_pull` drives the pad
push-pull while a frame goes out, the pull-up holding it between frames:
the open drain rises on the pull-up alone, an RC a link wire's load
slows further (the baud generator's findings below).

**One pad, three jobs.** `USARTn_RTS_DE_CK` is the flow-control RTS, the
RS-485 driver enable AND the synchronous clock (DS13560's tables; ch. 33
never says so). An instance does one of the three, never two - which is
a compile-time refusal in the task.

## Types and verbs

- `Usart<n>` - the instance. `regs()`, `irq()`, `bus_clock(on)`, the
  capability constants (`is_full`, `has_prescaler`, `has_fifo_mode`,
  `has_synchronous_mode`, `has_lin_mode`, `has_receiver_timeout`,
  `fifo_depth`, `exti_line`, `has_clock_select`) and the header's own
  run-time twins (`has_fifo()`, `has_autobaud()`, `has_lin()`,
  `has_irda()`, `has_smartcard()`, `has_half_duplex()`,
  `has_flow_control()`, `has_driver_enable()`, `has_wake_from_stop()`,
  `has_synchronous()`).
- The configuration verbs, every one enable-protected:
  `kernel_clock(UsartClock)`, `configure(UartFormat, brr)`,
  `stop_bits(UartStop)` (the 0.5/1.5 codes `UartFormat` cannot name),
  `oversampling(over8)`, `prescaler(UsartPrescaler)` (the twelve codes;
  a Reserved one is REFUSED, because 33.8.14's note says the silicon
  turns it into divide-by-256), `fifo(bool)` (turning it on REFUSED in
  LIN or IrDA mode, and `lin()` / `irda()` refusing with it on - 33.8.1),
  `fifo_thresholds(rx, tx)`, `swap`, `invert(tx, rx, data)`,
  `msb_first`, `half_duplex`, `one_bit_sampling`, `overrun_disable`,
  `flow_control(rts, cts)`, `driver_enable(DriverEnableConfig)`,
  `mute_mode(MuteConfig)`, `character_match(ch)`,
  `receiver_timeout_enable` + `receiver_timeout(bits)` (the ENABLE is
  enable-protected and the VALUE is not - 33.8.7 lets RTOR move on the
  fly, so they are two verbs), `block_length(blen)`,
  `lin(LinConfig)`, `synchronous(SyncConfig)`,
  `smartcard(SmartcardConfig)`, `irda(IrdaConfig)`,
  `auto_baud(AutoBaudMode)`, `wake_from_stop(UsartWakeSource)` +
  `wake_line(on)`, `guard_time(gt)`, and an `*_off()` for each mode.
  Each of the five modes REFUSES the company its own section forbids
  rather than writing a combination the silicon does not define.
- Live verbs: `enable`, `request(UsartRequest)` (RQR's five write-only
  strobes), `send_break()`, `auto_baud_restart()`, `status()`,
  `flag(mask)`, `clear_flags(icr_mask)`, `read_data`/`read_word`,
  `write_data`/`write_word`, `stop_retries()` (33.8.4's one documented
  escape from the enable rule), the interrupt enables under the guard,
  `dma_transmit`/`dma_receive` (legal with UE set: 33.8.4 gives DMAT and
  DMAR no "only when UE = 0" clause), `rx_request_restart()` (DMAR
  dropped and raised again, the receive engine's kick below),
  `dma_rx_request()`/`dma_tx_request()` (table 55's numbers, published by
  the peripheral that owns them), `reset()`.
- `UsartFlag` / `UsartClear` / `UsartInterrupt` - every ISR bit by BOTH
  names the register description gives it (`rxne` and `rxfne`, `txe` and
  `txfnf`), every ICR twin, every CR1 enable.
- `UartOptions` - the constexpr struct that is the task's LAST template
  argument: `kernel_clock`, `prescaler`, `over8`, `fifo` +
  `rx_threshold`/`tx_threshold`, `swap`, `invert_tx`/`invert_rx`/
  `invert_data`, `msb_first`, `half_duplex`, `tx_speed` (the TX pad's
  slew class, low by default), `single_wire_push_pull` (the half-duplex
  pad driven while a frame goes out, refused at compile time without
  `half_duplex`), `one_bit`,
  `overrun_disable`, `driver_enable` + `de_pin`/`de_assertion`/
  `de_deassertion`/`de_active_low`, `rts`/`cts` + their pads,
  `wake_from_stop`, and `rx_priority` - the receive engine's level in
  the DMA arbitration, `very_high` by default and refused on a port with
  no receive engine (docs/design/dma.md). Makers: `uart_half_duplex()`,
  `uart_with_driver_enable()`. `fifo` is a `UartFifo`, THREE states a
  bool converts to: stated on (`.fifo = true`, refused at compile time
  on an instance with no FIFO), stated off (`.fifo = false`), and
  unstated - the default - in which the task decides. `tx_threshold`
  defaults to `full_or_empty` (the transmitter refills an EMPTY FIFO),
  and `none` makes it ride TXFNF; `rx_threshold` defaults to `half` -
  the receiver paced by RXFT, its tail by the receiver time-out where
  the instance has one and by IDLE where it has not (an LPUART) - and
  `none` keeps RXFNE, a character an entry; under a receive engine the
  receive threshold is the task's (seven eighths, the FIFO wedge's watch)
  and a stated `rx_threshold` is refused; `rx_timeout` is RTOR.RTO in
  bit times, 10 by default.
- `Uart<n, pins, rx_size = 64, tx_size = 256, TxEngine = NoDmaEngine,
  RxEngine = NoDmaEngine, opts = {}, Placement = InFlash>` - where the
  two interrupt bodies run from (util/placement.hpp; "Where the hot
  paths run from" below), and `init(clock, baud, format)`,
  `isr()`, `dma_isr()`, `harvest()`, `write_byte`/`write_bulk`,
  `read_byte`/`read_bulk`, `read_span`/`consume` (the receive run in
  place; with a receive engine `consume()` answers whether the run was
  intact), `rx_skips` (the gaps in the stream the receive ring hands
  out, never cleared - util/serial_port.hpp's epoch: without an engine
  the SkipRing's skips, one at the consumer's look after `isr()` reported
  a byte refused by a full ring, dropped for FE or PE, or an overrun -
  the look discarding what the ring held, so where in the entry's drain
  the loss fell does not matter; with one the ring's skips plus an ORE,
  an FE or a PE the channel never took, and a restart after a transfer
  error),
  `rx_pending`, `tx_idle` (the wire's: nothing queued, no block in
  flight, TC set), `rebase(hz)`,
  `set_baud(hz, baud)`, `actual_baud`, `can_baud`, `min_hz_for`,
  `kernel_hz<Clock>()`, the counters (`rx_overruns`, `hw_overruns`,
  `frame_errors`, `parity_errors`, `noise_errors`, `dma_faults`,
  `wakes`, `rx_kicks` - eight bits each, SATURATING at 255: a storm of
  errors never reads back as a few), `clear_errors`, `release()`, and `fifo_mode` - the FIFO
  decision as the instantiation resolved it: the option where it is
  stated, otherwise ON where the instance has the FIFO
  (`has_fifo_mode`: a FULL USART, every LPUART) unless `wake_from_stop`
  is an address match, and `rx_paced` and `rx_tail` - the receiver's
  pace and the flag that ends a burst, as the instantiation resolved
  them. The public surface is
  IDENTICAL to avrxt's and samc21's, which is what lets
  `util/serial_port.hpp` and `print()` compile here untouched.
- `Rs485<n, pins, de_pin, assertion, deassertion, ..., Placement>` - the
  same task with the driver enable filled in. **`OneWire` is NOT a task here**:
  33.5.15 is a bit, so single-wire is `Uart` with
  `uart_half_duplex()`, where the AVR stratum spells a task.
- `SyncHost<n, pins, ck>`, `IrdaLink<n, pins>`, `AutoBaud<n>`,
  `Smartcard<n, pad, ck>` - POLLED tasks over the resource, with no ring
  and no interrupt of their own: they are protocol shapes used a few
  characters at a time, not byte transports. A ring-fed version of any
  of them is a `Uart` with that shape's own `configure()` call.
- `usart_brr(hz, baud)` / `usart_brr_over8(hz, baud)` (SIBLING VERBS,
  not one verb with a bool - the measured reason is under "The options
  cost nothing"), `usart_actual_baud` / `usart_actual_baud_over8`,
  `usart_min_hz` / `usart_min_hz_over8`, `usart_kernel_hz(ker, presc)`,
  `usart_kernel_clock_hz<Clock>(UsartClock)` - constexpr, and the
  fixture pins the chapter's own two examples IN BOTH OVERSAMPLINGS.

## How to use it

The console:

```cpp
constexpr brio::UartPins console_pins{
    .tx = {'A', 2, brio::PinFunction::af1},   // USART2_TX, DS13560 table 13
    .rx = {'A', 3, brio::PinFunction::af1},   // USART2_RX
};
using Serial = brio::Uart<2, console_pins>;
constexpr Serial serial;

extern "C" void USART2_LPUART2_IRQHandler() {     // the SHARED line's name
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}

Serial::init(clock, 115200);                      // after SysClock::init()
brio::print(serial, "hello", brio::crlf);
```

USART2 is a FULL instance on the G071 and G0B1 classes, so this console
runs in FIFO mode: a print costs one interrupt per eight characters.
On the G031 class USART2 is BASIC and the same line builds the
one-register transport (`Serial::fifo_mode` says which). The same
console with its interrupt body in SRAM, the vector left where it is -
436 bytes of SRAM, a fourteenth off every entry:

```cpp
using Serial = brio::Uart<2, console_pins, 64, 256, brio::NoDmaEngine, brio::NoDmaEngine,
                          brio::UartOptions{}, brio::InRam>;
```

A one-wire link on a single pad, a port that keeps its rate through a
clock change, and one that keeps the FIFO OFF - one character per
interrupt, the shape a program asks for by name:

```cpp
constexpr brio::UartOptions one_wire = brio::uart_half_duplex();
using Bus = brio::Uart<1, u1_pins, 64, 64, brio::NoDmaEngine,
                       brio::NoDmaEngine, one_wire>;

constexpr brio::UartOptions on_hsi{.kernel_clock = brio::UsartClock::hsi16};
using Steady = brio::Uart<1, u1_pins, 128, 512, brio::NoDmaEngine,
                          brio::NoDmaEngine, on_hsi>;

constexpr brio::UartOptions one_register{.fifo = false};
using Plain = brio::Uart<1, u1_pins, 64, 256, brio::NoDmaEngine,
                         brio::NoDmaEngine, one_register>;
```

RS-485, whose DE rides the RTS pad and whose timings are SAMPLE times -
1/16 of a bit at OVER8 = 0 and 1/8 at OVER8 = 1, which is the one thing
about the feature that is easy to get wrong:

```cpp
constexpr brio::PinSel de{'B', 3, brio::PinFunction::af4};  // USART1_RTS_DE_CK
using Link = brio::Rs485<1, u1_pins, de, /*DEAT*/ 16, /*DEDT*/ 16>;
```

A port that wakes the core out of Stop. The kernel clock must be one
that survives Stop and the task refuses PCLK and SYSCLK at compile time;
the obligations it CANNOT enforce are 33.5.21's and the caller's - no
transfer ongoing at entry, REACK checked after the enable, DMA reception
disabled first:

```cpp
constexpr brio::UartOptions waker{
    .kernel_clock = brio::UsartClock::hsi16,
    .wake_from_stop = brio::UsartWakeSource::start_bit,
};
```

## What the default buys, and what an option costs

`UartOptions` is ONE trailing NTTP with a default, and every member of it
is `if constexpr`-ed, so a feature a program does not name compiles to
nothing.

**The default is the FIFO, where the instance has one.** With `fifo`
unstated the task turns it on (`fifo_mode`), the receiver rides RXFT
at half the FIFO with the time-out for the tail ("The receive side"
below) and the transmitter rides TXFT at "TXFIFO becomes empty". Each refill reads the ring as a RUN - the
contiguous characters `read_span()` hands over, stored while TXFNF
stands and released with one `consume()` - so the release image's
per-character loop is eight instructions (`ldr` ISR, `tst`, `bne`,
`ldrb`, `adds`, `str` TDR, `cmp`, `bne`), and a ring found empty is
disarmed in the same entry. It costs 124 bytes of flash in the console
image over `.fifo = false`. Measured on the Nucleo-G0B1RE at 115200 (`bench_stm32` letter
`p`, a print of 4096 bytes, wire-bound either way at `x` = 1.00):

| transmitter | USART2 entries | cycles an entry | cycles a character |
|---|---|---|---|
| `.fifo = false`, TXE | 4098 | 109 | 109 |
| FIFO, `tx_threshold = none`, TXFNF | 4096 | 183 | 183 |
| the default, FIFO, TXFT at empty | 515 | 295 | 37.1 |
| the same, the task `InRam` | 515 | 275 | 34.5 |

(stamps included: the bench meter's own 42 cycles an entry.) TXFNF
pacing is the trap the middle row shows: under a long print the FIFO
stays full and every character that leaves frees one place and costs one
entry, each carrying the run's bookkeeping for a single character - the
FIFO bought nothing and the entry got dearer. The 515 are 512 refills of
eight and three at the print's start: the first byte goes
straight through the idle FIFO into the shift register and the vector
is entered once more to find the transmitter disarmed, and the second,
pushed before the first has left, goes out alone - 2, 5, 35 and 515
entries for prints of 1, 16, 256 and 4096 bytes. The receiver gains
nine characters of slack where it had one. The transmitter's slack is
one frame per refill - a handler later than that leaves the wire idle
for the difference, which costs time and no data - and `tx_threshold`
buys more: a code below 101 refills before the FIFO runs dry, and so
more often, and `none` keeps up to seven characters queued at the price
of an entry per character.

Two shapes keep the baud arithmetic from moving images that do not use
it:

- `usart_brr()` and `usart_brr_over8()` are SIBLING VERBS. Spelling them
  as one verb with a `bool over8 = false` third argument costs forty
  bytes in an image that never passes it, although the folded code for
  `false` is identical.
- The task keeps a `plain` constant for the default arrangement (PCLK,
  divide-by-1, no OVER8) and names the plain expression under it,
  because folding `hz / usart_prescaler_divisor(div1)` to `hz` gives the
  same value and NOT the same code.

## The receive side

What chapter 33 offers a byte transport's RECEIVER, item by item:

| item (RM0444) | used? |
|---|---|
| RXNE / RXFNE, a character waiting (33.8.10) | the one-register receiver's pace (a BASIC instance), and the FIFO's under `rx_threshold = none`; the request a receive channel answers |
| the RXFIFO, eight deep, its flags per entry (33.5.4) | on wherever the instance has it; the drain reads ISR before every RDR, so each entry's FE/NE/PE are its own |
| RXFT at RXFTCFG's level (33.8.4) | under a receive engine, THE WEDGE'S WATCH at code 100, seven eighths: the FIFO one entry and the shift register short of an overrun, where the kick still loses nothing (the bench finding below); without one, THE PACE of the FIFO receiver, at code 010 - half the FIFO: one entry drains four characters or more, and a handler five frames late loses nothing (four places and the shift register); three quarters would save a twelfth of an entry a character and leave three frames of slack, which the top rate (a frame is 320 cycles at 2 Mbaud) does not afford beside the image's longest handler |
| the receiver time-out, RTOF after RTOR.RTO bit times (33.5.16) | the TAIL of the paced receiver on a FULL instance, at ten bit times: the characters below the level are delivered within two frames of the last stop bit, and a burst costs one entry more |
| IDLE, one idle frame once a burst (33.8.10) | the tail where the instance has no time-out (an LPUART), and THE EDGE OF A RECEIVE ENGINE on every instance - a channel can sit on a BASIC USART or an LPUART, and IDLE is the flag table 184 gives them all |
| EIE, PEIE under DMAR (33.8.1, 33.8.4) | armed with a receive engine: every error enters the vector once and is counted there, exactly |
| DDRE (33.8.4) | left clear: a character with an error is not transferred, the next good one is, the channel keeps running |
| the ICR clears (33.8.11) | every flag, errors included: no clear reads RDR, so none takes a byte from under a channel |
| the character match, CMF (33.5.11) | declined: an end-of-line edge would serve a line protocol and nothing else; the time-out serves every stream, and the console's lines are the line assembler's |
| OVRDIS (33.8.4) | declined as a default: it bypasses the RXFIFO (measured, letter e) and silences the overrun it would hide |
| a channel's half and full marks, circular (RM0444 10.4.6) | the engine's edge on a stream with no silence ([dma.md](dma.md)) |
| the single-wire loop, HDSEL (33.5.15) | the bench's loop: the receiver hears every frame its transmitter sends |

**THE EDGE COMES FROM A VECTOR.** Without an engine `isr()` reports the
ring's empty -> non-empty transition across one entry - a level or the
tail. With one, the line's IDLE enters `isr()` and the ring's half and
full marks enter `dma_isr()`, and both run `harvest()`: the receive
errors read and cleared through ICR, a channel a transfer error stopped
started again, and the EDGE - true when the ring holds bytes and the
consumer has looked and found it empty since the last true, the look
being the ring's `waiting()`, which writes nothing and so is a vector's
to ask while the consumer holds a run. The consumer sets its "looked"
flag before it looks and clears it when the look found bytes, so a byte
landing between the two is reported twice at worst and never not at
all. `harvest()` stays public and idempotent; no owner polls it.

**THE COST**, counted in the release listing and measured with
`bench_stm32` letter `u` on the Nucleo-G0B1RE at 64 MHz (USART1's single
wire, the transmitter on its engine so USART1's vector serves the
receiver alone; `isr` includes the meter's 118 cycles an entry):

| receive of 256 bytes | entries | cycles an entry | cycles a byte |
|---|---|---|---|
| RXFNE, a character an entry (before) | 256 | 212 | 212 |
| RXFT at half + the time-out (now), 115200 and 1 Mbaud | 64 | 385 | 96 |
| the same at 2 Mbaud | 52 | 442 | 90 |
| the receive engine (the DMA vector, a block of the transmitter's included) | 3 | 275 | 3.2 |

The 1 and 2 Mbaud rows, and the edge's latency at 1 Mbaud below, are the
loop's with PA9 unloaded: on the paired desk the open-drain loop carries
nothing above 460800 (the ceiling above), and the 115200 rows read the
same there (64 entries of 386 cycles, the edge 1.06 frames).

The paced drain is 23 instructions a character with no call in a
console's vector - the ISR load and its test, the RDR load and the
ring's push inline; the error flags are tested on the ISR word in hand
and served off the character's path, a loss reported to the ring there -
and an entry adds the prologue, the tail's ICR store and the edge's two
ring loads. The bench's USART1 vector serves four transports in one
function, and there gcc reloads the ring's address from the literal pool
at every character: 25 instructions, the table's 96 cycles a byte; the
same receiver's body in a function of its own reads 94, beside 95 for a
plain Ring's (one session, the call inside the meter). The engine's edge costs one entry
of `isr()` (an ISR load, the IDLE clear) and a call of `harvest()`.

**THE EDGE'S LATENCY**, from the burst's last stop bit (TC rising) to
the vector's true, on a burst of 17 - one past four levels: the paced
receiver 1.06 frames at 115200 (the time-out's ten bits, its two
samples and the entry), and at 1 Mbaud the tail taken by the level's
own entry before the stop bit has ended; the engine's IDLE 1.06 frames
at 115200 and 1.40 at 1 Mbaud. Before this shape the engine's edge was
its owner's poll - 4.4 and 15 frames on a 1 ms tick - and the
one-register receiver's every character.

**`tx_idle()` is the wire's**: the ring empty, no transmit block in
flight, and TC - which the handler's TDR stores clear and a transmit
block clears through ICR when it starts (33.5.19's step 6), so a TC
left by the previous block never answers for this one.

**THE COPY INTO THE RING.** `write_bulk()` copies a run of 16 bytes or
more whose two ends share their word alignment with the runtime's
`memcpy` and anything else with its byte loop: measured with letter `u`,
the loop is 4 cycles and 9 a byte, `memcpy` 70 and under one a byte
co-aligned - the two even near 13 - and on ends aligned differently
`memcpy`'s own byte path, 62 cycles behind the loop at every length.

**AGAINST THE VENDOR**, ST's HAL v1.4.7 on the same board and loop in a
scratch program (brio's crt, clock and console around it): its FIFO
receive (`HAL_UART_Receive_IT` with `HAL_UARTEx_EnableFifoMode` at half,
`UART_RxISR_8BIT_FIFOEN`) is 107 cycles a byte on 256 bytes against
brio's 96 - and it needs the length up front: its tail is taken one
character an entry once fewer than a level remain. Its character-at-a-
time receive is 184 cycles a byte and LOSES BYTES at 2 Mbaud (130 of
256 wrong), where this transport's lost none at any rate.
`HAL_UARTEx_ReceiveToIdle_DMA` - IDLE, the channel's half and full,
EIE and PEIE, the shape adopted here - tells its burst 1.04 frames after
the last stop bit at 115200 and 1.34 at 1 Mbaud (its transmit-complete
callback the reference), brio's engine 1.06 and 1.40: the same edge,
within the instrument's own reach.

## The two optional DMA engine slots

`Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine, opts>` takes a
`stm32g0/dma.hpp` `DmaTxEngine` and/or `DmaRxEngine`; both default to
`NoDmaEngine`, which is a TAG and not a base class - `present` is all the
task asks about and it asks with `if constexpr`. It lives in
`stm32g0/dma_engine.hpp`, a file of two lines and a paragraph, and NOT
in `dma.hpp`: that separation is the whole point of an optional slot,
since a driver with one must not include the DMA driver or every program
with a console (or a bus) would carry the controller. Two headers need
the same tag - this one and `stm32g0/spi.hpp`, which has two slots of
its own - and neither is the other's natural home, so it belongs to
neither. `uart_engines_distinct()` lives here.

- **CR3.DMAT / CR3.DMAR** are set in `init()` before the enable, and the
  matching INTERRUPT is NOT armed: the request and the interrupt are the
  same condition. Neither bit is UE-protected (33.8.4), which is what
  lets the vector drop and raise DMAR with the instance running.
- **THE LEVELS ARE THE RULE'S** (docs/design/dma.md): the receive engine
  arms at `UartOptions::rx_priority`, `very_high` unless the program
  ranks another channel above its ring - the FIFO and the shift register
  hold nine characters, and a channel starved past them loses the next
  one - and the transmit engine at `high`, a starved transmit only
  leaving the line idle. RM0444 10.4.4 alternates a memory-to-memory
  channel with any other requester, so one copy never starves the ring;
  two copies alternate with each other and do, and the ring at
  `very_high` beside the same pair loses nothing (`test_stm32_dma`
  letter `p`, below).
- **`dma_isr()`** is the body of whichever channel vector the engines
  report on; each engine reads only its own channel's flags. On the
  receive channel the ring's HALF and FULL marks arrive: the full one is
  a LAP, counted and nothing more, and either runs `harvest()`, whose
  edge `dma_isr()` returns - a stream with no silence is told twice a
  lap (letter `p` of `test_stm32_serial`: 200 bytes at 1 Mbaud across a
  64-byte ring read on the edges alone, none lost). A transmit
  completion is no edge.
- **THE RECEIVE ENGINE RUNS A RING, NOT RUNS.** `init()` arms it in its
  circular shape over the whole receive ring's storage
  ([dma.md](dma.md), "The circular receive"): the channel writes the
  storage lap after lap and is never re-armed, the ring's producer index
  is its count register, and the receive ring is the consumer half of
  that, `util/ring.hpp`'s `HardwareRing`. A byte that lands is readable
  at once through `read_span()`, `read_byte()` and `read_bulk()`; a
  burst longer than the run to the end of the storage wraps with no CPU;
  and nothing is lost between runs, there being none (letter `o` of
  `test_stm32_dma`: 60 bytes landing with the consumer away, all 60 read
  back, where a one-shot run to the storage's end delivered 22 and an
  ORE). The ring is ONE circular block, so `rx_size` is refused past
  32768 where the transport is named (CNDTR counts 65535).
- **What the consumer did not keep up with** - a lap written over its
  unread bytes, or a run written over while it was held - is counted by
  the ring and skipped, and `rx_overruns()` reports it (without an engine
  it counts a byte a full ring refused); `read_bulk()` does not count a
  run that came back torn, and `consume()` answers false for one.
- **`harvest()`** is what the two vectors run on a receive event, and
  nothing is published in it: it reads and clears the receive errors
  through ICR, starts again a channel a transfer error stopped (the ring
  cleared with it, its unread bytes gone with the abandoned lap, which
  `dma_faults()` counts), and reports the EDGE a ring with no publish
  step cannot give by itself ("The receive side" above). It stays a
  public verb and is idempotent. It costs 71 cycles with the consumer
  told and still reading and 106 on a drained ring (letter `o`).
- **The errors are counted one by one**: EIE and PEIE enter `isr()` on
  each, the character it belongs to is not transferred (DDRE clear), and
  no clear reads RDR - letter `p`: 64 slots with 4 breaks, 60 bytes
  delivered intact and in order, the frame-error counter 4, one entry
  each. What is traded away is the per-character attribution of a
  noise error: NE keeps its character, and under the engine nobody
  knows which. `isr()` never touches RDR where a receive engine is named:
  the channel owns RXNE.
- **`write_byte()` still nudges on a refusal** when a TX engine is
  present, because `print()` answers a false by trying for ever.
- **A block's start masks the CLAIM and nothing else.** The thread and
  the transmit channel's completion both start blocks, so `pump_tx()`
  takes the engine's busy flag (`DmaTxEngine::claim()`) under the
  platform's guard - seven instructions - and programs the channel
  unmasked: a free engine has no block in flight, so no completion,
  error or `consume()` can run under the five stores
  ([dma.md](dma.md), "Two moments"). A claim that finds the ring empty
  is given back.
- **In FIFO mode** - the default where the instance has one - the two
  requests are TXFNF and RXFNE (33.5.19's two notes): still one request
  a character, with the FIFO as slack in front of each channel, and the
  transmit threshold written but never armed. THE RECEIVE FIFO STAYS ON
  UNDER THE ENGINE, WITH A KICK: `isr()` drops and raises DMAR when an
  entry for the line's IDLE, a receive error or RXFT at seven eighths
  finds RXFNE still set - the cure for this silicon's FIFO wedge, the
  bench finding below - and `rx_kicks()` counts it. Without the FIFO
  (`.fifo = false`) there is nothing to wedge and nothing to watch.
- **`release()` stops the instance before its channels**: 10.4.5 stops a
  circular transfer by stopping the peripheral's requests first, a ring
  having no idle moment of its own.

## Bench findings

`test_stm32_serial` on the Nucleo-G0B1RE, WIRELESS. Two facts make
the whole chapter stageable on one board: CR3.HDSEL turns any instance
into its own loop-back, and a pad handed to the RX alternate function is
an INPUT whose internal pull the CPU can move in under a microsecond -
so at 2400 baud (a 417 us bit) software puts an ARBITRARY frame on the
receive line. TIM2 free-running at 64 MHz is the ruler, and a DMAMUX
request generator counting EXTI edges is the no-CPU witness.

### The instances, three authorities and the silicon

FIFOEN and PRESC written straight into a disabled instance, on all eight:

- **The FIFO split is table 183's exactly.** FIFOEN sticks on USART1..3
  and both LPUARTs and is DROPPED by USART4..6 - the manual, the device
  header and the silicon agreeing on every instance of the part.
- **PRESC[3:0] TAKES the value and reads it back on ALL SIX USARTs**,
  including the three table 184 says have no prescaler. Whether it
  DIVIDES is another question, and the answer is the letter's own
  finding: on USART1 nine zero bits last 90 us at /1 and 1440 us at /16
  - exactly sixteen times - while **on USART4 the frame at /16 NEVER
  LEFT THE PAD AT ALL** (90 us at /1, nothing at /16, TC never rising).
  Table 184 says the prescaler is not there; the silicon says writing it
  STOPS THE TRANSMITTER. So `prescaler()` refusing on a BASIC instance
  is the only safe reading, and trusting the readback would be a
  transmitter that goes silent for no visible reason. **ES0548 2.11.2**
  is the documentation erratum about exactly this per-instance split;
  RM0444 Rev 6's own table 184 carries it, and the silicon carries it in
  BEHAVIOUR and not in the register's readback.
- **CR2.CLKEN sticks on all six USARTs and on NEITHER LPUART** - the
  third authority on the row of table 184 that is not the FULL/BASIC
  split. The synchronous CK belongs to every USART of this part.
- The kernel-clock multiplexer is USART1..3's and both LPUARTs'; an
  instance without one answers true for PCLK and false for anything
  else instead of writing a field that does not exist. The vectors and
  the wake lines are this part's shared ones: USART2 with LPUART2,
  USART3..6 with LPUART1, EXTI 25/26/24 for USART1/2/3 and 28/35 for the
  LPUARTs.

### The instrument, and every frame format

PA9 follows its own pull as an input AND under the USART1_TX OPEN-DRAIN
alternate function - which is what makes 33.5.15's external pull-up an
internal one here (the SAM found that a DRIVING function takes the pull
away; an open-drain one released high does not). 0xA5 out on PA9 and
0xA5 back, with no wire on the board. **All thirteen frame formats** -
7/8/9 data bits x none/even/odd parity x 1/2 stop bits, four values each
- byte-exact, with zero receive errors. SBKRQ on the plain loop lands as
a zero character carrying a framing error.

### The baud generator

**Both of 33.5.7's examples land in BRR bit for bit in both
oversamplings**: 9600 at 8 MHz is 0x341 and 0x681, 921600 at 48 MHz is
0x34 and 0x64. OVER8 = 1 at 115200 carried 32 of 32 bytes on the loop
with BRR 0x453. **All twelve prescaler codes carry 9600 baud** - one
line rate, twelve kernel rates from /1 to /256. **The generator's floor
is USARTDIV 16 in both oversamplings**: 4 Mbaud at OVER8 = 0 and 8 Mbaud
at OVER8 = 1 from a 64 MHz kernel both give BRR 16, and 5 Mbaud / 9
Mbaud are refused by the constexpr arithmetic. **The open-drain loop is
byte-exact to 460800 on the paired desk** (16 of 16 at 115200, 230400
and 460800; 0 of 16 at 921600, 1 M, 2 M and 4 M), and what fails above
it is the rise time of the pad's own 40 k pull-up into the LINK WIRE PA9
carries to the far board's receive pad - some 15 cm of jumper and a
second chip's input, held hi-Z by that board's dark console. **THE
DRIVEN LOOP** (the pad push-pull at very high speed while a frame goes
out, `single_wire_push_pull`'s shape) **is byte-exact at every rung,
115200 to 4 Mbaud, and at OVER8's 8 Mbaud**, 16 of 16 each, on the same
loaded wire: the generator's ceiling, the wire no longer in it - which
is why letter `p`, `test_stm32_dma` letter `o` and `bench_stm32` letter
`u` run their 1 and 2 Mbaud streams on a driven single wire. The open
drain's ceiling stays a figure of the desk. The generator is not in
either: every rung runs at its rate on the wire (below).

### The rate on the wire, timed

A single wire shares ONE divisor between the transmitter and the
receiver it loops into, so a wrong divisor is a slower or a faster loop
and every byte still comes back - the STM32F4's OVER8 ran at half its
rate through every self-looped suite until a letter timed the wire. So
every letter of `test_stm32_serial` that loops or sets a rate on a
transmitter times a run of its frames on TIM2 (64 MHz, one count a
cycle) from the first store into TDR to TC - the last stop bit's end -
against the frames' time AT THE RATE ASKED, and judges the reading in
thousandths: -1 % to +3 %, never the rate the divisor produced. A run is
a hundredth of a second of frames, 16 at least and 1024 at most. The
letters whose rate is a RECEIVER's (`f`, `g`, `i` and the receive halves
of `h`, `k` and `l`) are fed by the bit-banged line, which TIM2 paces at
the rate asked, so their decoding is the check. TIM2 and every kernel
clock but the LSE derive from HSI16 on these boards (no HSE fitted): the
oscillator's error cancels and what is read is the divisor; against the
32768 Hz crystal the reading carries HSI16's trim too.

| letter | what runs | rate asked | STM32G0B1RE | STM32G071RB |
|---|---|---|---|---|
| `b` | 256 frames of each of the 13 formats, each against its own length | 115200 | 1000 .. 1000 | 1000 .. 1000 |
| `c` | OVER8 on the loop | 115200 | 999 | 999 |
| `c` | the twelve PRESC codes, 96 frames each | 9600 | 998 .. 1000 | 998 .. 1000 |
| `c` | the OVER16 ladder, 1024 frames a rung | 115200 .. 4 M | 1000, 993 at 921600 | 1000, 993 at 921600 |
| `c` | OVER8 at its ceiling, through the FIFO | 8 M | 1000 | 1000 |
| `d` | the console's own line on HSI16, SYSCLK, PCLK (64 frames) | 115200 | 1001, 1001, 1001 | 1001, 1001, 1001 |
| `d` | USART1 on the LSE, OVER16 and OVER8 | 2048, 4096 | 1004, 1004 | 1002, 1002 |
| `e` | 1024 frames through the transmit FIFO | 115200 | 1000 | 1000 |
| `e` | the 256-byte block through the interrupt transport, without and with the FIFO | 115200 | 1000, 1001 | 1000, 1001 |
| `h` | LIN mode's frames | 2400 | 1000 | 1000 |
| `j` | smartcard characters of twelve baud periods (33.5.17), /256 kernel | 1200 | 998 | 998 |
| `k` | SIR frames, normal and low-power (/16 kernel) | 1200 | 999, 999 | 999, 999 |
| `l` | the swapped wire | 115200 | 1000 | 1000 |
| `m` | the synchronous master's frames, three arrangements | 9600 | 1000 .. 1000 | 1000 .. 1000 |
| `o` | the IRTIM envelope's frames | 1200 | 999 | 999 |
| `p` | the paced transport's transmit engine, 1024 frames | 115200 | 1001 | 1001 |
| `p` | both engines, 1024 frames | 1 M | 1001 | 1001 |
| `q` | the first to the last of eight start bits on the pad (EXTI) | 9600 | 997 (interrupt), 999 (engine) | 997, 999 |

Every reading is inside its window. The two that are not 1000 are the
divisor's own rounding, stated at the verdict: 921600 from 64 MHz is
BRR 69 for 69.4, a rate 6.4 thousandths fast (993); PRESC /256 at 9600 is
BRR 26 for 26.04 (998). The LSE legs read 1001 to 1004, HSI16's trim
against the crystal. The transports' 1001 is the start of the run - the
first frame waits for the engine's launch and a full shift register
starts "on the next baud clock edge" (33.5.17) - over 1024 frames; the
pad's 997 and 999 are the poll that sees each start bit, a few tens of
cycles in a 466 666-cycle span. The 4 Mbaud rung (OVER16) and the 8 Mbaud
one (OVER8) are timed although the open-drain loop carries neither: TC
ends the transmitter's frames whatever the pad's rise does to the
receiver, so the generator is read where the loop cannot be.

### The kernel clocks

**The console moves under itself and the verdict lines are the
witness**: USART2 to HSI16 (BRR 139 where PCLK wanted 556), then to
SYSCLK (BRR 556 - the same 64 MHz by a different route), then back to
PCLK, keeping 115200 throughout with not one framing, parity, noise or
overrun error counted. **USART1 ran off the 32768 Hz crystal**: 2048
baud at USARTDIV 16 byte-exact, and 4096 baud with OVER8, because eight
samples a bit halves the floor. And a `Uart` on HSI16 told that SYSCLK
moved to 16 MHz kept BRR 1667 - `rebase()` rewrote NOTHING, which is
what an application asks for when it names a kernel clock.

### The FIFOs

The transmit side took **nine characters** before TXFNF fell (eight
entries plus the shifter) with TXFE clear while they were in it; the
receive side took **nine and lost none** (RXFF set, ORE clear). Past
nine, **the overrun drops the NEWEST character and keeps the queue** -
twelve in, nine out, the first entry still the first thing sent. All six
RXFTCFG codes raise RXFT at exactly the depth 33.5.4 names, code 101
being the whole FIFO. **OVRDIS is not "no more errors", it is "no more
reports"** - and 33.8.4's second sentence is literal: with OVRDIS set
the RXFIFO IS BYPASSED, so a FIFO-mode receiver collapses to ONE
character in RDR (eleven in, the newest one out).

**ON THE LOOP THE FIFO PAYS WHERE BOTH SIDES RIDE A LEVEL.** 256 bytes
round the single wire through the task cost 259 interrupts with the FIFO
stated off and 36 with it on (`rx_threshold` at the whole FIFO, the
transmitter refilling an empty one): a single wire makes the two sides
of a byte one event, so the saving needs the receiver paced too, and
the receiver time-out takes the tail. Not one byte different, the same
public verbs and one option between them.

### The receive transport and the wire's idle (letters `p` and `q`)

- **The paced receiver**: 17 bytes at 115200 in 5 entries of USART1
  where RXFNE took 17, the tail delivered 5775 cycles after the last
  stop bit (a frame is 5555), and nothing entered in the 10 ms of
  silence after it - RTOF rises once a burst, not once a time-out.
- **K breaks in N slots**, both receivers: 64 slots, 4 of them a break
  (SBKRQ) - 60 bytes delivered intact and in order, FE counted 4, no ORE
  and no ring overrun, through the paced receiver and through the
  engine (where each error was one entry of the vector).
- **The engine's edges**: 17 bytes told by IDLE 5805 cycles after the
  last stop bit; 200 bytes at 1 Mbaud into a 64-byte ring, read on the
  vectors' edges alone - 7 edges, 3 laps, no overrun - on the driven
  single wire (the open drain's rise into the link wire gives out below
  1 Mbaud, above).
- **`tx_idle()` on the pad**: EXTI line 9 timing PA9's start bits (eight
  frames of 0xFF at 9600, one falling edge each), `tx_idle()` turned
  true 8 cycles after the last stop bit's end on the interrupt
  transmitter and 29 before it on the transmit engine - inside the
  poll's own resolution; a bit is 6666 cycles. The ring alone, which
  `tx_idle()` was, answers with up to nine frames still in the FIFO and
  the shift register.

### The bit-banged line: parity, framing, noise, tolerance

PA10 under USART1_RX (an INPUT alternate function) follows its own pull,
so software is the transmitter. A hand-made 8N1 frame at 2400 baud reads
back exactly; **an 8E1 frame with the wrong parity raises PE and only
PE**; **a stop bit that is a zero raises FE and only FE**.

**The noise flag is a MAJORITY VOTE DISAGREEMENT and nothing else.** A
2/16-bit glitch walked across a data bit raises NE at 6..9 sixteenths -
where it splits the three samples - and nowhere else, and with ONEBIT
set it can never be raised at all.

**The receiver's tolerance, walked from both sides in all four of tables
188/189's arrangements** at 2400 baud, in PARTS IN TEN THOUSAND (the
tables print per cent with two decimals):

| arrangement | last good + | last good - | table |
|---|---|---|---|
| ONEBIT = 0, BRR[3:0] = 0 | +450 | -450 | 188: 375 |
| ONEBIT = 1, BRR[3:0] = 0 | +500 | -550 | 188: 437 |
| ONEBIT = 0, BRR[3:0] != 0 | +400..450 | -450 | 189: 333 |
| ONEBIT = 1, BRR[3:0] != 0 | +500 | -550 | 189: 388 |

Every row MEETS its table and the ORDER is the tables' too: ONEBIT = 1
buys tolerance, a non-zero BRR nibble costs it. (The walk's own step is
50 parts in 10000.)

**ES0548 2.11.1 REPRODUCES, with its control.** A quarter-bit glitch to
zero inside the SECOND half of a stop bit spoiled **8 of 8** frames -
0x96 coming back as 0xCB with NO error flag at all - while the same
glitch in the FIRST half spoiled **0 of 8**. Silent corruption is
exactly what the erratum describes and there is no workaround.

### Auto-baud

**All four of 33.5.9's patterns converge on a rate the receiver was
never told**, learned from ONE character the software put on the pad:
mode 0 (any character starting with 1), mode 1 (a 10xx pattern), mode 2
(0x7F), mode 3 (0x55) learned BRR 21294/21315/21328/21328 against a
truth of 21333 - **one part in a thousand at worst, and zero for three
of the four**. Mode 3 given a 0x00 raises **ABRE** rather than a wrong
BRR believed. 33.5.9's own precondition is a refusal here: a detector
armed on a zero BRR has nothing to measure against, so `auto_baud()`
returns false.

### LIN

**The break is THIRTEEN zero bits**, measured on the pad the transmitter
drives: 5416 us at 2400 baud is 13.0 bit times. **The detector counts to
exactly LBDL's own length** - a 10-bit break sets LBDF with LBDL = 0 and
a 9-bit one does not; an 11-bit break sets it with LBDL = 1 and a 10-bit
one does not. LBDF is a flag of its own beside FE (ISR 0x6200A2 after
the last break) with an ICR bit of its own.

### Mute mode, the receiver time-out, the character match

**Idle-line mute silences the receiver completely** and an IDLE FRAME is
what brings it back. **Address-mark wake compares four bits or seven**
on the MSB mark and REPORTS the address character that woke it (0x85 for
a 4-bit address 0x5, 0xC5 for a 7-bit 0x45). **The receiver time-out is
twenty-two bit times to within the measurement**: RTOF rose 9173 us
after the character at 2400 baud, where 22 bits are 9167 - and the count
starts at the END of the stop bit, so the read's own delay is inside
that. **The character match fires on the character it was given and no
other**, and **CMIE does not gate the flag**: with the interrupt enable
clear the same LF still raises CMF.

**CR2.ADD is BOTH the mute address and the matched character** - one
field, two jobs, and 33.5.10 and 33.5.11 each describe it as if it were
theirs alone. A program cannot have both at two different values and
this driver does not pretend it can.

### Smartcard

8E1.5 on the TX pad alone at 1200 baud (kernel 250 kHz). **Smartcard
mode is its own half duplex with no HDSEL bit**: 0x3B out on the card
wire comes back as 0x13B, because the ISO 7816 frame is 8 bits PLUS
parity and RDR KEEPS the parity bit. **The guard time is counted in BAUD
PERIODS after the stop bit and holds TC down for exactly that long**: TC
at 10005 us with GT = 0 and 36628 us with GT = 32, a delay of 31.9 baud
periods where the register asked for 32 - and **TCBGT rises before TC**,
which is the flag that says the frame left with no NACK behind it
without waiting for the guard time. **CK is the kernel rate over TWICE
the prescaler**, counted by a DMA channel with no CPU: PSC 4/8/16/31
give 31250 / 15630 / 7810 / 4030 Hz against 31250 / 15625 / 7812 / 4032.
**The automatic retry is real and countable on the pad**: a NACK pulled
onto the wire from the PORT pull during the 1.5 stop bit with SCARCNT =
2 was followed by three further start bits and then the ISR read
0x26000F0.

### IrDA

**The 3/16 pulse is three sixteenths of the bit period**: 156 us at 1200
baud, where 3/16 of 833 us is 156. The idle level is LOW with a HIGH
pulse per zero - the opposite of the decoder's own input, which the
chapter says in one sentence and no figure repeats. **In low-power mode
the pulse stops being 3/16 of a BIT and becomes three periods of the PSC
clock**: 48 us with PSC = 64 on a 4 MHz kernel, where three periods of
62500 Hz are 48 us - a width that no longer moves with the baud rate.
**The SIR decoder turned a hand-made RZI frame back into its byte**, 3
of 3. **The glitch filter is one PSC period**: a low pulse of half of it
started nothing, which is what PSC is FOR on the receive side and why
the ENDEC refuses PSC = 0.

### The pads' extras

**SWAP exchanges the PADS**: the single wire moved to PA10 with PA9
given back, and 8 of 8 bytes went round it. **DATAINV survives a loop
where the LINE inversions cannot** - 4 of 4 with DATAINV at both ends
and 0 of 4 with TXINV + RXINV, because the idle level of a released
open-drain pad belongs to the PULL-UP and not to the transmitter, so an
inverted receiver reads the resting line as a start bit. TXINV inverts
the line idle level and all (measured on a push-pull transmitter); RXINV
inverts the receive line, and the same waveform read without it is not
the byte. **MSBFIRST is an exact bit reversal**: 0xB2 on the wire reads
back 0x4D. **DEAT is counted in SAMPLE times and not bit times**: DE
rose 416 us before the start bit for DEAT = 16 at 2400 baud, where a bit
is 416 us - sixteen sample times are one whole bit at OVER8 = 0. RTS is
asserted exactly while the receiver cannot take another character;
**CTS held high stops the transmitter BETWEEN frames** and releasing it
lets the character go, with the line pull-walked and no wire.

### Synchronous master

Counted on the CK pad with no CPU: **one clock pulse per DATA bit, none
for the start and the stop** - eight characters cost 56 rising edges,
i.e. 7.0 each - **LBCL adds the pulse of the LAST bit** (64 edges, 8.0
each), and CPOL is the level CK rests at.

### Host-assisted (outside `z`, `brio stress`)

**Streaming across the kernel clocks** (letter y): the console, in FIFO
mode as it is by default, took the host's stream byte-exact on PCLK at
115200, HSI16 at 115200, SYSCLK at 460800 and PCLK at 921600 - 4672,
4672, 17480 and 31364 bytes, ZERO wrong, zero hardware overruns, zero
framing errors - and again (4672 bytes, 0 wrong) with FIFOEN cleared
under a transport built for it: the receive drain reads ISR before every
RDR, which is right in both views.

**THE WAKE FROM STOP** (letter w), with the console on HSI16 and the
RTC's wake-up timer as the backstop:

- All three of 33.5.21's sources woke the part out of **Stop 0** from a
  byte the host sent halfway through the window, WUF seen once each and
  the backstop never firing: **start bit at 9600, RXNE at 9600, start
  bit at 115200, address match at 9600** - the Stop lasting 539..542 ms,
  which is when the poke landed.
- **THE BYTES SURVIVE.** All four poked bytes arrived at 9600 AND at
  115200, first byte 0xA5 - so a start-bit wake at these rates loses
  nothing.
- **The address-match wake keeps ONE byte and drops the rest**, which is
  right and is worth stating: the receiver is in mute mode, so the
  matching address character wakes it and is reported, and the three
  characters after it are not addressed to it. The console runs in FIFO
  mode here, and mute mode is what 33.5.21 asks of an address-match wake
  with the FIFO on - which this letter sets through the resource.
- **ES0548 2.2.4 REPRODUCES ON A USART WAKE.** With HSIDIV = /4 the same
  Stop was NOT ended by the same poke - WUF never rose - and ran to the
  RTC backstop's full 1.4..2.0 s, with the RTC wake-up timer as the
  control that the Stop itself works. The same erratum does NOT reach an
  RTC wake ([pwr.md](pwr.md)), and the difference is exactly the one it
  names - the USART is a clock-request peripheral (33.5.21's
  `usart_ker_ck_req`) and the RTC is not.
- **AND HSIKERON DOES NOT RESCUE IT.** The same leg with RCC_CR.HSIKERON
  set - HSI16 kept running for its kernel-clock consumer, so no request
  is needed - still did not wake: WUF 0, the backstop firing. So 2.2.4
  reaches further than the request path. Recorded as measured, not
  explained. A USART on the LSE kernel, on the same divided HSI, IS
  woken (letter `u`, "Against a far board"): 2.2.4 is the HSI16 kernel's.

### The receive FIFO under a starved channel (`test_stm32_dma` letters `p` and `q`)

**A RECEIVE FIFO BEHIND A STARVED CHANNEL CAN WEDGE, AND IT IS THE
SILICON.** Measured on the STM32G0B1RE (revision Z) with USART2's ring on
DMA1's channel 7 and two memory-to-memory copies on channels 1 and 2 -
the pair alternating with each other (RM0444 10.4.4) and the ring ranked
below them: after a starvation of a little over one character time - two
entries queued - the receiver can stop for good. RXFNE stands with one to
four entries, DMAR is set, the channel is enabled with its count frozen,
the DMAMUX routes the request with no overrun, ORE is clear, and nothing
moves; new characters fill the FIFO, ORE follows, and the receiver is
dead until it is initialized again. ST's HAL v1.4.7 alone, configured
the CubeMX way on the same board under the same starvation, wedges in
the same state register for register; it hides it during a stream only
because its error handler treats every overrun under DMA as fatal and
restarts the transfer - a loss each time. Every difference between the
two sequences (DMAR before or after UE and the channel, the threshold
code, IDLEIE, DMAT and the transmit channel, the receive channel's
number, starved by number or by level) was measured shared or harmless.
Neither ES0548 Rev 3 nor ES0418 Rev 5 lists it. How often a starvation
wedges depends on the cycle at which the grant comes back - 0 to 17 % of
the episodes for one length, moved by a few cycles of the copies' launch.

What releases a wedge, and what does not:

| action | releases | at what cost |
|---|---|---|
| DMAR cleared and set again | at once, every time | none - the standing entries are taken in order |
| the DMAMUX channel routed to request 0 and back | at once, every time | none |
| the channel's EN cleared and set | yes | the ring's position is lost |
| a CPU read of RDR, or the FIFO drained | yes | the bytes read |
| ORE or any ICR flag cleared, RQR.RXFRQ, the DMAMUX rewritten with its own value, a new character | NO | - |

The DMAMUX toggle shows the request ASSERTED throughout - handed to the
channel anew it is served within 200 cycles with no character arriving -
so the channel takes no further request until it sees this one fall:
10.4.3's handshake with a FIFO that keeps its request up across the
acknowledge because another entry stands behind the one just read fits
every fact, and nothing here can see the two signals themselves. With the
FIFO off there is no second entry and the wedge never formed - but every
starvation above one character then loses frames.

**THE CURE IS A KICK, AND THE FIFO STAYS.** The receive engine's vector
drops and raises DMAR (`Usart::rx_request_restart()`) when an entry for
the line's IDLE, a receive error or RXFT finds RXFNE still set on a fresh
read; RXFT is armed at seven eighths under an engine, the FIFO one entry
and the shift register short of an overrun. The three are each needed:
IDLE delivers the tail of a burst the threshold never reaches but comes a
character after the burst ends, too late inside a long one; the threshold
catches a wedge mid-stream; the error keeps a FIFO that both missed from
staying dead. A healthy stream never kicks - the channel empties the FIFO
within a few bus cycles of each character.

Measured, the ring told `rx_priority = low` so the pair starves it for
real (letter `q`, the host as the sender through the ST-LINK's bridge,
two copies launched together every 5 ms, a 2.5 s window each, every byte
of the xorshift stream judged with gaps located and measured):

| baud, pair | longest starvation | STM32G0B1RE | STM32G071RB |
|---|---|---|---|
| 115200, 2 x 2048 B | 2.6 characters | 23104 bytes in sequence, 0 kicks | 23104 in sequence, 0 kicks |
| 921600, 2 x 256 B | 4.3 characters | 153180 in sequence, 27 kicks | 146280 in sequence, 29 kicks |
| 921600, 2 x 512 B | 6.8 to 7.2 characters | 153180 in sequence, 223 kicks | 146740 in sequence, 90 kicks |
| 921600, 2 x 2048 B | 22 characters | 401 gaps of 2715 bytes, every one counted (`rx_skips()` 2715), the stream in sequence to its end | 401 gaps of 2584 bytes, `rx_skips()` 2584, in sequence to its end |
| 921600, no copies, after | - | 153180 in sequence | 146280 in sequence |

A kick at 4.3 characters is a wedge cured: that starvation alone queues
five entries, short of the seven RXFT watches and with no IDLE inside a
continuous stream. The same board without the kick, the oracle's
measurement on the STM32G0B1RE: 28 of 992 frames delivered at 921600 with
a 256-byte pair, the receiver dead at the end; with the FIFO off, 580 of
993. With the ring at its default `very_high` the pair starves nothing
(letter `p`: 115200 with a 2 KB pair, 921600 with 512 B and 2 KB pairs -
every byte in sequence, no overrun, no kick, on both boards).

WHAT THE KICK COSTS, read in the release listing of `test_stm32_dma`'s
USART2 vector: an entry that does not kick pays a literal load, a mask
test, one more ISR load and a branch over the vector it already had; a
kick is a call to `rx_request_restart()` - the guard, a CR3 load, a BICS
and two stores, the mask held for six instructions - and the saturating
count, about twenty cycles. The RXFT entry is a LEVEL: under a starvation
of seven characters or more the vector re-enters, kicking, until the
channel is served (77201 kicks in 10 s with 256-byte pairs back to back at
921600, where more than nine characters pile up between relaunches) - a cost that
exists only past the point where the FIFO off already loses every frame.

### Against a far board: the STM32F446RE's `usart_peer` (the second all-key `t`)

The desk pairs this board's USART1 with the STM32F446RE's, CROSSED - PA9
to its PA10, PA10 from its PA9 - the far board running the stm32f4
`usart_peer` (firmware 0x0401), driven IN BAND with the protocol every
architecture on the link compiles (`avrxt/src/apps/usart_link.hpp`): an
ack before every action, a bounded action, the peer back in command
mode by itself. Letters `1` to `9`, `0`, `r`, `s` and `u` run under the
second all-key `t` and never under `z`, which runs with the far board on its
dark console; letter `W`, the far engine's map of ES0548 2.11.1, runs under
neither (it takes some ten minutes). The
far peer's time base is its crystal (the ST-LINK's 8 MHz in bypass), so
what it times is THIS core's HSI16 - and this board times the same HSI
against its own LSE over the very window of each run, the stratum's
`LseTracker` ([clock.md](clock.md), "The core against the LSE, tracked")
on DMA1 channel 4 with a ring of 4096, a second of the crystal, for the
length of the set: every rate verdict is the divisor's on the wire with
the oscillator's walk taken out. Score: **171 of 171**, the refusals
SKIPPED with their reason; letter `x`, outside `z` and `t`, is letter
`7`'s control through the bridge (below).

- **The rendezvous and the refusal path** (`1`): ping, IDENT (kind
  0x04, another die's unique ID, a crystal time base), a bad checksum
  NAKed, a desync recovered; the far USART's refusals - a synchronous
  client, a seven-bit word - acked, reported `report_cfg_failed`, the
  peer at once in command mode again.
- **THIS HSI16 against two crystals at once**: three gapless runs of
  40000 frames at 1 Mbaud (64 MHz / 64 here, 90 MHz / 90 there; 4000 on
  the 8 KB part) timed by the far crystal at +2830, +2956 and +2946 ppm,
  and the same three windows timed by this board's LSE at +2823, +2947
  and +2939 - the far crystal against this LSE +7, +9, +7, judged to agree
  across the three within 7 ppm (a bound DERIVED below, "The
  rendezvous's bound"; 48 over the 8 KB part's 40 ms) and measured +6..+10
  over 20 runs, never more than 4 apart in one; the HSI 0.28 % fast at
  its factory trim and WANDERING: a few hundred ppm between runs
  a tenth of a second apart (a 1 Mbaud reference taken beside a run left
  the 9600 rung 500..700 ppm away three times in ten, which is why the
  verdict reads the run's own window). The far crystal's 1 Mbaud blast
  reads -2950 ppm on this ruler against the far divisor's 0 (90 MHz / 90)
  less the HSI's +2949 over its window: 1 ppm apart. A far `send` or
  `blast` STATES ITS OWN RATE (the Report's aux16 with
  `report_rate_stated` beside it, the far divisor on its crystal,
  usart_link.hpp), so the far transmitter is judged as tightly as this
  one - the far divisor's EXACT rungs (9600, 1, 1.5 and 2 Mbaud: 90 MHz
  over a whole number of sixteenths) at 100 ppm with the rest; a peer
  that cannot tell leaves the flag clear and is judged at the loose per
  cent.
- **The baud matrix** (`2`), on the two dividers' COMMON ladder: this
  USART makes 64 MHz / n (OVER8 adds no resolution, above), the far one
  90 MHz / k, and the two meet where k / n = 7 / 5 - above 2 Mbaud at
  2.56, 3.2, 4.267 and 6.4 Mbaud, 0.15 % apart. At 9600, 115200, 921600,
  1, 1.5, 2 Mbaud and those four: 16 frames echoed by the far chip, every
  one equal; this transmitter timed by the far receiver over a thousand
  frames or 25 ms (864 frames at 9600, nine tenths of the trackers'
  second) within 100 ppm of the divisor's error plus the HSI's over the
  run - measured within 60 ppm at every rung: 9600 +3044 against +3033,
  921600 +9472 against +9434 (BRR 69 for 69.4), 6.4 Mbaud +3151 against
  +3140. THE FAR RECEIVER'S FIRST OR LAST STAMP MOVES BY ABOUT 0.4 BIT
  from run to run - with the 64 frames of 25 ms at 9600 it read +600 ppm
  off in a third of the runs, with 288 at 115200 +140 - which is why the
  span is a thousand frames (0.04 / N: under 40 ppm); the cause is not
  located. The far blast whole on the polled receiver (16000 of 16000 at
  6.4 Mbaud, a hundred cycles a frame, on a lean loop: the ISR load, the
  stamp, the RDR load and the count), its rate within 40 ppm of the far
  divisor's stated one less this HSI's (6.4 Mbaud: +1174 against
  +4464 - 3264); and 1024 frames whole through the interrupt receiver and
  through the receive engine (DMA1 channel 3) at every rung, OVER8's task
  above 4 Mbaud.
- **The frame matrix** (`3`) at 115200: 8N1, 8E1, 8O1, 7E1, 7O1, 9N1,
  8N2, 8N0.5 and 8N1.5 echoed by the far chip; 7N1 and 6E1 SKIPPED (the
  far USART refuses them: M counts its parity bit and has no seven-bit
  word), 9E1 SKIPPED here (no word length on this USART either); the
  half stop bits timed as LENGTHS by the far receiver, 19 and 21 half
  bits within 100 ppm of an 8N1 run with the HSI taken out.
- **The receiver's errors from a foreign sender** (`4`): six frames of
  odd parity into an even receiver, PE on each, the data readable; THE
  FE ONSET with the far divisor 0..8 % off 9600, back to back (a fast
  sender's frames with idle between them never fail the stop bit): clean
  within 4 % either way, FE from +5 % and -6 % (33.5.8's tables 188/189
  give 3.75 % at OVER16 and BRR[3:0] = 0, NE rising first from 4 %); a
  run of 16 read late - ORE, and the RXFIFO's eight plus the shift
  register's one kept in order (the tenth frame is the overrun, as the
  loop measured); THE START BIT'S VOTE, one low pulse of w ticks on the
  idle line at 9600 (a sixteenth is 156): nothing under 1350, a start
  with NE from 1350 to about 1700 with ONEBIT clear, and with ONEBIT set
  a start from about 1400 and no NE at any width - the threshold blurred
  by a sample's phase from run to run; a break of 20 bit times one frame
  with FE and data 0x00, LBDF only in LIN mode; THE FIFO'S PER-ENTRY
  FLAGS drained late - six wrong-parity frames, three PE with the clear
  after the RDR read, six with it before (above); and through the task,
  odd parity counted six of six and nothing delivered, a break one frame
  error and nothing delivered.
- **The far engine's shapes** (`5`), the STM32F446RE's TIM1 drawing what
  no clean UART can, every rate against this receiver's REAL one (its
  divisor's error and the HSI's out): WHERE A START IS BELIEVED, eight
  low pulses a width at 115200 (a sixteenth is 13 ticks) - nothing under
  half a bit in either oversampling, 3 of 8 taken at 110 ticks (8.4
  sixteenths) with NE, every one from 117, clean eight of eight from 130
  at OVER16 (10 sixteenths: 33.5.6's samples 8, 9 and 10) and from 156
  at OVER8, whose NE band is wider; THE OPERATIONAL RANGE of a gapped
  0x00 frame, free of any divisor's rounding: decoded from -50 to +45
  per mille at 9600 and -50 to +55 at 115200, broken by FE on the slow
  side and by 0x80 on the fast one (bit 7's sample in the stop bit), the
  majority vote's NE rising first, from +-30..50 per mille - table 189's
  3.33 % inside both ways; ONE STRETCHED CELL of 0x55 at 115200 - the
  start or a data cell 30 % longer or shorter decoded, 45 % not, the stop
  cell's length free with an idle behind it. **ES0548 2.11.1 REPRODUCES
  AT EVERY RATE**, the far engine drawing a glitch to zero INSIDE the
  stop bit of eight 0x96 frames (the protocol's `stage` op): an eighth of
  a bit from 75 % of the stop bit corrupts 7 or 8 frames of 8 at 2400,
  9600 and 115200, a quarter at 1 Mbaud (an eighth is under the far
  engine's floor there), a tenth from 88 % at OVER8 - every one 0x96 read
  as 0xCB, the data one bit late with the stop bit's 1 shifted into bit
  7, and NO flag at all; the same glitch from 6 % (the first half)
  decodes 8 of 8 clean everywhere. Walked across the stop bit, a glitch
  over the majority's samples is a noise flag and no corruption, past
  them it is the erratum - from 62 % at 115200 and 1 Mbaud, from 75 % at
  2400 and 9600, where the start's detection puts the samples later in
  the bit. With 8N2 it is the FIRST stop bit's second half: a glitch
  anywhere in the second spoils nothing. AND THE WIDTH IS NOT THE
  ERRATUM'S "shorter than the half-bit": letter `W` (outside both
  all-keys, some ten minutes) draws the glitch's width, 3 to 44 % of the
  bit, against its start, 50 to 95 % of the stop bit, at 4800, 9600,
  14400 and 19200, eight frames a cell, twice. At 4800, 14400 and 19200
  the map has one shape, both runs alike: a glitch reaching the
  majority's samples (from 50 to 56 %) is a noise flag on every frame and
  no corruption; 59 % is the border, the two mixed; from 62 % on, past
  the samples, every width from 6 to 25 % corrupts eight frames of eight
  with no flag (four to seven in a few 6 % cells), to the stop bit's end; a
  glitch of 31 or 38 % starting at 62 or 65 % - one that lasts past
  where a narrow one would already have done its damage - corrupts AT
  MOST ONE frame of eight (at 68 %, at most two, but for one 4800 cell
  that took all eight); and a 3 % glitch, half a sixteenth of the
  receiver's sampling, corrupts some cells and not others, different
  cells in the two runs - the sampling clock catching it or not. At 9600
  there is no such shape: the noise flag reaches 65..71 %, the corruption
  region past it is broken by clean cells, and the two runs disagree
  cell by cell (the open finding below).
- **The receive ring beyond the bridge** (`6`): the receive engine ranked
  LOW beside test_stm32_dma q's pair (two copies alternating on DMA1's
  channels 1 and 2, every 5 ms), a counting stream from the far chip
  with no gap for 2.5 s at 1, 2, 3.2 and 6.4 Mbaud. With no pair nothing
  is lost at any rung; a 512 B pair at 1 Mbaud loses nothing (the kick
  cures its 4-character starvations); past the FIFO's nine characters
  frames go - 5746 at 1 Mbaud with 2 KB pairs, 60783 at 6.4 Mbaud - in
  500-odd gaps, NOT ONE BYTE CORRUPT OR TAKEN TWICE and never more gaps
  than `rx_skips()` moved, the receiver delivering to the end of every
  leg: THE KICK RACING A CHARACTER LOSES NOTHING IT DOES NOT REPORT, to
  6.4 Mbaud. `rx_skips()` is an EPOCH under an overrun, not a count:
  one ORE costs many characters and moves it once (15881 lost against
  13354 at 2 Mbaud, 60783 against 20011 at 6.4). At the default level
  the ring outranks the pair at every rung, no kick.
- **Finding 4 over the wire** (`7`): the DMA study saw a whole message
  vanish now and then from a host-fed stream with no counter moving.
  From the far chip instead, bursts of 200 frames at 921600 a millisecond
  apart, the default ring, five minutes: **18 878 000 frames in 94 390
  bursts, not one gap**, the far end's counts agreeing. Letter `x` is the
  control through the ST-LINK's bridge - the console moved to 921600 on
  a receive engine for five minutes of the host's xorshift back to back
  (brio stress's `sink`) and five in the study's shape, 200-byte bursts a
  millisecond apart (its `sink_bursts`, the shape the board announces):
  22 431 456 bytes back to back and 17 908 815 in bursts in one session,
  not one gap, not one corrupt byte, no skip, ORE or FE either. Neither
  path loses a burst today (at the study's rate, one in some 28000
  messages, five minutes of either would have shown three): the device
  is ruled out at the study's rate, and the bridge did not reproduce it
  in two sessions.
- **The pad and the shared wire** (`8`): the HDSEL receiver on PA9
  hears the far engine drive that wire from ITS receive pad, eight
  frames whole at 9600, 115200 and 1 Mbaud; ONE OPEN-DRAIN WIRE BETWEEN
  TWO CHIPS, this end swapped onto PA10 and the far HDSEL on its PA9:
  eight frames out, each heard back as itself and answered by the far
  end's complement, at 9600, 115200 and 460800 (the far peer waits for
  its reply's own echo before it flushes: on the F446 that echo's RXNE
  lands after TC, and a flush at TC answered the echo inside this end's
  next frame - measured, then fixed in the peer); and THE WIRED AND: the
  far end pulls 0x00 into this end's 0xFF behind a trigger frame, which
  comes back whole while the 0xFF reads 0x00.
- **IDLE from a faster sender** (`9`): twenty bursts of sixteen frames at
  50000 baud, the far end 0 to 3 % faster than this receiver's real
  rate, gaps of one frame, one and a half and five: IDLE once a burst at
  every gap and offset, the receiver time-out (RTOR 12 bits) once a burst
  from a frame and a half - the control for the F446's ES0298 2.13.1,
  and ES0548 lists no such item. THE IDLE FRAME ITSELF, drawn by the far
  engine with the gap stated in ITS bit times (the `stage` op): sixteen
  frames, an idle of 9.5 to 11 sender bits after each. From an exact
  sender IDLE follows every frame from ten bits and none at 9.75; from a
  sender 1 % fast from 10.25 of its bits, 3 % fast from 11 - so ONE IDLE
  FRAME AND THEN A CHARACTER FROM A FASTER SENDER IS NOT TOLD HERE
  EITHER, which is ES0298 2.13.1's own case: the receiver counts its
  idle frame from the stop bit as IT sampled it, and a faster sender's
  frame ends ahead of that by its error over the frame (ten bits of 3 %
  are 0.3 bit). The far F446 reads the same table, entry for entry
  (docs/stm32f4/usart.md).
- **Auto-baud from a foreign crystal** (`0`): the far chip SENDS four
  frames of each mode's pattern at 9600, 57600, 230400 and 123456 baud
  into this receiver told 19200, and the learnt divisor is judged in
  kernel cycles a bit against the far rate it states and this HSI's
  error over the leg, within a DERIVED tolerance: two cycles (the
  synchronizer, the truncation) and 900 ppm of the bit - the HSI16's
  WANDER between the one span the detector times and the leg's 7 to 13
  ms window, measured by TIM1 capturing every edge of a far 0x55 blast
  on PA10 (four records of 17000 edges a rate, every span against every
  window of 4 to 16 ms around it: 868 ppm the worst at one bit at 9600,
  sd 165, 603 at eight bits; in counts 5.8 of 6686 at 9600 and 1.2 of
  279 at 230400, each under one count plus 868 ppm of the bit) plus the
  tracker's grain (32 ppm), rounded up to the half cycle the divisor is
  read in. At 9600 that is 8.5 cycles of 6687, where a
  mode-0 leg measured up to 3 off the leg's mean; at 230400, 2.5 of
  279. ALL FOUR MODES learn every rate inside it (22 runs of each leg:
  at most 2 cycles off at 9600, 1 above), the measured frame itself read
  whole and the frames after it decoded.
  ABRE: mode 3 handed a 0x00, AND mode 2 handed a 0x55 - its second
  measurement disagrees with its first - each with FE on the measured
  frame and the divisor left as it was. THE RANGE as 33.5.9 states it,
  at OVER16: 3.2 Mbaud (20 cycles a bit) learnt, 6.4 Mbaud (10) refused
  with ABRE. AT OVER8 NO LEG IS JUDGED: the common ladder's one rung
  inside OVER8's range and outside OVER16's is 6.4 Mbaud, where a bit is
  ten kernel cycles and the detector counts it to the cycle - its own
  quantization a tenth of the bit - so mode 3's transition check
  disagrees with itself now and then: 6.4 Mbaud at OVER8 is learnt
  sometimes and raises ABRE about one run in five. Auto-baud at a tenth
  of the clock is beyond what a leg can claim; the range 33.5.9 states
  is unchanged by it.
- **LIN from a foreign crystal** (`0`): the far engine's breaks at
  19200, of 9, 10, 11 and 13 bits with LBDL = 0 and of 10, 11 and 13
  with LBDL = 1, raise LBDF from ten and from eleven, never one bit
  short; with LIN mode
  off a 13- or a 20-bit break raises none (33.5.13's "without taking
  into account the break detection" - where the F446 sets LBD anyway,
  docs/stm32f4/usart.md). THIS END'S BREAK timed by the far receiver,
  from the break to a 0x55 written as SBKF falls: 14.97..15.04 bit
  times in LIN mode (thirteen and the two stop bits 33.5.13 inserts) and
  11.97..12.03 without (a frame's ten and the two of 33.5.4), over five
  runs. A LIN HEADER TAKEN AS A SLAVE TAKES IT: a master 10 % off the
  nominal 19200 either way (21120, 17280 - past any receiver's
  tolerance), its 13-bit break, a 0x55 sync and four payload frames: the
  break raises LBDF at the slave's nominal rate, the detector armed on
  LBDF learns the sync within one cycle a bit (3038 for 3039, 3715 for
  3714; judged within the same derived tolerance) and the payload
  decodes. ARMED BEFORE THE BREAK IT CANNOT: the
  break is then its first character, LBDF never rises and it ends in
  ABRE - which is why `AutoBaud`'s header says to arm it on the break.
- **RS-485** (`r`): the driver enable through the TASK - `Rs485` with
  DEAT 16 and DEDT 8 at OVER16, and a `UartTask` with the DE active low
  and DEAT 31, DEDT 31 at OVER8 - on PB3 (USART1_RTS_DE_CK, AF4), the
  desk's SPI SCK wire, the far board's PB13 an input under it: five
  bursts of eight frames each, every frame received whole by the far
  chip (40 of 40, the sum right) while DE framed each burst once; DE
  timed on this end's own pads in SAMPLE times (USARTDIV / 16 kernel
  cycles at either oversampling, on TIM2 - the kernel's own HSI, its
  error cancels): 16.48..16.51 before the start bit and 7.88 after the
  last stop bit for 16 / 8, 30.97..31.26 and 30.94..31.23 for 31 / 31
  (medians of five, over three runs), within a sample and
  the sampling loop's period (36 cycles; a sample is 34 at 115200 OVER16
  and 69 at OVER8).
- **IrDA SIR between two ST ENDECs on a bare wire** (`r`): the polarity
  33.5.18 states in one sentence is READ ON THE PAD - the encoder idles
  LOW with a HIGH pulse a zero, and with TXINV set it idles HIGH with a
  LOW pulse: TXINV and RXINV act on the SIR side of the ENDEC, so this
  end alone supplies the one inversion each direction needs. With both,
  the far F446's decoder takes 32 frames of 32 at 9600 and 115200,
  normal pulses (2.9 sixteenths at 9600) and low-power ones (PSC 35:
  three periods of 1.83 MHz, 100 of 105 cycles on the pad); and this
  decoder takes the far encoder's 32 of 32 at both rates, its normal
  and its low-power pulses alike. Without them the far decoder takes
  some twenty frames, nearly all with FE, and this one none. ONE
  INVERSION ALONE IS NOT
  ENOUGH: with RXINV and no TXINV this end's encoder idles low into the
  far decoder, a standing pulse its receiver chews - and an ENDEC is
  half duplex, so the far transmitter's frames after the first came out
  undecodable (one of 32). The far sender's first frame waits 20 ms past
  the settle, the turnaround the F446's suite measured the other way
  round (its first start pulse missed one leg in a few without it,
  docs/stm32f4/usart.md). THE LINK COMES UP WITHOUT A PULSE ONLY IF THE
  TX PAD WAITS: with the pad on the block through its reset and
  configuration, the far decoder took a 0xFF from the low that put on the
  line in every inverted leg (8 of 8, the inversions written before the
  first enable or after it alike) - at 9600 a frame ending 4.04 ms after
  the far chip's ack, against a flush 4 to 5 ms after it by its
  millisecond's phase, so now and then counted first, the 32nd frame left
  out (sum 0x8A0 for 0x7F0); held at the line's idle by a pull until
  TEACK, 0 of 8. `IrdaLink::init` takes the inversions (`IrdaInversion`)
  and holds the pad so. The far chip refuses SIR above 115200, its
  chapter's ceiling (RM0390 25.4.12). On this end's pad the 3/16 pulse
  at 115200
  reads 93..100 cycles for 104 and the low-power one 100..101 for 105,
  where the F446's pad reads its 294 exactly: `IrdaLink` leaves the TX
  pad at its reset (low) speed, and driven at very high speed (a scratch
  build) both read 101 - the slope is most of the gap, the last four
  cycles not located.

- **Mute mode from a foreign sender** (`s`): MMRQ written on a line idle
  for 2 ms does not mute (33.5.10's note: once an idle frame has
  elapsed); THE IDLE-LINE WAKE - MMRQ after a burst's first frame drops
  the rest of it, the idle frame between bursts wakes the receiver with
  no IDLE flag, the next two bursts whole (17 frames of 24, in order);
  THE ADDRESS MARK on the far `send_mpcm` (four groups of an address and
  its data): on an 8-bit word the mark is bit 7, a four-bit address 5
  takes 0x85 and 0x95, a seven-bit 0x05 takes 0x85 alone and 0x15 0x95
  alone; on a 9-bit word the mark is bit 8, four bits take 0x05, 0x85 and
  0x45, and with ADDM7 the address is EIGHT bits (33.5.10's note) - 0x85
  and 0x05 each their own group. A wrong-parity stream (8O1 into 8E1)
  woken by its address: every frame taken carries PE, THE WOKEN ADDRESS
  INCLUDED - where the far F446 sets none after a software mute (its
  ES0298 2.13.3).
- **The wake from Stop by the far chip** (`u`): USART1 - the link, FULL on
  every part of the family, its wake on EXTI line 25 - on HSI16 or the
  LSE as its kernel, the far `send` four frames 600 ms after its ack, the
  RTC's wake-up timer the backstop: woken out of Stop 0 on a start bit
  at 9600 and 115200, on RXNE, on address 0x85 under mute, out of Stop
  1, and on the LSE at 2048 baud (32768 / 16, exact) - the Stop lasting
  595..602 ms, WUF once, the backstop never; the four frames survive
  every start-bit and RXNE wake, the matching address alone the mute's.
  Four addresses that are not 5 never wake it (the backstop at 1.4 s).
  ES0548 2.2.4 FROM A FOREIGN SENDER: HSIDIV = /4 and the same start-bit
  wake on HSI16 never comes, the RTC ending the Stop at 1.8 s - AND ON
  THE LSE IT DOES: the same divided HSI, the USART kernel-clocked by the
  crystal, woken at 601 ms with all four frames. With HSIKERON keeping
  HSI16 running the HSI16 kernel was not rescued (letter `w`), so 2.2.4
  is not the clock request alone: it is the HSI16 kernel's on a divided
  HSI, and a USART on the LSE is the way round it.
- **THE TASK under the address-match wake** (`u`, twelve runs): a `Uart`
  naming `wake_from_stop = address_match` with its FIFO unstated comes
  up with the FIFO OFF, the option's default (33.5.21); the program
  states address 5 on the resource with the instance down. The far `send`
  sends 0x7E..0x85 at 2400 - two data frames, then six address marks of
  which the last alone is 5's, a frame every twelve bits (5.0 ms, the far
  peer's two-bit gap). UNDER MUTE MODE the matching address alone ends
  the Stop: the task counts one wake, the backstop never fires, its ring
  holds 0x85 and nothing else. WITHOUT MUTE MODE the address does not
  gate the wake: the task's own RXNEIE is a wake-up source (33.5.21's
  first bullet), the Stop ends at the FIRST frame - 39 ms before the
  mute leg's in every run, seven pitches being 35.0 ms, read on the RTC
  at 3.9 ms a unit - and the ring takes all eight frames, WUF still
  counted once when 0x85 matches with the core awake. So the option
  alone wakes on any frame; a program that wants its address alone to
  wake it sets mute mode beside it.

### The rendezvous's bound

Letter `1` times this HSI16 over each run twice, by the far crystal (the
far figure) and by this board's LSE through the tracker (`hsi_over`): the
difference is the two crystals' offset - the same in the three runs,
two crystals moving apart by well under a ppm in the second they take
(docs/stm32f4/usart.md, "The rendezvous's bound", where every stamp of
115 windows was fitted) - plus the two measurements' errors, each a TIME
at an end of the run whatever its length:

| the term, per run | its time | from |
|---|---|---|
| the far stamps: the F446 sees a frame at the next turn of a poll of some 22 cycles at 180 MHz, behind its receiver's 1/16-bit start sampling (62.5 ns at 1 Mbaud) | +-0.19 us | the far peer's release listing |
| one SysTick handler there on a stamp | +-0.25 us | some 45 cycles: the M4's entry and return and the handler |
| this end's tracker: exact for its window's mean, the window starting 0.5 to 0.8 ms ahead of the run and stopping up to an interval short of it, the HSI16 wandering hundreds of ppm over such slivers; with the captures' 8-cycle grain | +-0.47 us | measured on the far STM32G0 peer, the same tracker over 90 windows of 40 ms ([clock.md](clock.md), "The core against the LSE, tracked") |

Two runs' differences part by at most twice the sum, 1.82 us, and two
whole-ppm figures in each add two: over 0.4 s that is **7 ppm**, the
verdict's bound, computed in the suite from those three times
(`agree_ppm`); over the 40 ms the 8 KB part's 62 ms ring allows, 48.
The offset itself is judged within 150 ppm, the tens of ppm two
crystals' tolerances allow (UM2324 6.7.1 names a 20 ppm part for the
LSE; the ST-LINK's 8 MHz has none stated) against the 0.3 % an
uncorrected HSI16 would put there.

### Where the hot paths run from

The task's placement (util/placement.hpp, [platform.md](platform.md)'s
"Code in SRAM") moves both interrupt bodies, `isr()` and `dma_isr()`,
each one flattened instance in SRAM under `InRam`; under `InFlash`, the
default, each is inline in its vector. Both placements flatten the body:
the transmit engine's completion carries the next block's launch
(`pump_tx()`) inline, never a call, while `harvest()` and the engines'
`abandon()` stay out of line in the flash - once a burst and the error
path. Measured by `bench_stm32` (`InFlash`) and `bench_stm32_ram`
(`InRam`, the vectors in the flash in both) on the Nucleo-G0B1RE, the
cycles an entry with the meter's stamps inside:

| path | flash | SRAM | SRAM bytes |
|---|---|---|---|
| the transmit engine's completion, a block of 4096 at 2 Mbaud / 1 Mbaud / 115200 | 232 / 230 / 246 | 212 / 209 / 223 | 264, 340 with a receive engine |
| the transmitter's FIFO refill, eight bytes an entry, at 115200 / 2 Mbaud | 323 / 337 | 318 / 318 | 404 an engineless port's `isr()` |
| the receiver, four bytes an entry at 115200 | 355 | 316 | 332 |
| the receiver at 2 Mbaud, 256 bytes: entries, cycles an entry | 51, 408 | 64, 316 | |
| the console's interrupt, a print of 4096 bytes (letter `p`) | 295 | 275 | 436 |
| the receive engine: the line's IDLE / the ring's marks | 206 / 226 | 225 / 226 | 84 / 340 |

Every line of 256 and 4096 bytes stays wire-bound either way (`x` 1.00
to 1.05 to 2 Mbaud, the same under both): the placement buys the core's
time, not the wire's. The receiver and
the completion pay a ninth and a tenth; the refill, a store loop
whose turns wait on TXFNF, pays least; and the receive engine's IDLE
entry pays a veneer back into the flash for `harvest()` and comes out
longer from SRAM - a program on a receive engine gains from `InRam` in
the completion and the marks alone. `LpUart` takes the same parameter
([lpuart.md](lpuart.md)).

## On the STM32G071RB

`test_stm32_serial` runs on the Nucleo-G071RB (DEV_ID 0x460, REV_ID
0x2000) and scores **112/112** against the G0B1RE's 118, its console the
FIFO-mode USART2 it defaults to; letters `y`, `w` and `v` pass there with
`brio stress` as on the bigger part.

**TABLE 183 IS A DIFFERENT ROW THERE, and the silicon says so.** The part
has FOUR USARTs and one LPUART: USART1 and USART2 FULL, **USART3 and
USART4 BASIC** (USART3 is FULL on the G0B1), USART5, USART6 and LPUART2
absent. Letter `a` walks every instance the part HAS and finds the
reserve's `usart_is_full` column, the device header's own
`IS_UART_FIFO_INSTANCE` and the silicon agreeing on the FIFO for each of
them; CR2.CLKEN sticks on every USART including the BASIC ones and on
neither LPUART, so the synchronous row of table 184 still cuts
USART-vs-LPUART and not FULL-vs-BASIC; and PRESC still takes a value and
reads it back on a BASIC instance while the transmitter then emits
NOTHING (90 us of nine zero bits at /1 and no frame at all at /16), which
is the same finding on another die.

**THE WAKE LINE FOLLOWS THE COLUMN AND NOT THE INSTANCE NUMBER**: with
USART3 BASIC here, `usart_exti_line(3)` is 0xFF and the part has three
wake lines (25, 26 for USART1/2 and 28 for LPUART1) where the G0B1RE has
four. The vectors move the same way, derived from presence:
USART3_4_LPUART1_IRQn here against USART3_4_5_6_LPUART1_IRQn there, both
bound through `BRIO_STM32G0_USART3_HANDLER`.

**AND ONE ERRATUM TAKES FOUR VERDICTS WITH IT.** ES0418 2.2.4 says the
EXTI-related DMAMUX trigger inputs are routed to `it_exti_per(y)` instead
of to `exti[15:0]`, and this suite's edge counter - a DMAMUX request
generator triggered by an EXTI line, the instrument letters `j`, `m` and
`o` use to count a clock with no CPU - is exactly what that breaks. The
boot probe measures it on PB3 and the numbers are the mechanism: four pad
edges move **0** words with the EXTI event mask alone and **1 of 4** with
the interrupt mask armed too, while the EXTI's own rising pending bit
sees every edge and four SOFTWARE events on the same line, generator and
channel move **4**. So the trigger follows the LEVEL of the line's
pending interrupt - no IMR bit, no level, no trigger; with one, the first
edge raises it and nothing further arrives until somebody clears the
pending bit. The sheet's "no workaround" is right for this purpose: the
clear would need a CPU in the loop, which is the one thing a counter with
no CPU cannot have. The smartcard CK ladder, the synchronous CK census
and both IRTIM counts therefore skip by name - and the frames of those
letters are timed on the wire all the same, the rate needing no
counter (the table above).

ES0418 2.12.2 (data corruption on a noisy receive line) is the G0B1's
2.11.1 and letter `f` stages it the same way; 2.12.3 is the prescaler
documentation item measured above. **2.12.4** (data corrupted when ABREN
is cleared during a reception) is answered STRUCTURALLY:
`auto_baud_off()` refuses with UE set, so this driver has no path to that
write and a disabled receiver is not receiving. 2.12.5 (NE set with
ONEBIT on a noisy START bit) is not exercised - letter `f`'s ONEBIT rows
run on a clean start bit - and 2.12.1 (an SPI-slave TC anticipated) has
no letter, there being no synchronous slave on this desk.

## On the STM32G031K8

`test_stm32_serial` scored **79 of 88** as one image on the
Nucleo-G031K8 (DEV_ID
0x466, REV_ID 0x1003). This part has **USART1 and USART2 and nothing
else**, and the split moves one step further down table 183: **USART2 IS
BASIC HERE** - no FIFO, no PRESC that divides, NO KERNEL-CLOCK
MULTIPLEXER (`usart_has_clock_select(2)` is false: the header declares no
`RCC_CCIPR_USART2SEL_Pos`) and no wake from Stop. So the console's own
instance is the one instance this suite cannot move off PCLK, and three
legs go with that: letter `d`'s console-kernel-clock legs skip by name,
letter `a`'s "does a BASIC instance's PRESC DIVIDE" leg has no subject
that is not the console's own pad, and letter `w` - the wake from Stop
with ES0548 2.2.4 staged - skips whole, having no wake line to arm.

**THE FOUR EDGE-COUNTER VERDICTS RUN HERE**, because the erratum that
took them on the G071 is revision Z's alone on this part: ES0487 2.2.4
is marked absent on revision Y, and the boot probe on PB3 agrees - four
pad edges into the request generator move **4 words with the event mask
alone and 4 with the interrupt mask armed** (the G071RB: 0 and 1 of
4). And
ES0548 2.11.1's twin, **ES0487 2.10.1**, applies on Y and REPRODUCES with
its control in letter `f`: the glitch in the stop bit's second half
reaches the byte, the one in the first half does not.

**AND IT IS WHY THREE OTHER SUITES MOVE THEIR CONSOLE TO LPUART1**
(`test_stm32_clock`, `test_stm32_spi`, `test_stm32_i2c`): a console whose
divisor follows the switch under test cannot report on it, and every
LPUART has a multiplexer (34.4.6). LPUART1_TX/RX reach PA2/PA3 at AF6 -
the same two pads - which is letter `v`'s shape put to work.

What still runs on this part is most of the chapter: USART1 is FULL, its
CK, DE, RTS and CTS are on **PB3 and PB4 at AF4** (DS12992 table 14, and
the LQFP32 bonds both), so the smartcard clock ladder, the driver enable
and the flow-control legs are all measured here. What skips beside the
above: LPUART1's own single wire (its only bonded pads on this package
are the console's) and LPUART2 (absent). Letter `v` runs whole on the
crystal this board carries - the console on LPUART1 at 9600 baud from
the 32768 Hz LSE (1062 bytes, 0 wrong, BRR 874) and the Stop 1 ended by
a start bit on that LPUART, 590 ms deep with the character intact -
which is the letter's 2/2 as on the Nucleo-64s.

## The peer: `usart_peer` (the instrument a far board's suite drives)

This stratum's port of the protocol's instrument
(`stm32g0/src/apps/usart_peer.cpp`, firmware 0x0301, built for the
G0B1RE and the G071RB): one polled loop on USART1's PA9/PA10, dark until
a frame checks out (PA9 an input, no pull on either pad; a NAK only
while engaged), a bounded action after every ack. The surface is the
protocol's; the engine is this chapter's, item by item:

| the chapter's offer | in the peer |
|---|---|
| the 8-deep FIFOs (33.5.4) | USED: the echo and the sink drain the RXFIFO polled, the ISR word read before each RDR and its flags cleared before the pop; a blast keeps the TXFIFO topped up, gapless at 6.4 Mbaud with no DMA |
| transmit and receive DMA | DECLINED: the FIFO is eight frames of slack under a loop of a fraction of a frame (below) |
| OVER8 | USED where it is closer to the rate, and alone under the protocol's clk2x flag - for range: it adds no resolution (above) |
| the kernel-clock multiplexer | PCLK; HSI16 DECLINED - a lower ceiling and the same oscillator |
| TIM1_CH2 / TIM1_CH3 on PA9 / PA10 (AF2) | USED: THE WAVEFORM ENGINE - the pad re-muxed to the timer with the channel forced high (no edge), toggle mode with CCR preload off, the first compare by the CPU and every later one by DMA1 channel 1 on the channel's own request from a table of ABSOLUTE compares; the prescaler per action so the longest interval fits half the counter; the CPU sampling the pad meanwhile and counting its transitions, and the run timed whole, so a compare written too late (an edge a wrap later) is caught. The `stage` op is its widest user: a train of frames drawn cell by cell from usart_link.hpp's cell table, a pulse inside each and the gaps in sixteenths of a bit, refused where a pulse is under the floor or the train past the 512-edge table |
| SWAP + HDSEL | USED for the shared wire: the single wire on PA10, the far TX's, open drain on its pull-up, the reply's own echo flushed |
| TXINV / RXINV | USED for IrDA alone, both at once: two ST SIR ENDECs face each other with opposite polarities (33.5.18), and both bits act on the SIR side of the ENDEC (measured, letter `r`); one alone leaves the encoder idling low into the far decoder, which its half-duplex ENDEC chews |
| the 0.5 / 1.5 stop codes | served (33.5.5 recommends them for smartcard mode and forbids them nowhere) |
| seven-, eight- and nine-bit words | 6+P, 7, 7+P, 8, 8+P and 9 served; 5 and 6 data bits without parity and 9 with REFUSED |
| synchronous master and slave | REFUSED: USART1's CK pads carry another wire on this desk, and the far clock reaches PB3 only through a strap the desk has not got |
| IrDA SIR | IREN to 115200, PSC from the requested filter, the low-power pulse from the requested width |
| auto-baud, the receiver time-out, LIN, mute, DE | the DUT's subjects: the peer feeds them (send, brk, autobaud_tx, send_mpcm) |

**THE TIME BASE IS TRACKED, NOT MEASURED ONCE.** The core runs the HSI16
through the PLL, 0.26..0.33 % fast on this die and wandering hundreds of
ppm over seconds - measured once at boot it read +3326 ppm and a minute
later +2950..+3093, and three calibrate runs a second apart then read
+164, +8 and +76 ppm against the far crystal. So the peer measures its
core against the LSE ALL THE TIME, in hardware - the stratum's
`LseTracker` ([clock.md](clock.md)) on DMA1 channel 2, a ring of 4096
captures, a second of the crystal - takes the rate over the last 250 ms
into every divisor and edge before each action, converts a span at the
rate over the span's own window, and states a `send` or `blast`'s rate
(the Report's aux16, `report_rate_stated` set) at the rate over the
run's own window. Then: -4..-14
ppm against the far crystal. A `onewire` reply's own echo is waited for,
two bit times at most, before it is flushed (the far F446 peer measured
that echo landing after TC).

**The cost.** The echo's clean character is some thirty instructions with no
call (release listing): the ISR load and its test, the TIM2 stamp, the
RDR load, the TDR store and the count, the mask and the bound held on
the stack - some fifty cycles against the hundred of a 6.4 Mbaud frame
(85 at 7.5); the sink is the same less the store, and its 16000 frames
at 6.4 Mbaud arrived whole with every span within 40 ppm. The engine's
floor is a DMA reload: an interval of 10 counts (156 ns) played, of 8
the compare was written after the counter passed it; 12 is the floor
the peer keeps, and the lead before the first compare is 10 us (a 64-bit
division sat inside it once and the first edge came before the
sampler's first look - the bound is computed before the counter
restarts).

## Not covered yet

Declined with a reason (every field of chapter 33 is implemented):
- **The synchronous DATA path.** The master's CK, its polarity, its
  phase and LBCL are all measured on the pad; a synchronous LINK needs
  a second node on CK, TX and RX, and the wires this desk carries
  between its boards are the SPI and I2C links.
- **The synchronous SLAVE** (CR2.SLVEN, DIS_NSS, the underrun flag UDR):
  the register verbs exist on the resource, no task does, and neither is
  claimed to work.
- **Smartcard block mode.** BLEN + 4 characters did not raise EOBF on
  the loop, and 33.8.7 says why: the block counter is RESET while the
  USART transmits (TXE = 0), which on a wire whose transmitter is its
  own sender is always. A real card would settle it. The letter stages
  it and prints the outcome either way.
- **A LIN network.** The break is sent and detected and timed; the
  protocol layer above it (identifiers, checksums, a second node) is not
  brio's business yet.

Implemented, not bench-verified:
- `test_stm32_serial` as the G031K8's five group images (letters a to
  j, k to q, the host letters v, w, x and y, the two-board set's 1 to 8,
  and its 9, 0, r, s and u) and the G071RB's two (the single-board
  letters, the two-board set): the
  suite outgrew those parts whole, and the scores under "On the
  STM32G031K8" and "On the STM32G071RB" are the one image's; one run of
  each image on those boards measures them, the rate on the wire among
  them (no board is paired with either on the desk).
- The peer's `autobaud_tx` and the 0.5 / 1.5 stop codes as a SENDER:
  served, judged by no letter - the far F446 has no auto-baud detector
  to feed (its `send_mpcm`, its `stage`, its IrDA both ways and its
  `brk` are the far suite's letters `s`, `7`, `r`, `0` and `4`).
- `usart_peer` on the STM32G071RB: built, never run (not on the desk).
- The receive ring on the STM32G031K8: `test_stm32_dma`'s letter `o` is
  built for it and has not run there (the board is not on the desk); one
  run of the suite measures it. The ring under an `LpUart` runs
  (`test_stm32_dma` letter `x`); its kick is read, not measured
  ([lpuart.md](lpuart.md)).
- The wake from Stop on any instance but USART1 and USART2 (and the
  LPUARTs, docs/stm32g0/lpuart.md).
- The FIFO default on the STM32G031K8, and with it the paced receiver
  (RXFT, the time-out or IDLE for the tail) and letters `p` and `q`: the
  score under "On the STM32G031K8" is the one-register console's, and
  the LPUART1 three suites move to there runs in FIFO mode by default
  and is compiled, not run (the G031K8's own USART2 is BASIC and keeps
  the one-register transport, which `brio check stm32g0` proves at
  compile time); one run of `test_stm32_serial` on the board measures
  it.
- On the STM32G071RB, **the smartcard CK ladder, the synchronous
  master's CK census and both IRTIM counting legs** - every one an edge
  counter with no CPU, which is exactly the path ES0418 2.2.4 breaks on
  that part (above); and on the STM32G031K8 the console's kernel-clock
  legs and the wake from Stop, its USART2 being BASIC (above).
- `SyncHost`, `IrdaLink` and `Smartcard` under a `DynamicClock`: no
  `rebase`, refused at `init` by `clock_follows`. (The `Uart` on PCLK
  IS driven through the ladder as the dynamic clock's rebased user -
  byte-exact on its loop at 64, 16 and 2 MHz, [clock.md](clock.md);
  `init` reads the rate through `clock_hz(clock)`, a constant for a
  static clock.)

Findings left open, with what would settle them:
- **ES0548 2.11.1 at 9600 baud** (letter `W`): where 4800, 14400 and 19200
  share one map, 9600's noise region sits a sixteenth or two later and
  its corruption region is broken by clean cells that move between runs.
  A logic analyser on the link would say first whether the far engine
  draws each glitch where its `stage` op is told at that rate; only a
  drawing found exact leaves the receiver itself to explain.

The DMA half is bench-verified in `test_stm32_dma`, whose own console
carries both engines - so every verdict line of that suite left the chip
through a `DmaTxEngine` and every letter arrived through a `DmaRxEngine`
ring, and its letter `o` judges the ring on USART1's single wire
(docs/stm32g0/dma.md has the throughput table and the ST-LINK VCP's own
921600 ceiling).
