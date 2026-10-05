# USART (STM32G0)

Documents of record: RM0444 Rev 6 - USART ch. 33 (the implementation
tables 183/184, the baud generator 33.5.7, the tolerance tables 188/189,
auto-baud 33.5.9, mute mode 33.5.10, the character match 33.5.11, LIN
33.5.13, synchronous 33.5.14, single-wire 33.5.15, the receiver time-out
33.5.16, smartcard 33.5.17, IrDA 33.5.18, flow control and driver enable
33.5.20, low-power 33.5.21, the registers 33.8), RCC 5.4.21 (the CCIPR
multiplexers) and 5.4.1 (HSIKERON), the EXTI's table 65 (lines 24, 25,
26), the DMAMUX's table 55. Errata ES0548 Rev 3: 2.11.1 STAGED AND
REPRODUCED, 2.11.2 applied, 2.2.4 STAGED AND REPRODUCED on a USART wake
(see "Bench findings"). Driver: `stm32g0/usart.hpp` (`Usart<n>` resource
+ `UartTask` and its names `Uart<n, pins, ...>`, `Rs485`, plus
`SyncHost`, `IrdaLink`, `AutoBaud`, `Smartcard`); the instance, vector,
bus-clock, capability and EXTI facts come from
`stm32g0/device_tables.hpp`. The LPUARTs are
[lpuart.md](lpuart.md) - the SAME task over a different resource. Bench
suite: `test_stm32_serial` (17 letters in `z`, 95 verdicts, wireless;
three more outside `z` through `brio stress`). Family fixture
`test/family_stm32g0/usart.cpp` plus ELEVEN negatives under
`brio check stm32g0` (an instance nowhere, an instance off a part, a
pad used twice, two engines on one channel, FIFO mode on a BASIC
instance, a Reserved prescaler code, a Reserved threshold code, a wake
on PCLK, DE without a pad, and DE together with RTS).

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
puts the ceiling at kernel/16 and kernel/8. And what the divisor divides
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
inherits its neighbour's framing error.

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
byte it sends. The whole bench suite is built on it.

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
  `dma_transmit`/`dma_receive`, `dma_rx_request()`/`dma_tx_request()`
  (table 55's numbers, published by the peripheral that owns them),
  `reset()`.
- `UsartFlag` / `UsartClear` / `UsartInterrupt` - every ISR bit by BOTH
  names the register description gives it (`rxne` and `rxfne`, `txe` and
  `txfnf`), every ICR twin, every CR1 enable.
- `UartOptions` - the constexpr struct that is the task's LAST template
  argument: `kernel_clock`, `prescaler`, `over8`, `fifo` +
  `rx_threshold`/`tx_threshold`, `swap`, `invert_tx`/`invert_rx`/
  `invert_data`, `msb_first`, `half_duplex`, `one_bit`,
  `overrun_disable`, `driver_enable` + `de_pin`/`de_assertion`/
  `de_deassertion`/`de_active_low`, `rts`/`cts` + their pads, and
  `wake_from_stop`. Makers: `uart_half_duplex()`,
  `uart_with_driver_enable()`. `fifo` is a `UartFifo`, THREE states a
  bool converts to: stated on (`.fifo = true`, refused at compile time
  on an instance with no FIFO), stated off (`.fifo = false`), and
  unstated - the default - in which the task decides. `tx_threshold`
  defaults to `full_or_empty` (the transmitter refills an EMPTY FIFO),
  and `none` makes it ride TXFNF; `rx_threshold` defaults to `half` -
  the receiver paced by RXFT, its tail by the receiver time-out where
  the instance has one and by IDLE where it has not (an LPUART) - and
  `none` keeps RXFNE, a character an entry; `rx_timeout` is RTOR.RTO in
  bit times, 10 by default.
- `Uart<n, pins, rx_size = 64, tx_size = 256, TxEngine = NoDmaEngine,
  RxEngine = NoDmaEngine, opts = {}>` - `init(clock, baud, format)`,
  `isr()`, `dma_isr()`, `harvest()`, `write_byte`/`write_bulk`,
  `read_byte`/`read_bulk`, `read_span`/`consume` (the receive run in
  place; with a receive engine `consume()` answers whether the run was
  intact), `rx_skips` (the ring's skip epoch, zero without an engine),
  `rx_pending`, `tx_idle` (the wire's: nothing queued, no block in
  flight, TC set), `rebase(hz)`,
  `set_baud(hz, baud)`, `actual_baud`, `can_baud`, `min_hz_for`,
  `kernel_hz<Clock>()`, the counters (`rx_overruns`, `hw_overruns`,
  `frame_errors`, `parity_errors`, `noise_errors`, `dma_faults`,
  `wakes`), `clear_errors`, `release()`, and `fifo_mode` - the FIFO
  decision as the instantiation resolved it: the option where it is
  stated, otherwise ON where the instance has the FIFO
  (`has_fifo_mode`: a FULL USART, every LPUART) unless `wake_from_stop`
  is an address match, and `rx_paced` and `rx_tail` - the receiver's
  pace and the flag that ends a burst, as the instantiation resolved
  them. The public surface is
  IDENTICAL to avrdx's and samc21's, which is what lets
  `util/serial_port.hpp` and `print()` compile here untouched.
- `Rs485<n, pins, de_pin, assertion, deassertion, ...>` - the same task
  with the driver enable filled in. **`OneWire` is NOT a task here**:
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
one-register transport (`Serial::fifo_mode` says which).

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
| the default, FIFO, TXFT at empty | 515 | 331 | 41.6 |

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
| RXFT at RXFTCFG's level (33.8.4) | THE PACE of the FIFO receiver, at code 010 - half the FIFO: one entry drains four characters or more, and a handler five frames late loses nothing (four places and the shift register); three quarters would save a twelfth of an entry a character and leave three frames of slack, which the top rate (a frame is 320 cycles at 2 Mbaud) does not afford beside the image's longest handler |
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
| RXFT at half + the time-out (now), 115200 and 1 Mbaud | 64 | 335 | 84 |
| the same at 2 Mbaud | 51 | 384 | 77 |
| the receive engine (the DMA vector, a block of the transmitter's included) | 3 | 275 | 3.2 |

The paced drain is 24 instructions a character with no call - the ISR
load and its test, the RDR load and the ring's push inline; the error
flags are tested on the ISR word in hand and served off the
character's path - and an entry adds the prologue, the tail's ICR
store and the edge's two ring loads. The engine's edge costs one entry
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
brio's 84 - and it needs the length up front: its tail is taken one
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
  same condition.
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
  transmit threshold written but never armed.
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
byte-exact to 2 Mbaud** (16 of 16 at 115200, 230400, 460800, 921600, 1 M
and 2 M; 0 of 16 at 4 M) and what fails above it is the rise time of the
pad's own 40 k pull-up, not the generator.

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
- **The engine's edges**: 17 bytes told by IDLE 5799 cycles after the
  last stop bit; 200 bytes at 1 Mbaud into a 64-byte ring, read on the
  vectors' edges alone - 7 edges, 3 laps, no overrun.
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
  explained.

## On the STM32G071RB

`test_stm32_serial` runs on the Nucleo-G071RB (DEV_ID 0x460, REV_ID
0x2000) and scores **83/83** against the G0B1RE's 88.

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
and both IRTIM counts therefore skip by name.

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

`test_stm32_serial` scores **79 of 88** on the Nucleo-G031K8 (DEV_ID
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
- The receive ring off the STM32G0B1RE: `test_stm32_dma`'s console
  carries both engines on the STM32G071RB too, and its letter `o` is
  built for both smaller parts; none of it has run there (the boards are
  not on the desk), and one run of the suite on each measures it. The ring
  under an `LpUart` is the same task's and compile-only, as the LPUART's
  engine slots are ([lpuart.md](lpuart.md)).
- `Rs485` as a TASK (the driver-enable timings are measured through the
  resource on the DE pad; the task's own `init()` path is compile-only).
- The wake from Stop on any instance but USART2, and on Stop 1 (measured
  on an LPUART, docs/stm32g0/lpuart.md).
- The FIFO default off the STM32G0B1RE, and with it the paced receiver
  (RXFT, the time-out or IDLE for the tail) and letters `p` and `q`: the
  scores under "On the STM32G071RB" and "On the STM32G031K8" are the
  one-register console's,
  and the consoles that run in FIFO mode by default there - the
  G071RB's USART2, the LPUART1 three suites move to on the G031K8 - are
  compiled and not run (the G031K8's own USART2 is BASIC and keeps the
  one-register transport, which `brio check stm32g0` proves at compile
  time); one run of `test_stm32_serial` on each board measures it.
- The default under a wake on an ADDRESS MATCH: a Uart naming
  `wake_from_stop = address_match` with `fifo` unstated keeps the FIFO
  off, and no suite builds one - letter w arms that wake through the
  resource, with mute mode set.
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

The DMA half is bench-verified in `test_stm32_dma`, whose own console
carries both engines - so every verdict line of that suite left the chip
through a `DmaTxEngine` and every letter arrived through a `DmaRxEngine`
ring, and its letter `o` judges the ring on USART1's single wire
(docs/stm32g0/dma.md has the throughput table and the ST-LINK VCP's own
921600 ceiling).
