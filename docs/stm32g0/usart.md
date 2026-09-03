# USART (STM32G0)

> **PROVISIONAL.** Chapter 33 is implemented WHOLE - every field of
> CR1/CR2/CR3/BRR/GTPR/RTOR/RQR/ISR/ICR/PRESC in both register views -
> and almost all of it is bench-measured on one board with no wires.
> What keeps the banner is in "Not covered yet" and every item there is
> DECLINED WITH A REASON or IMPLEMENTED-AND-NOT-STAGED, not missing: the
> synchronous DATA path and the synchronous SLAVE (they need a second
> board), smartcard block mode (it needs a card), and a LIN network.

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
suite: `test_stm32_serial` (15 letters in `z`, 88 verdicts, wireless;
three more outside `z` through `tools/uart_stress.py`). Family fixture
`test/family_stm32g0/usart.cpp` plus ELEVEN negatives under
`tools/check_stm32g0.sh` (an instance nowhere, an instance off a part, a
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
ignore it - the LPTIM campaign of this stratum found a forbidden write
landing anyway, so the refusal is the driver's.

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
for ever. **WUF IS THE SAME SHAPE ONE FLAG ALONG** and this campaign
paid for it twice: with UESM and WUFIE set, a WUF nobody clears makes
the vector re-enter until the watchdog reboots the board (caught by
halt-and-dump, IPSR 44 with ISR bit 20 standing). Every error flag has
its ICR twin and every handler here clears what it can see.

**RDR holds the last GOOD byte when ORE is set**; FE/NE/PE belong to the
byte in RDR, so a framed or parity-failed byte is dropped precisely and
a noisy one is kept and counted. **In FIFO mode 33.5.4 stores those
three flags WITH EACH ENTRY**, so a draining handler must read ISR
BEFORE every RDR or the attribution slides by one and a good byte
inherits its neighbour's framing error.

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
  turns it into divide-by-256), `fifo(bool)`,
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
  `uart_with_driver_enable()`.
- `Uart<n, pins, rx_size = 64, tx_size = 256, TxEngine = NoDmaEngine,
  RxEngine = NoDmaEngine, opts = {}>` - `init(clock, baud, format)`,
  `isr()`, `dma_isr()`, `harvest()`, `write_byte`/`read_byte`, `write`,
  `write_bulk`/`read_bulk`, `rx_pending`, `tx_idle`, `rebase(hz)`,
  `set_baud(hz, baud)`, `actual_baud`, `can_baud`, `min_hz_for`,
  `kernel_hz<Clock>()`, the counters (`rx_overruns`, `hw_overruns`,
  `frame_errors`, `parity_errors`, `noise_errors`, `dma_faults`,
  `wakes`), `clear_errors`, `release()`. The public surface is
  unchanged from the bring-up's and IDENTICAL to avrdx's and samc's -
  which is what lets `util/serial_port.hpp` and `print()` compile on the
  third architecture untouched.
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

The console, unchanged since the bring-up:

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

A one-wire link on a single pad, and a port that keeps its rate through
a clock change:

```cpp
constexpr brio::UartOptions one_wire = brio::uart_half_duplex();
using Bus = brio::Uart<1, u1_pins, 64, 64, brio::NoDmaEngine,
                       brio::NoDmaEngine, one_wire>;

constexpr brio::UartOptions on_hsi{.kernel_clock = brio::UsartClock::hsi16,
                                   .fifo = true};
using Steady = brio::Uart<1, u1_pins, 128, 512, brio::NoDmaEngine,
                          brio::NoDmaEngine, on_hsi>;
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

## The options cost nothing

`UartOptions` is ONE trailing NTTP with a default, and every member of it
is `if constexpr`-ed, so `Uart<n, pins>` compiles to exactly what it
compiled to before the parameter existed. That is not a hope: the md5
byte-identity gate on a pinned-mtime worktree of the previous commit
shows all FOURTEEN pre-existing STM32G0 release images byte-identical
after this chapter's whole tail was added - the console and
`test_stm32_dma` (whose own console carries both DMA engines) among
them.

Keeping that identity forced two shapes, and both are on record:

- `usart_brr()` and `usart_brr_over8()` are SIBLING VERBS. Giving
  `usart_brr()` a `bool over8 = false` third argument moved
  `test_stm32_dma` by forty bytes although the folded code for `false`
  is identical - the samc SPI-DMA campaign's ruling met again on this
  silicon: byte identity outranks API economy.
- The task keeps a `plain` constant for the default arrangement (PCLK,
  divide-by-1, no OVER8) and names HEAD's own expression under it,
  because folding `hz / usart_prescaler_divisor(div1)` to `hz` gives the
  same value and NOT the same code.

## The two optional DMA engine slots

`Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine, opts>` takes a
`stm32g0/dma.hpp` `DmaTxEngine` and/or `DmaRxEngine`; both default to
`NoDmaEngine`, which is a TAG and not a base class - `present` is all the
task asks about and it asks with `if constexpr`. `NoDmaEngine` and
`uart_engines_distinct()` live in THIS header rather than in `dma.hpp`,
which is the whole point of an optional slot: usart.hpp must not include
the DMA driver, or every program with a console would carry it.

- **CR3.DMAT / CR3.DMAR** are set in `init()` before the enable, and the
  matching INTERRUPT is NOT armed: the request and the interrupt are the
  same condition.
- **`dma_isr()`** is the body of whichever channel vector the engines
  report on; each engine reads only its own channel's flags.
- **`harvest()`** publishes received bytes: a receive block completes
  only when the buffer fills, which on an idle line may be never, so the
  owner ASKS. On this silicon the asking is one CNDTR read.
- **What is traded away** is per-byte error attribution: nobody reads
  ISR per character, so `harvest()` reads it once and counts what it
  finds.
- **`write_byte()` still nudges on a refusal** when a TX engine is
  present, because `print()` answers a false by trying for ever.

## Bench findings

`test_stm32_serial`, board E (Nucleo-G0B1RE), WIRELESS. Two facts make
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

**The console was moved under itself, and the verdict lines are the
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

**AND THE LOOP CANNOT SHOW WHAT A FIFO IS FOR, which is worth saying
rather than dressing up.** 256 bytes round the loop through the task
cost 258 interrupts without the FIFO and 257 with it: a single wire is
its own pacer, so each byte's transmit and receive events fall in the
SAME interrupt and there is never a second character waiting. One
interrupt a byte is the floor here whatever FIFOEN says. What the letter
does prove is that the FIFO COSTS NOTHING to turn on - no more
interrupts, not one byte different, the same public verbs and one option
between them.

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

### Host-assisted (outside `z`, `tools/uart_stress.py`)

**Streaming across the kernel clocks** (letter y): the console took the
host's stream byte-exact on PCLK at 115200, HSI16 at 115200, SYSCLK at
460800 and PCLK at 921600 - 4672, 4672, ~16000 and ~28000 bytes, ZERO
wrong, zero hardware overruns, zero framing errors - and again with
FIFOEN set under a transport that knows nothing about it.

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
  characters after it are not addressed to it.
- **ES0548 2.2.4 REPRODUCES ON A USART WAKE.** With HSIDIV = /4 the same
  Stop was NOT ended by the same poke - WUF never rose - and ran to the
  RTC backstop's full 1.4..2.0 s, with the RTC wake-up timer as the
  control that the Stop itself works. This is the first REPRODUCTION of
  2.2.4 in this project: the RTC campaign found it did NOT reach an RTC
  wake, and the difference is exactly the one the erratum names - the
  USART is a clock-request peripheral (33.5.21's `usart_ker_ck_req`) and
  the RTC is not.
- **AND HSIKERON DOES NOT RESCUE IT.** The same leg with RCC_CR.HSIKERON
  set - HSI16 kept running for its kernel-clock consumer, so no request
  is needed - still did not wake: WUF 0, the backstop firing. So 2.2.4
  reaches further than the request path. Recorded as measured, not
  explained.

## Not covered yet

Driver gaps: none. Every field of chapter 33 is implemented.

Declined with a reason, and they are the banner:
- **The synchronous DATA path.** The master's CK, its polarity, its
  phase and LBCL are all measured on the pad; a synchronous LINK needs
  something at the other end to clock, and this desk has one board.
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
- `Rs485` as a TASK (the driver-enable timings are measured through the
  resource on the DE pad; the task's own `init()` path is compile-only).
- The receive-threshold interrupt as a TRANSPORT pace (RXFT is measured
  at every code; the task deliberately keeps RXFNE, because a
  threshold-only receiver leaves the tail below the threshold unserved -
  RXFT plus the receiver time-out is the Modbus pattern and it is driven
  through the resource).
- The wake from Stop on any instance but USART2, and on Stop 1 (measured
  on an LPUART, docs/stm32g0/lpuart.md).
- The G071 and G031 instance sets: compile-only, pinned by the family
  fixture on all three headers.
- `usart_kernel_clock_hz` for a `DynamicClock` (the suite uses a static
  `Clock`).

The DMA half is bench-verified in `test_stm32_dma`, whose own console
carries both engines - so every verdict line of that suite left the chip
through a `DmaTxEngine` and every letter arrived through a `DmaRxEngine`
(docs/stm32g0/dma.md has the throughput table and the ST-LINK VCP's own
921600 ceiling).
