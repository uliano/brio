# LPUART (STM32G0)

> **PROVISIONAL.** Chapter 34 is implemented whole and most of it is
> bench-measured: both instances, all four kernel clocks, the twenty-bit
> baud generator against the manual's own two tables, the FIFOs, the
> prescaler, single-wire half-duplex, the shared vectors and the wake
> from Stop 1 on the 32768 Hz crystal. What keeps the banner is in "Not
> covered yet" - the features shared with chapter 33 that the USART's
> own suite exercises on a USART and not here (mute mode, character
> match, driver enable, flow control, the inversions), and the DMA
> engine slots, which compile and have never carried a byte on an
> LPUART.

Documents of record: RM0444 Rev 6 - LPUART ch. 34 (the implementation
tables 183/184 it shares with the USART, the baud generator 34.4.7 with
its tables 198 and 199, the low-power management 34.4.14, the registers
34.7), RCC 5.4.21 (LPUART1SEL at bit 10 and LPUART2SEL at bit 8 of
RCC_CCIPR) and the EXTI's table 65 (lines 28 and 35). Errata ES0548
Rev 3: NO ITEM NAMES THE LPUART; 2.2.4 reaches it in the same way it
reaches the USART, because both are clock-request peripherals (see
"Bench findings"). Driver: `stm32g0/lpuart.hpp` (`Lpuart<n>` resource +
`LpUart<n, pins, ...>`, which is `stm32g0/usart.hpp`'s OWN `UartTask`
named over this resource). The base, presence, bus clock, clock-select
position, vector and EXTI line come from
`stm32g0/device_tables.hpp`. Bench suite: `test_stm32_serial` letters n
(in `z`) and v (host-assisted, outside `z`). Family fixture
`test/family_stm32g0/lpuart.cpp` plus four negatives under
`tools/check_stm32g0.sh`.

## What the silicon does

**Chapter 34 is chapter 33 with three differences, and only one of them
is small.**

1. **The baud generator is another arithmetic altogether** (34.4.7):
   `baud = 256 x lpuart_ker_ck_pres / LPUARTDIV`, LPUARTDIV a TWENTY-bit
   number in `LPUART_BRR`, values below 0x300 forbidden, and fck between
   3 x and 4096 x the baud rate. Those two constraints are ONE
   constraint: 256 x 3 is 0x300 and 256 x 4096 is one past the field, so
   `lpuart_brr()` returning nothing IS both rules. It is not "the
   USART's divisor with a factor": it is fixed point with a legal window,
   and 256 x fck overflows 32 bits above 16.7 MHz (the arithmetic is done
   in 64 bits, once, at compile time for a static clock).
2. **The feature set is the `LP` column of table 184**: no synchronous
   mode, no smartcard, no IrDA, no LIN, no auto-baud, no receiver
   time-out, no Modbus - and NO OVER8, because with 256 sub-steps there
   is nothing for it to do. What IS there, and is the point of the
   peripheral: the FIFOs, the prescaler, all four kernel clocks and the
   wake from Stop 0/1 on LSE or HSI16.
3. **Every LPUART has a kernel-clock multiplexer**, where among the
   USARTs only the FULL ones do.

**Everything else is the USART's, and is REUSED and not copied.** The
device header gives both peripherals a `USART_TypeDef`; the enable rule
(a field 34.7.x calls "can only be written when the LPUART is disabled"
is written with UE clear or not at all), the flags in both register
views, the ICR twins, the ORE storm, the FIFO's per-entry error flags,
single-wire half-duplex, mute mode, character match, driver enable and
flow control are one implementation. `LpUart<n, ...>` IS
`UartTask<Lpuart<n>, ...>` - the same task `Uart<n, ...>` names over a
`Usart<n>` - and what differs the RESOURCE answers: `brr_for()`, the
capability flags, where the kernel-clock field sits, which vector,
which EXTI line. A second copy of that file would have been a second
place to get the ORE storm wrong.

**BOTH LPUARTs sit on APBENR1** (LPUART1 at bit 20, LPUART2 at bit 7) -
the header is the authority, and it is not what the USARTs' APB2/APB1
split invites one to guess.

**The vectors are shared, and not with each other.** On the G0B1 class
LPUART1 arrives on `USART3_4_5_6_LPUART1_IRQn` and LPUART2 on
`USART2_LPUART2_IRQn` - the console's own line. On the G071 class
LPUART1 shares `USART3_4_LPUART1_IRQn`; on the G031 class it has a line
of its own. An application writes ONE handler per line and calls each
`isr()` body from it.

**The wake lines are EXTI 28 (LPUART1) and 35 (LPUART2)**, both DIRECT:
no trigger to choose, no pending bit of the EXTI's own - the
peripheral's WUF is the pending state. Line 35 lives in the EXTI's
SECOND register group, which only the G0B1 class has and which
`stm32g0/exti.hpp` already handles.

## Types and verbs

- `Lpuart<n>` - the same surface `Usart<n>` offers, minus what the LP
  column has not got: `regs()`, `irq()`, `bus_clock`, `kernel_clock`
  (every instance has one), `configure(UartFormat, brr)` with a
  TWENTY-BIT divisor, `enable`/`enabled`, `brr()`/`set_brr()` (both
  wider than the USART's), `fifo`, `fifo_thresholds`, `prescaler`,
  `swap`, `invert`, `msb_first`, `half_duplex`, `overrun_disable`,
  `driver_enable`, `flow_control`, `mute_mode`, `character_match`,
  `wake_from_stop` + `wake_line`, `request`, `send_break`, the flags and
  their ICR twins, the interrupt enables, `dma_transmit`/`dma_receive`
  and the table-55 request ids - **LPUART1's are the two rows table 55
  calls `LPUART_RX` and `LPUART_TX` WITH NO INDEX AT ALL, 14 and 15, low
  down among the I2Cs and the SPIs and nowhere near the USARTs' block,
  while LPUART2 sits at 64/65 with the rest of the G0B1 class's
  additions** - and `reset()`. The four verbs the chapter
  has no field for answer honestly rather than lying:
  `oversampling(true)` returns false, `one_bit_sampling(true)` returns
  false (34.4.6: the receiver samples once a bit by construction, so
  there is no ONEBIT bit and no noise flag to lose), and `has_lin()`,
  `has_irda()`, `has_smartcard()`, `has_autobaud()` are constant false.
- `LpUart<n, pins, rx_size = 64, tx_size = 256, TxEngine, RxEngine,
  opts>` - every verb, every default and every option of `Uart`
  (docs/stm32g0/usart.md), because it is the same task.
- `lpuart_brr(hz, baud)` / `lpuart_actual_baud(hz, brr)` /
  `lpuart_min_hz(baud)` (3 x) / `lpuart_max_hz(baud)` (4096 x) -
  constexpr, and the fixture pins ALL SIX rows of table 198 and ALL TEN
  of table 199.

## How to use it

```cpp
constexpr brio::UartPins lp_pins{
    .tx = {'C', 1, brio::PinFunction::af1},   // LPUART1_TX, DS13560 table 17
    .rx = {'C', 0, brio::PinFunction::af1},   // LPUART1_RX
};
constexpr brio::UartOptions lse_opts{.kernel_clock = brio::UsartClock::lse};
using Low = brio::LpUart<1, lp_pins, 64, 128, brio::NoDmaEngine,
                         brio::NoDmaEngine, lse_opts>;

extern "C" void USART3_4_5_6_LPUART1_IRQHandler() { (void)Low::isr(); }

brio::RtcDomain::lse_enable(true);            // the kernel clock has to run
(void)brio::RtcDomain::lse_wait_ready(4'000'000UL);
Low::init(clock, 9600);                       // 9600 is the LSE's ceiling
```

A port that keeps working while the chip is stopped adds one option and
one obligation - the caller's, and the task cannot enforce it: no
transfer ongoing when Stop is entered, REACK checked after the enable,
and any DMA reception disabled first (34.4.14).

```cpp
constexpr brio::UartOptions wake_opts{
    .kernel_clock = brio::UsartClock::lse,
    .wake_from_stop = brio::UsartWakeSource::start_bit,
};
```

## Bench findings

Wireless, on LPUART1's own pad PC1 (AF1) and LPUART2's PC6 (AF3), each
proven free by its own pull first, and each looped back on itself
through CR3.HDSEL - the single-wire half duplex the USART's own suite
uses as its instrument. Letter v moves the WHOLE CONSOLE onto LPUART1
through PA2/PA3 AF6, which is the console's own pair reached by a
different alternate function.

**The twenty-bit divisor is the manual's, to the count.** All six rows
of table 198 (LSE, 32768 Hz) land in BRR: 27962, 13981, 6991, 3495,
1748, 874 for 300..9600 baud. Three of the six differ from the manual's
printed value by ONE, and in every case brio's is the nearer: this
driver rounds to nearest where the table truncates (874 gives 9597 baud
against 873's 9608, for a nominal 9600). All ten rows of table 199
(100 MHz) are pinned in the fixture the same way.

**9600 IS the LSE's ceiling and the arithmetic says so**: 19200 baud on
32768 Hz needs LPUARTDIV 437, below the 0x300 floor, and
`lpuart_brr()` returns nothing rather than a divisor the silicon
forbids.

**All four kernel clocks carry it**, byte-exact on the single wire:
PCLK 64 MHz at 115200 (BRR 142222), HSI16 at 115200 (35556) and at 9600
(426667), and the 32768 Hz crystal at 9600 (874) and at 300 (27962).

**The wire, not the divisor, is the ceiling.** 115200, 460800 and 921600
are byte-exact on the open-drain loop; 2 Mbaud and above are not, and
what fails is the rise time of the pad's own 40 k pull-up. 34.4.7's own
ceiling from a 64 MHz kernel is fck/3 = 21.3 Mbaud and this bench cannot
reach it.

**The FIFO and the prescaler are both there** (the LP column of table
184, and the silicon agrees - letter a writes FIFOEN and PRESC on every
instance of the part): FIFOEN byte-exact at 115200, PRESC = /64 at 9600
byte-exact. There is no OVER8 to choose.

**LPUART2 runs the same way** on its own pad, with its own LPUART2SEL
field and its own APB bit: 8 of 8 bytes at 115200, BRR 142222.

**CR2.CLKEN does not stick on either LPUART** while it sticks on all six
USARTs - the third authority on the one row of table 184 that is NOT the
FULL/BASIC split (docs/stm32g0/usart.md carries the finding).

**The shared vector reaches it**: LPUART1's interrupt was served on
`USART3_4_5_6_LPUART1_IRQn` by a handler that also serves four USARTs,
while the console on USART2 - which shares ITS line with LPUART2 - kept
talking throughout.

**A WHOLE CONSOLE MOVES ONTO IT AND COMES BACK** (letter v, through
`tools/uart_stress.py`): `LpUart<1>` on PA2/PA3 AF6, the same task and
the same verbs, clocked by the 32768 Hz crystal at 9600 took 1061 bytes
of the host's stream byte-exact with BRR 874, then on HSI16 at 115200
took 9280 bytes byte-exact with BRR 35556, and USART2 came back to print
the verdict - which is the other half of the proof.

**AND THE REASON THE PERIPHERAL EXISTS, measured**: with the console's
USART2 down and LPUART1 on the crystal with `wake_from_stop = start
bit`, a **Stop 1** ended at 589 ms when the host poked one byte - WUF
seen once, the RTC backstop never fired, and **the character that woke
it survived and read back 0xA5**. A start-bit wake ARRIVES BEFORE ITS
CHARACTER DOES, and at 9600 on a 32768 Hz kernel a frame is a whole
millisecond, so the byte has to be waited for; reading RDR the instant
the WFI returns reads an empty register and calls a working wake a lost
byte.

## Not covered yet

Driver gaps: none - chapter 34's every field is implemented.

Implemented, not bench-verified on an LPUART (all of them are the
USART's own code, measured there on a USART):
- Mute mode and the character match, driver enable and flow control, the
  swap and the three inversions, MSB first, the break request.
- The two DMA engine slots. They compile (the family fixture
  instantiates them and a negative refuses two engines on one channel)
  and no byte has ever moved through one on an LPUART.
- 7- and 9-bit words, parity and two stop bits.
- LPUART2 on any kernel clock but PCLK, and LPUART2 as a wake source
  (its EXTI line 35 is in the second register group; only LPUART1's 28
  has been used to wake this board).
- The G071 and G031 vectors and the single-LPUART parts: compile-only,
  pinned by the family fixture on all three headers.
