# An AVR64EA48 board

The REV 1.2 PCB of the self-built 48-pin design ("AVRxxx[DA|DB|EA]48 /
ATmega4809 - REV 1.2", the same board as [avr128da48.md](avr128da48.md))
fitted with an AVR64EA48 (64 KB of flash, 6 KB of SRAM, 512 bytes of
EEPROM, signature `1E 96 1E`, `SYSCFG.REVID` 0x21, which is silicon
revision B1): the EA of the `avrxt/` stratum's bench
([../avrxt/README.md](../avrxt/README.md)). A board type names an
assembly, and this one differs from the DA's on the same PCB in what is
fitted: a 20 MHz quartz for the EA's crystal oscillator, a 5 V supply
the EA needs for that rate, its own fuse set and a console bridge with
its own serial number. The page ends with what every suite measured on
this board.

## The board

- **Clock**: a 20 MHz quartz on PA0/PA1 for the EA's crystal oscillator
  (XOSCHF), the "EXT CLK OPTS" solder jumpers in the quartz position;
  the board's clock type `BoardClock` is
  `Clock<ClockSource::crystal, 20'000'000, ClockDiv::div1, 20'000'000>`,
  the last argument OSCHF claimed at 20 MHz
  (`avrxt/src/apps/avr_board.hpp`). A
  32.768 kHz crystal on PF0/PF1. Neither crystal's pins are ever driven
  by a program on this board, started or not.
- **Supply**: 5 V (the VDD jumper), measured 5.03 V by the probe. The
  EA's 20 MHz needs it: CLK_MAIN reaches 20 MHz only from 4.5 V (5 MHz
  from 1.8 V, 10 MHz from 2.7 V; DS40002443A 35.10.5), so at 3.3 V this
  board would have to run its clock at 10 MHz or less.
- **Console**: USB-C to a CH340B behind a digital isolator (an
  ADuM1201), reaching USART2 ALT1, PF4 (TX) / PF5 (RX), through the "to
  UART" solder jumpers JP12/JP11; the board's 6-pin "UPDI+SERIAL" header
  carries the same two lines. The CH340B's EEPROM names it product
  "AVR64EA48-REV1.2", serial BRIO0002, so its console is addressed by
  `/dev/serial/by-id`; 460800 baud in the console apps.
- **Probe**: an Atmel-ICE over UPDI on the "ATMEL ICE AVR" connector,
  at firmware 1.46 or later: 1.41 acknowledges every NVM version 3
  write and loses it (fuses read back unchanged, flash reads 0xFF),
  [../probes/atmel-ice.md](../probes/atmel-ice.md).
- **LED and button**: the LED on PF2, the button on PF3 (to ground,
  pulled up), each through a jumper.
- **Identity**: a label in the USERROW, written once over UPDI
  ([../avrxt/userrow.md](../avrxt/userrow.md)).

## The fuses

OSCCFG 0x00 (OSCHF at 20 MHz, the factory value); SYSCFG0 0xD8
(RSTPINCFG = RESET on PF6, UPDIPINCFG = UPDI on PF7, CRC off, EESAVE
clear); SYSCFG1 0x07; WDTCFG 0x00; BODCFG 0x4D, the BOD at BODLEVEL2,
continuous with the start-up wait awake and continuous asleep, which is
what guards the rev B1's NVM writes against DS80001048D 2.2.1, as
`test_avr_nvm` asserts ([../avrxt/nvm.md](../avrxt/nvm.md); written
with `brio fuses <board> bodcfg=0x4D`, which warns on an EA whose
BODCFG leaves them unguarded); BOOTSIZE 0x00 and CODESIZE 0x00, the
whole flash as BOOT, because nothing writes flash from software and the
EA's flash self-programming is declined, its storage being the EEPROM.

**SYSCFG0 bit 4 (UPDIPINCFG) must never clear.** PF7 would start as a
GPIO, and only a high-voltage pulse on RESET brings the UPDI back, which
no probe of this desk produces. `brio fuses` refuses such a write on
every EA before anything reaches the chip, the DA board's own
`syscfg0=0xC8` included (its bit 4 is clear), and refuses the CRC
combinations DS80001048D 2.3.1 (rev B1) and 2.2.4 (rev B2) forbid.

## What a program meets on this board

- **PD7 (VREFA) carries 100 nF to ground**, the reference's decoupling,
  as on the DA's assembly. PA7 carries nothing: CLKOUT runs there in
  `test_avr_clock` (`avr_board.hpp`'s `board_pa7_free`).
- **USART1 ALT1 jumpers**: PC4-PE0, PC5-PE1, PC6-PE2 and PC7-PE3 are
  each one net, and PE0..PE3 run on to the partner board's wires (the
  SPI and USART links of the two-board suites). No program drives a PE
  pin and its PC partner at once: the pin-check tool waves PC4..PC7 and
  leaves PE0..PE3 inputs with no pull, and the suites leave the unused
  half an input (`board_pc_pe_jumpers`).
- **The analog net**: PD6 (the DAC's output) is wired to PD1 and to PD7,
  with 10 kOhm from PD6 to ground, one net of three pins for the
  DAC-to-ADC and DAC-to-AC loops ([../avrxt/dac.md](../avrxt/dac.md)),
  stated in `avr_board.hpp` (`board_analog_net`, `board_dac_load_ohms`).
  The net carries 96 nF besides (its decay through the 10 k is 0.96 ms
  to 1/e, `test_avr_analog` letter n), which is PD7's 100 nF within its
  tolerance. A pad driving the net reaches VIH in 4.7 us and VIL in
  3.3 us, the DAC's buffer moves it in tens of microseconds, and a suite
  that drives PD1, PD6 or PD7 drives that net and waits for it.

## Flashing

`brio flash <board> <app>` writes the board through avrdude over UPDI;
the board type in the manifest is `ea48`, and the project is `avrxt/`
([../avrxt/README.md](../avrxt/README.md)). On this board type a suite
with a groups line builds as one image per group, `<app>-1`, `<app>-2`
and so on ([README.md](README.md), "How a board joins the bench").

## Bench findings

- The 20 MHz quartz starts and is the main clock
  (`Clock<crystal>::init()` true, MCLKCTRLA EXTCLK, EXTS set, TIMEBASE
  20); `Clock<internal, 20 MHz, div4>` runs at 5 MHz with TIMEBASE 5 and
  PA7 toggling (measured by the family probe); the console runs at
  460800 on USART2 ALT1 (USARTROUTEB 0x01, BAUD 174) through the
  isolated CH340B.
- **The 32.768 kHz crystal does not oscillate.** XOSC32K enabled and
  forced on (RUNSTDBY, 64k cycles of start-up, LPMODE off) never sets
  XOSC32KS, and the RTC selected onto it never synchronizes its CTRLA
  (CTRLABUSY stuck), where the same RTC on OSC32K synchronizes at once:
  a joint, a crystal not fitted, or its load. The Ticker runs from
  OSC32K meanwhile (`preferred()`). It is the one red of the board: five
  verdicts of `test_avr_clock` (the rulers timed against it) and two of
  `test_avr_rtc` (letter 9, XOSC32K as CLK_RTC).
- **Every suite image of the board, each run in its final form.** The
  partner is the SAM C21 board on the I2C node and the four link wires,
  and the partner idle on its console for the single-board keys (a key
  is one letter of a suite's console; `z` runs the single-board set and
  `y` the set that needs the partner):

  | image | key | partner | result |
  |---|---|---|---|
  | `test_avr_platform` | `z` | - | 109 of 109 |
  | `test_avr_pin` | `a` | - | 30 of 30 |
  | `test_avr_clock` | `a` | - | 22 of 27, the five the 32 kHz crystal's |
  | `test_avr_power` | `z` | - | 53 of 53 |
  | `test_avr_meter` | `z` | - | 38 of 38 |
  | `test_avr_rtc` | `a` | - | 89 of 91, the two the 32 kHz crystal's |
  | `test_avr_nvm` | `z` | - | 110 of 110 |
  | `test_avr_analog` | `a` | - | 158 of 158 |
  | `test_avr_timer-1`, `-2` | `a` | - | 141 of 141, 67 of 67 |
  | `test_avr_sleep-1` | `z` | - | 56 of 56 |
  | `test_avr_sleep-2` | `y` | `sleep_peer` | 47 of 47, three runs |
  | `test_avr_twi-1` | `z` | - | 226 of 226, ten runs |
  | `test_avr_twi-2` | `y` | `twi_peer` | 136 of 136, ten runs |
  | `test_avr_spi-1` | `z` | - | 180 of 180 |
  | `test_avr_spi-2` | `y`, `z` | `spi_peer` | 51 of 51, 20 of 20 |
  | `test_avr_spi-3` | `y` | `spi_peer` | 54 of 54 |
  | `test_avr_serial-1` | `z` | - | 163 of 163 |
  | `test_avr_serial-2` | `y` | `usart_peer` | 88 of 88 |
  | `test_avr_serial-3` | `y`, `w`, `x` | `usart_peer` | 30 of 30, 6 of 6, 3 of 3 |
  | the partner's `test_samc_i2c` | `z` | this board's `twi_peer` | 72 of 72, ten runs |
  | the partner's `test_samc_spi` | `z` | this board's `spi_peer` | 138 of 138 |
  | the partner's `test_samc_uart` | `y` | this board's `usart_peer` | 190 of 190 |
  | `bench_avr` | `r` `m` `p` `t` `e` `u` `i` | - | each "ran"; letter i's reads byte-exact in ten runs of ten |

  What each suite measured is its chapter's document:
  [../avrxt/platform.md](../avrxt/platform.md),
  [../avrxt/port.md](../avrxt/port.md),
  [../avrxt/clkctrl.md](../avrxt/clkctrl.md),
  [../avrxt/rtc.md](../avrxt/rtc.md), [../avrxt/nvm.md](../avrxt/nvm.md),
  [../avrxt/adc.md](../avrxt/adc.md), [../avrxt/dac.md](../avrxt/dac.md),
  [../avrxt/vref.md](../avrxt/vref.md), [../avrxt/tca.md](../avrxt/tca.md),
  [../avrxt/tcb.md](../avrxt/tcb.md), [../avrxt/ccl.md](../avrxt/ccl.md),
  [../avrxt/evsys.md](../avrxt/evsys.md), [../avrxt/twi.md](../avrxt/twi.md),
  [../avrxt/spi.md](../avrxt/spi.md), [../avrxt/usart.md](../avrxt/usart.md),
  [../samc21/sercom.md](../samc21/sercom.md); the costs `bench_avr`
  prints at 20 MHz are in [../design/benchmark.md](../design/benchmark.md).
- The timer suite's pads on this board: TCA0 on PORTD (PD0, PD2), TCA1
  on PORTB, PORTC (PC4..PC6, the jumpered pins, nothing driving their PE
  partners meanwhile), PORTA (PA4..PA6) and PORTD (PD4..PD6), the CCL on
  LUT2's PD0/PD2/PD3, the DAC on PD6, EVOUT on PD2, PC7 and PA7 (each
  read 5 us after a change). The analog net's 96 nF sets what a pad
  driving it sees: letter c of `test_avr_timer` takes its second LUT
  input on PD2 rather than PD1, and letter x waits 30 us before it reads
  PD6 under TCA1's PORTD route.
- No regulator start-up at any wake; OSCHF restarts in 9.2 us, this
  board's 20 MHz quartz in 0.61..0.67 ms from cold; one edge of any of
  ten pins wakes the chip from power-down (`test_avr_sleep`,
  [../avrxt/platform.md](../avrxt/platform.md)). The BOD runs
  continuously asleep by this board's BODCFG, its current in every
  sleep.
- `test_avr_sleep` letter p (in `test_avr_sleep-1`) drives PC7 at 32 Hz,
  which is the echo net into the partner. Run with the partner on its
  `sleep_peer`, the partner's first capture after it reads stale (letter
  h's printed "arming to that edge" figure) and every train after it
  right. The peer drains both capture stages before it arms; why a stamp
  older than the arming survives that is not explained, and a peer
  variant printing CC1, STATUS.CCBUFV1 and INTFLAG at the arming, after
  letter p, would tell.
- **The TWI client's data setup** (the dual loop of `bench_avr` letter i
  and `test_avr_twi`): at SDASETUP four clocks a Smart-mode client loses
  a long read where its stretch must end with SDA falling; the driver's
  default is eight ([../avrxt/twi.md](../avrxt/twi.md), "The client's
  data setup", recorded on this node by the partner board).
- The two crystals differ by about 110 ppm: the partner's reads 109 ppm
  slow against this board's 20 MHz quartz (`test_avr_serial` x).
