# An AVR64EA48 board

The REV 1.2 PCB of the self-built 48-pin design ("AVRxxx[DA|DB|EA]48 /
ATmega4809 - REV 1.2", the same board as [avr128da48.md](avr128da48.md))
fitted with an **AVR64EA48** (64 KB flash, 6 KB SRAM, 512 B EEPROM,
signature `1E 96 1E`, `SYSCFG.REVID` 0x21 = silicon revision B1), the
EA of the `avrdx/` stratum's bench. A board type names an ASSEMBLY, and
this one differs from the DA's on the same PCB in what is fitted.

- **Clock**: a **20 MHz quartz on PA0/PA1** for the EA's crystal
  oscillator (XOSCHF) - the "EXT CLK OPTS" solder jumpers in the quartz
  position; `BoardClock` is `Clock<ClockSource::crystal, 20'000'000>`
  with OSCHF claimed at 20 MHz (`avrdx/src/apps/avr_board.hpp`). A
  **32.768 kHz crystal on PF0/PF1**. Neither crystal's pins are ever
  driven by a program on this board, started or not.
- **Supply**: **5 V** (VDD jumper), measured 5.03 V by the probe. The EA's
  20 MHz needs it: CLK_MAIN reaches 20 MHz only from 4.5 V (5 MHz from
  1.8 V, 10 MHz from 2.7 V; DS40002443A 35.10.5) - at 3.3 V this board
  would have to run its clock at 10 MHz or less.
- **Fuses**: OSCCFG 0x00 (OSCHF at 20 MHz, the factory value), SYSCFG0
  **0xD8** (RSTPINCFG = RESET on PF6, UPDIPINCFG = UPDI on PF7, CRC off,
  EESAVE clear), SYSCFG1 0x07, WDTCFG 0x00, BODCFG 0x00, BOOTSIZE 0x00 and
  CODESIZE 0x00 (the whole flash is BOOT: nothing writes flash from
  software, and the EA's flash self-programming is declined - its
  storage is the EEPROM). **SYSCFG0 bit 4 (UPDIPINCFG) must never
  clear**: PF7 would start as a GPIO, and only a high-voltage pulse on
  RESET brings the UPDI back, which no probe of this desk produces.
  `brio fuses` refuses such a write on every EA before anything reaches
  the chip - the DA board's own `syscfg0=0xC8` included (bit 4 clear) -
  and refuses the CRC combinations DS80001048D 2.3.1 (rev B1) and 2.2.4
  (rev B2) forbid.
- **Console**: USB-C to a **CH340B** behind a digital isolator (an
  ADuM1201), reaching USART2 ALT1, **PF4 (TX) / PF5 (RX)**, through the
  "to UART" solder jumpers JP12/JP11 (the board's 6-pin "UPDI+SERIAL"
  header carries the same two lines). The CH340B's EEPROM names it
  product "AVR64EA48-REV1.2", serial **BRIO0002**, so its console is
  addressed by id (`/dev/serial/by-id`); 460800 in the console apps.
- **Probe**: an Atmel-ICE over UPDI on the "ATMEL ICE AVR" connector, at
  **firmware 1.46 or later**: 1.41 acknowledges every NVM version 3
  write and loses it (fuses read back unchanged, flash reads 0xFF) -
  [../probes/atmel-ice.md](../probes/atmel-ice.md).
- **LED and button**: the LED on **PF2**, the button on **PF3** (to
  ground, pulled up), each through a jumper.
- **PA7** carries 100 nF to ground, as on the DA's assembly: CLKOUT is
  never put there.
- **USART1 ALT1 jumpers**: PC4-PE0, PC5-PE1, PC6-PE2, PC7-PE3 - each
  pair ONE net, and PE0..PE3 run on to the partner board's wires (the
  SPI and USART links of the two-board suites). No program drives a PE
  pin and its PC partner at once: `pin_check` waves PC4..PC7 and leaves
  PE0..PE3 inputs with no pull, and the suites leave the unused half an
  input (`board_pc_pe_jumpers`).
- Manifest type `ea48`; the project is `avrdx/`
  ([../avrdx/README.md](../avrdx/README.md)).

## Bench findings

- Read over the debugger with the resident images (the console path
  being open, below): the 20 MHz quartz starts and is the main clock
  (`Clock<crystal>::init()` true, MCLKCTRLA EXTCLK, EXTS set, TIMEBASE
  20); `Clock<internal, 20 MHz, div4>` runs at 5 MHz with TIMEBASE 5 and
  PA7 toggling (`family_probe`); the console image configures USART2
  ALT1 (USARTROUTEB 0x01, BAUD 174 for 460800) and transmits, and sits
  in the kernel's idle.
- **PF4/PF5 reach no bridge**: PF5 floats (it reads 0, and 1 with its
  pull-up enabled) and stays flat while the host streams bytes into the
  CH340B, and nothing the board transmits arrives - the "to UART"
  jumpers JP11/JP12 are not closed on this assembly (or the isolator's
  board side is unpowered). Until they are, no suite's console runs.
- **The 32.768 kHz crystal does not oscillate**: XOSC32K enabled and
  forced on (RUNSTDBY, 64k cycles of start-up, LPMODE off) never sets
  XOSC32KS, and the RTC selected onto it never synchronizes its CTRLA
  (CTRLABUSY stuck), where the same RTC on OSC32K synchronizes at once -
  a joint, a crystal not fitted, or its load. The Ticker runs from
  OSC32K meanwhile (`preferred()`).
- Driven through the debugger instead of the console - a command byte
  pushed into the Uart's receive ring, the suite's tallies read back
  ([../probes/atmel-ice.md](../probes/atmel-ice.md)): `test_avr_pin`
  30 of 30, `test_avr_platform` 109 of 109 and `test_avr_power` 53 of
  53 (one verdict of its letter b red in two runs of ten - platform.md)
  at 20 MHz on the quartz, and the clock suite's letters that need no
  32 kHz crystal ([../avrdx/platform.md](../avrdx/platform.md),
  [../avrdx/port.md](../avrdx/port.md),
  [../avrdx/clkctrl.md](../avrdx/clkctrl.md)).
