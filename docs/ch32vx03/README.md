# Target: CH32V203 and CH32V303 (`ch32vx03/`)

The operational page for the CH32V203 and CH32V303 target: WCH's
QingKe V4B, a RISC-V core with the **RV32IMAC** instruction set -
thirty-two registers, a multiplier and a divider, the atomic extension,
user mode and a 64-bit system counter - on the CH32V203C8T6 of the bench (64 KB
flash, 20 KB SRAM, 144 MHz), and the **QingKe V4F** of the CH32V303,
the same core with a single-precision floating-point unit
(**RV32IMAFC**), on the CH32V303VCT6 (256 KB of zero-wait flash, 64 KB
SRAM, 144 MHz) - the reference manual's CH32V30x_D8, the third device
class of this stratum beside the CH32V203's two. brio's second WCH
family and its second RISC-V one, and the fact this target exists to
keep true is the same as every other: the kernel and util strata
compile here unchanged.

Two things shape the stratum and are stated up front. **There is no
vendor header in the build**: WCH ships its register definitions inside
the EVT package, whose licence is written for software running on WCH
parts, so [brio/ch32vx03/device.hpp](../../brio/ch32vx03/device.hpp)
carries the map, read off the reference manual and answerable to it -
and per-part variability, which the ST strata ask the device header
for, is stated in [parts/](../../brio/ch32vx03/parts/). And **this is
not the CH32V00x with a bigger core**: the peripheral generation is a
different one, the STM32F1's under WCH's names - sixteen pins a port
with the F1's two-bit MODE, a USB device controller that is ST's
register for register on the CH32V203 (the CH32V303 carries WCH's own
host/device block instead), a bxCAN, an F1 timer set - where the
CH32V00x is a reduced design of its own.

## The documents

One document per peripheral driver is the shape
[../README.md](../README.md) prescribes, and on this target every
driver has its page; what this page adds is what the silicon taught the
stratum across the chapters.

| Driver | State |
|--------|-------|
| [device.hpp](../../brio/ch32vx03/device.hpp) + [parts/](../../brio/ch32vx03/parts/) | the register map and the part table: thirteen parts under three device classes - the CH32V203's nine in the CH32V20x_D6 and the CH32V20x_D8, the CH32V303's four in the CH32V30x_D8 - each with its memories (the zero-wait window, the tail above it and the whole array), its bonded pads, its instances, its core's FPU or none, its vector table's tail and the interrupt lines its class has not got |
| [platform.md](platform.md) | Platform: `Ch32vx03Platform` (the csrrci critical section, the WFE-shaped `idle()`, `ebreak`, the `.noinit` breadcrumb), `Pfic` and the one handler attribute `BRIO_CH32_INTERRUPT` (the core's hardware prologue MEASURED on the CH32V203C8T6: 53 cycles of round trip against 63, 152 bytes of flash and SIXTY-FOUR BYTES OF USER STACK the internal hardware stack carries instead - which is what parts this core from the CH32V00x's), the 64-bit STK `BasicTicker` (its CNT arithmetic 1 ppm against the interrupt count over 200 reloads) and `delay_us` on that counter (100 us in 14434 cycles of 14400 asked, a tick period and above refused), corecfgr's 0x1F measured at two cycles in 36811, and the tick rate against the host's clock - the HSI half a per cent fast where the board's crystal is exact; then the failing half, [reset.hpp](../../brio/ch32vx03/reset.hpp): the six flags as the history they are, `Reset::software()` through the core's keyed PFIC_CFGR reading back as SFTRSTF alone, `ResetReporter`, `fault_reset<P>()` carrying the cause the core left in mcause, and the ebreak that lands on the BREAKPOINT vector and not the exception one; three real resets in the suite. On the CH32V303VCT6 the same letters (a round trip of 50 to 66 cycles, corecfgr's bits SLOWER there by half to eight tenths of a per cent) and the V4F's FLOATING-POINT UNIT: off at reset and switched on by the crt of an image built with F, left Dirty by the crt's own fcsr write, recording its exceptions and never trapping on them, twenty float accumulators exact through 19671 interrupts whose handler does float work of its own, and a round trip of 56, 67 and 101 cycles for a handler with no f-register, with float work, and calling out - the hardware prologue saving integers only and the compiler all twenty caller-saved f-registers once a handler calls a function; and THE BUS IN SLEEP asked again of that die, a DMA block moving 37 words across 1.9 ms of idle() against 48010 awake; and THE MASK'S SHADOW, the question behind the manual's V2.5 note asking for a fence.i after a mask: none behind the guard's csrrci (0 of 5000 races taken after it, with or without a fence.i, with or without the hardware prologue) and up to three instructions behind a store into a line's own PFIC_IRER (750 races of 750), which a fence.i does not close |
| [clock.md](clock.md) | RCC and the EXTEN bits that belong to the tree: the two high-speed roots, the PLL whose input divider is per DEVICE CLASS (the HSI's in EXTEN, the HSE's dividing by one or two here and by four or eight on the CH32V203RB) and whose input and output RANGES are part facts, the whole prescaler table with PB1 capped and the timers' doubling rule stated, the USB divider written before any USB gate can open, the ADC's divider - the one place a legal rate leaves a peripheral out of specification - the LSI, the clock security system as the non-maskable interrupt's body, the ready interrupts, the output pad two packages have not got, and the peripheral gates; `Clock` static and `DynamicClock` over a pack of rate tuples, every switch parking on the HSI. Measured on the CH32V203C8T6: ten trees entered, read back and bracketed against the host's clock (every HSI-rooted rate 0.42 to 0.46 % fast, every crystal-rooted one within 0.05 % of exact), the pack walked up and down with the tick and the console rebased at each step, the LSI ready 3.1 ms after LSION and the board's crystal 1.7 ms after HSEON with its ready interrupt reaching the RCC vector, the HSI stopped under a tree that runs off the crystal, the security system armed over a healthy crystal, and twenty-six gates opened and closed - the bits of blocks this part has not got reading back zero. On the CH32V303VCT6 the same ten trees and 49 verdicts: the HSI-rooted rates a third of a per cent fast and the crystal's within a tenth, the LSI ready 4.4 ms after LSION, the board's crystal 1.05 ms after HSEON, and forty-two gates - with the USB DEVICE CONTROLLER'S bit among those that read back zero, that part having no such controller |
| [pin.md](pin.md) | GPIO, the remaps and the EXTI: the F1's four-bit nibble over two registers with a MODE that carries a speed, the pull that lives in the output register and belongs to input mode alone, the whole-port verbs and the one-way configuration lock, over the five ports the LQFP100 bonds; the remap columns of AFIO_PCFR1/PCFR2 as constexpr pad tables judged by the DEVICE CLASS and by the package's bonding - the CH32V303's own among them (USART1's four columns, UART4's other table, TIM8/TIM9/TIM10, UART5..UART8, SPI3 with I2S3, the ADC trigger remaps, FSMC_NADV), CAN2's and the Ethernet's refused - the debug port's own field read and never written, the event output whose source no document of this family names; and the twenty-two EXTI lines - sixteen pin lines through AFIO_EXTICR with the one-pad-per-line rule refused by `select()` and overridden by `steal()`, the PVD's, the RTC alarm's and the peripheral wake-ups each part's table states, the senses, the two enables, the software trigger and the write-one flags over five single vectors and two shared ones. Measured: all fifteen configurations read back in both registers on both parts, the LQFP48's unbonded nibbles reading ZERO where the LQFP100's full port reads the manual's value, every free pad of ports C, D and E driven and pulled both ways, an open-drain output that drives nothing high, nine remap columns written and restored on the CH32V203C8T6 and thirty-one on the CH32V303VCT6, two fields the CH32V203 holds at zero against the manual's own notes and the CH32V303 takes, five/five/ten edges of a pad on its own line, an edge over a wire reaching its handler in 17 to 20 cycles, and a line event ending idle() in 15 and 34 core cycles |
| [tim.md](tim.md) | The timers: TIM1 and TIM2..TIM4 on every part, a 32-bit TIM5 on the CH32V203RB, and on the CH32V303RC and VC the whole chapter set - three more advanced timers (TIM8, TIM9, TIM10, four unshared vectors each), a sixteen-bit TIM5 and the two basic timers TIM6 and TIM7, on which a channel verb does not compile; the F1's register file under WCH's names - the two shadow registers, the four channels in both faces with CCyS writable only off, the slave controller with its three encoder modes and the master TRGO, the internal trigger table folded through what the part HAS, the advanced timers' repetition counter, complementary outputs, dead-time generator and break input, the DMA burst engine and the class's DMA2 requests as data, and the CH32V30x_D8's dual-edge capture as a verb that ASKS the die; the pads taken from afio.hpp's remap columns, and the nine tasks. Measured on both parts: the counter and both shadows against the core's counter, a PWM captured by its own timer to the microsecond at four duties, a dead band of 27.9 us against 28 asked, the interval and period meters behind a MeterLatch, two timers counting each other over an internal trigger (201 updates, a 25 % duty gated to 5000 us), a one pulse 501 us wide for 500 asked, forty encoder counts for ten quadrature cycles, a pad in PLAIN OUTPUT mode reaching a capture input and a channel's output stage cut off from its pad in an encoder mode; and on the CH32V303VCT6 a second timer reading the first over a wire (999 and 299 us for 1000 and 300), TIM8's PWM captured by TIM4 to the count at nine duties and rates up to 100 kHz, TIM8's dead band (27.8 us), four vectors and break from BKIN, TIM9 and TIM10 with no wire, TIM5 sixteen bits wide, TIM6 and TIM7 keeping the core's time with their TRGO counted on TIM9, nineteen internal-trigger links counting 201 each, TIM3's external trigger on PD2 in both external clock modes, a break held at its level re-raising its flag without end, and a die whose lot has no dual-edge register |
| [watchdog.md](watchdog.md) | The two watchdogs, the same blocks on every device class: the independent one on the LSI (three keys, no way back but a reset, the oscillator forced on - and its two registers taking a write only while that oscillator RUNS, which is why arm() starts before it configures and ends with the refresh that re-locks them) and the window one on PCLK1/4096/2^WDGTB (the counter that does not run unarmed against its own chapter, the clock gate under which the registers still read their reset values and drop a write, the window whose early refresh IS the reset, the early wake-up one tick before it, the block's reset line as the only way back). Measured on the CH32V203C8T6 and the CH32V303VCT6 with the same verdicts: the tick at three prescalers to the microsecond (2628/5347/21389 us for 47 ticks), EWIF at 28671 us of 28672, three real WWDG resets (29 ms, 0 ms for an early refresh, 29 ms with the interrupt having run once) and the IWDG's own at 206 and 207 ms on the CH32V203C8T6 and 199 ms twice on the CH32V303VCT6 - which puts those dies' LSIs at 38.8 and 40.2 kHz |
| [dma.md](dma.md) | The DMA controllers: ONE of eight channels on the CH32V203 and TWO on the CH32V303 - a DMA1 of seven and a DMA2 of eleven whose last four report in a flag pair of their own - where THE CHANNEL IS THE REQUEST: no multiplexer, the tables wiring each peripheral event to one channel of one controller, so a request lands in a SLOT, controller and channel, and every type names its controller first; the PART's own table deciding which rows exist, and a channel's rows an OR - a peripheral left with its DMA bit set drives whatever the channel holds; the configuration word, the count read-only while the channel runs, the addresses the silicon would align in silence and this driver refuses, the CH32V303 DMA1's 64 KB boundary refused on every lot, four flags and one vector a channel, blocks with no reset line in RCC, and `any_enabled()` asked of both controllers; then the four engines on either controller. Measured on both parts: six cycles an item at all three widths (23.8, 47.1 and 92.3 MB/s at 144 MHz), the half flag at 2049 and 2050 of 4096, every configuring verb refused under a running channel, five addresses outside the map completing as ordinary blocks instead of raising the promised transfer error, a circular run reloading itself at a timer's pace, a staircase of another timer's counter 500 us a step, the burst engine's four compares, the software priority beating the channel number and the channel number beating a head start, and THE SLEEP - a handful of bytes a wake; and on the CH32V303VCT6 DMA2's eleven channels copying out of flash with 8 to 11 in the extended pair, all eighteen vectors each the channel's own, TIM5's, TIM6's and TIM7's updates as DMA2 requests, both block engines on DMA2, DMA2 starving in Sleep as DMA1 does, UART4's engines on DMA2 against USART2's on DMA1 byte for byte over the board's crossed pair, a receiver left running serving a timer's transfer sixteen items at once - and THE 64 KB RULE REAL ON THAT DIE: with the refusal bypassed, DMA1 read the upper half of a span across 0x0801 0000 from the bottom of the same page |
| [adc.md](adc.md) | The two converters: the STM32F1's ADC under WCH's names with an input buffer and a gain of 1, 4, 16 or 64 in front of it - how many converters and bonded channels a part has (two and nine or ten up to the CH32V203C8, ONE and sixteen on the 128 KB part, two and ten or sixteen on the CH32V303), the ONE PLACE A LEGAL CLOCK TREE LEAVES A PERIPHERAL OUT OF SPECIFICATION (ADCCLK is PCLK2 over 2, 4, 6 or 8 against a 14 MHz rating, refused at compile time above 112 MHz of HCLK), the calibration and the order that hides in BUFEN's note, the regular group of sixteen and the injected four with a signed offset, the eight sampling times against the source impedance each settles and a conversion 12.5 cycles longer than its sampling time - the datasheets' figure, not the manual's 11 -, the analog watchdog, code 110 an EXTI LINE that the CH32V303RC and VC can hand to TIM8, the DMA request as a block stream, the dual modes, one vector for both units, and the CH32V303's LOT-KEYED additions - four short sampling times, ADC2's own DMA request, a 75 % ADC clock - each ASKED OF THE DIE. Measured: the calibration at 8 us leaving 2049 (3 us and 2044 on the CH32V303VCT6), VREFINT at 1489 counts = 3301 mV, the sensor at 1404 mV, a 40 kOhm pull read 1060 counts at 1.5 cycles and 4095 at 239.5 in a scan - full scale at every sampling time when the multiplexer is parked on the CH32V203C8T6, a quarter of the scale however long it is parked on the CH32V303VCT6 -, four blocks lent through a relay in 679 us with the overrun staged, two or three items past a half flag before the fastest handler can stop the channel, a timer's TRGO pacing 399 conversions in 200 ms, AN EXTI LINE REACHING THE CONVERTER THROUGH ITS EVENT ENABLE AND NOT ITS INTERRUPT ENABLE, both converters' data in one register, a discontinuous sequence stepping 2, 4, 6 with one EOC; and on the CH32V303VCT6 the 12.5-cycle tail at four sampling times, the part's own DAC as the source - thirty-one codes on a line of gain 0.9984 with no residual above 2.6 counts, the PGA at 4.01, 16.18 and 65.79 -, code 110 handed to TIM8 and paced by it at 200 conversions in 100 ms, and a die whose lot has none of the three lot-keyed additions |
| [dac.md](dac.md) | The CH32V303's digital-to-analog converter: two 12-bit channels on PA4 and PA5, which are also the ADC's inputs 4 and 5, so the converter reads the DAC through the bond pad they share; nine spellings of a holding register that is not the output register, the buffer DISABLED by a one, table 17-1's eight triggers with the absent timers refused, the noise and triangle generators, the DMA request a HARDWARE trigger raises on DMA2's channels 3 and 4 handed to a block engine in one verb, the dual register fed by one request - and no status register, no interrupt and no underrun. Measured on the CH32V303VCT6, read back by the ADC: six placements in one register per channel, seven codes on each pad within 13 counts buffered and 10 unbuffered with the rails at 0 and some 10 mV under the supply, the TRGO of all six timers holding a datum until the update, EXTI line 9 REACHING THE CONVERTER THROUGH ITS EVENT ENABLE as the ADC's does, the noise generator equal to figure 17-5's register for sixty-four triggers at three widths, the triangle turning at both ends, a table played for ever by the loop engine on DMA2's channel 3 and found on the pad level by level, both channels fed a word a trigger from one request, and each channel following its OWN configuration with both enabled - V2.3's note otherwise does not hold |
| [opa.md](opa.md) | The operational amplifiers - two on the CH32V203, four on the CH32V303: FOUR BITS EACH in one register inside the EXTEN block's window - an enable, one of two positive pads, one of two negative ones, one of two outputs - and nothing else: no key, no lock, no gain, no internal feedback, so with no wire strapped the block is an open-loop stage, a comparator. Every first output pad is an ADC input pad; the CH32V303's second outputs are port-E pads no converter reads, and its high-speed bits live in EXTEN_CTR2 - a lot's register WHOSE ADDRESS MIRRORS EXTEN_CTR ON A DIE WITHOUT IT, so the driver reads the word before it writes a bit; the twenty-pin CH32V203 has OPA2 alone. Measured on both parts: saturation to the rail and to ground on every amplifier and input pair, a selection moving the level to the new output pad and off the old, the high rail 1 to 4 mV under the supply, the four-bit fields independent, the amplifier winning against its pad's 40 kOhm pull; and on the CH32V303VCT6 OPA3 and OPA4 on both input pairs, the four port-E outputs read as levels both ways, a differential step followed in about a microsecond, and a die with no EXTEN_CTR2 whose address read EXTEN_CTR's word |
| [usart.md](usart.md) | The serial ports: four instances on two peripheral buses, every divisor asked of the instance's OWN bus, and WHICH instances a part offers taken from the datasheet's table; the frame in every shape M and STOP allow, mute mode with both wakes, LIN's break, single-wire half duplex, IrDA, THE SMARTCARD and THE SYNCHRONOUS CLOCK - which exist here because the fourth serial port of this part is a USART4 and not a UART4, the chapter's own exception - the flow-control pair, the two DMA requests, every flag with the sequence that clears it and both interrupt sources; the pads are afio.hpp's COLUMNS and a remap code of 0 writes no register at all; then the transport, with its frame, its two engine slots and its trailing options. Measured: twenty-eight baud rows within one per mille on both buses with 1200 refused from the 144 MHz one (the register's sixteen bits, not the generator's floor), a start bit that is the divisor to the clock at four rates from 9600 to 3 Mbaud, the word length as a frame's low run, THE TWO HALF STOP CODES COMING OUT AS WHOLE ONES, TE's idle frame at 1146 us with TXE back in 104 us on an idle transmitter, PE delivering its byte and ORE keeping the data register's, both mute wakes with 18.10.4's two notes, a break of ten bits outside LIN and thirteen inside it with the detector at ten and eleven, THE INFRARED DECODER TAKING THE MIRROR OF ITS OWN ENCODER'S PULSE, seven clock pulses a byte with LBCL clear and eight with it set - the F1's sense and not this manual's words - a card clock of 6001500 Hz for the 6 MHz asked and only with the transmitter enabled, and the CTS hold with the RTS hand |
| [spi.md](spi.md) | The two synchronous serial ports: the STM32F1's SPI under WCH's names, with the two instances on TWO BUSES so one BR code is two frequencies (SPI1 divides PB2 = HCLK, SPI2 divides PB1) and a program asks the INSTANCE; no FIFO at all - one buffer each way, which is what makes a host's pump run on RXNE and a client answer ONE FRAME AHEAD - the four modes, both frame widths, both bit orders, the three chip-select arrangements, the simplex and one-wire line modes, the hardware CRC, the two DMA rows a channel each, the pads as afio.hpp's COLUMNS (SPI1 has two, SPI2 no remap field at all), the high-speed read mode confined to one BR code on this device class, and no I2S: the datasheet gives this series none; then the host engine with the other strata's Request verbatim, the client with its dark listener, and a pad-slew verb on both. Measured: eight rising edges per frame at every BR code, counted by a timer on the SPI's own clock pad; a strap clean to 36 MHz and breaking at 72; four kilobytes byte-exact; the hardware CRC equal to a bitwise reference; the engines at 36 core cycles a frame against the wire's 32; the arbiter's four transactions, its rejection and both votes - and against a SECOND BOARD on SPI2's four wires: all four modes and both bit orders exact both ways, the dummy byte of a client that did not preload, a link exact to 1.125 MHz whose break is the far end's ANSWER RELOAD and not the wire, the roles inverted with this chip as the client, a ten-second stress with no error at either end, and TWO THINGS THE CHAPTER DOES NOT SAY - that 20.2.7's recipe for clearing MODF does not clear it (fourteen sequences measured, WCH's own library on the same board among them; SPE never takes while the flag stands, and only the block's reset line puts it down) and that the CLIENT'S ANSWER EDGE is a correctness parameter in the two modes that sample on the falling edge |
| [i2c.md](i2c.md) | The two-wire ports: the STM32F1's I2C under WCH's names, host and target, with a rise-time register the CH32V00x's block has not - so the SCL timing is THREE registers (FREQ, CKCFGR under F/S and DUTY, RTR from the WIRE's rise) and the chapter's own ceiling is FREQ's six bits: 4 to 60 MHz of PB1, which is the one peripheral of this family that cannot be used at the top of the tree. The event machine's every sequence, the receive procedure by count, 7- and 10-bit own addresses with the dual address and the general call, ACK and POS, SMBus and PEC as bits, the two DMA rows a channel each (a one-byte read stays on the pump), two vectors an instance, the pads as afio.hpp's COLUMNS with the SMBus alert pad outside them; then the host engine with the other strata's Request verbatim, the arbiter over it, and the client with a polled option and a flush(). Measured: both speeds exact at 48 MHz of PB1 and SCL read off its own pad by a timer (9989/2489/2593 ns of period, the 40 ns each high half is short of telling the wire's rise), NO enable protection at all - CKCFGR, FREQ and RTR all take a write with PE set, against 19.12.9's own sentence - an absent address as i2c_nack_addr with BUSY back in 6 us, a held SDA answered i2c_arb_lost by the silicon and unstick()'s three clocks counted on the pad, the four receive procedures byte-exact against a second chip, both duty shapes and both speeds byte-exact (the wire carrying 87 and 251 kHz, the target's stretch and not the register's rate), both DMA engines, the kernel's arbiter with a held clock answered i2c_timeout at its 20 ms limit, ARBITRATION IN BOTH DIRECTIONS and this board as the far controller's target, 25100 tenures in ten seconds with no error at either end - and two facts a recovery has to know: a DMA-served read can leave BUSY standing over an idle wire (19.12.1's own case for SWRST, and SWRST takes CTLR2's interrupt enables with it), and clearing PE does NOT let a stretching machine's pads go, only the block's reset line does |
| [nvm.md](nvm.md) | The flash memory, the user option bytes and the electronic signature: TWO programming methods and FOUR erase grains in one chapter - a half-word behind PG and a whole 256-byte page behind FTPG, with 256 bytes, 4 KB, 32 KB and the whole user array as the things that can be erased - three locks each wanting its own key pair in order, a wrong one holding until the next system reset, and AN ERASED PATTERN THAT IS NOT ALL ONES (0xE339E339, stated four times in the chapter and the opposite of what the sister family promises); the rate as part of the contract, because the access clock may not exceed 60 MHz and SCKMOD already halves the system clock, so above 120 MHz an erase or a program wants HCLK divided around it - which the engine REFUSES rather than doing behind its caller's back, at compile time under a static clock and with a code under a dynamic one - the enhanced read mode that would fail an erase in silence, the option bytes decoded READ-ONLY with RDP written by no verb, and the signature; on both series the array is a zero-wait WINDOW with a non-zero-wait TAIL above it - (224K - R0WAIT) on every CH32V203, (480K - R0WAIT) on the CH32V303RC and VC - which both methods write and which a read reaches through the array's own address under the same rate contract, the option byte's memory split is decoded read-only (FLASH_OBR holding the USER byte whole at [9:2]) and ESIG_FLACAP is the largest window the split can select; plus MainFlash, the FlashMedia in the last 4 KB of every part's zero-wait window (one write-protection unit, the page as the cell, the clock in the medium's own type) with no heap and no journal above it. Measured on both parts: THE CORE RUNNING OUT OF THE ARRAY THROUGHOUT an erase, the kernel's tick advancing by what the ruler measured, so an erase is a wait and not a stall; the erased pattern on the zone and in the third word of each die's own identifier; A HALF-WORD CELL OF THE WINDOW TAKING PASS AFTER PASS between erases, each read back exactly, where a page programmed twice with no erase holds neither pattern nor their AND; a wrong key locking the engine until the next reset and raising NO bus error; the enhanced read mode never engaging; FLASH_ADDR not needed by the page program; SCKMOD written and read back in both positions with neither an instruction fetch nor a data read of the window measurably faster at 48 MHz; an erase at 144 MHz refused with a code and nothing written; and the medium's cell erased, programmed, read back and found again after a reset. On the CH32V203C8T6 a page erased in 9770 us and programmed in 1540 and a half-word in 2603; on the CH32V303VCT6 EVERY ERASE 16.85 ms WHATEVER ITS GRAIN, a word of the tail costing 20 core cycles against the window's 5 with SCKMOD moving the tail's price and not the window's, both methods writing the tail, a tail cell given a second pass reading neither value where a window cell keeps the second across a system reset, a page of the tail read exact at 144 MHz outside the rating the driver keeps, and the split's code read off FLASH_OBR with FLACAP answering 288 over a window of 256 |
| [rtc.md](rtc.md) | The real-time clock and the backup domain: not a calendar but a NUMBER - a 32-bit counter behind a 20-bit prescaler with a second, an alarm and an overflow - kept in a domain that survives a system reset and, on a battery, the loss of VDD; three gates before a register answers (two bus clocks and PWR's DBP), the write WINDOW and the read SYNCHRONIZATION the chapter demands of every access, a prescaler reload and an alarm that cannot be read back at all, the counter's two halves read against a carry the chapter is silent about, the alarm's SECOND PATH through EXTI line 17 - the one that survives a low-power mode - RTCSEL one-way with a domain reset the only way back, and an HSE division that is 512 or 128 BY LOT NUMBER on the CH32V20x_D6 - stated as a pair and measured - and one number on the other two classes, 512 on the CH32V203RB and 128 on every CH32V303; then the backup registers, ten on the CH32V20x_D6 and forty-two on the CH32V20x_D8 and the CH32V30x_D8, the tamper input that wipes them and REMEMBERS an edge it was not watching for, and the three things the one pad PC13 can carry. Measured on the CH32V203C8T6: the low-speed crystal at 32765.93 Hz - 63 ppm slow - counted over ten of its own seconds against a crystal-rooted core; THIS DIE DIVIDING THE HSE BY 128 (RTCCLK 62500 Hz), which the manual leaves to the lot number; the read synchronization back in 14 us; the alarm arriving as the counter LEAVES the value it was armed at and reaching EXTI line 17 with the block's own interrupt enable CLEAR; the overflow flag standing before the count the bus reads has wrapped, and that count stepping BACK by up to three ticks when it is read faster than it ticks; the ten data registers, the crystal, the clock select and the counter all surviving a system reset while the two bus clocks and DBP come up shut; BDRST wiping the domain while the block's own reset line moves nothing in it; and the tamper input unreachable from the pad's own side, TPE taking the pin whole |
| [crc.md](crc.md) | The cyclic redundancy check unit: three registers, no options, and the Ethernet polynomial WIRED IN - so what the block computes is exactly one function, CRC-32/MPEG-2, whose constexpr twin lives in util/crc.hpp because more than one target carries this block and one function keeps one home. The data register IS the operation (a write feeds a word, a read returns the running result, the computation stalling the bus instead of raising a flag), a WORD is the only grain so a byte stream is the caller's to pack and a length that is not whole words is a compile error, the reset bit is the only control, and the eight-bit scratch register is the one piece of state the reset spares. No interrupt, no DMA row, no per-part fact: the same block on both series. Measured on the CH32V203C8T6: eight lengths and a packed byte stream equal to util/crc.hpp's twin to the bit; the reset landing before the next instruction can look, with a word stored behind it TAKEN and not swallowed - where the same block on an STM32F4 needed cycles for both; the scratch taking a byte store, surviving RST and every fed word, and cleared by nothing short of a system reset, this block having NO line in RCC_AHBRSTR; a thousand words costing 12578 core cycles at 96 MHz against 81636 for the twin; and a block behind a shut clock gate answering a read with the last word the BUS carried instead of the register asked for |
| [rng.md](rng.md) | The random number generator of the CH32V303RC and VC, the only parts of the stratum that carry one: an analog seed behind an LFSR, three registers and one vector, two monitors whose errors are not the same kind - a clock error stops the words and leaves the last one good, a seed error spoils the word and wants 29.2.2's sequence - the two latched flags cleared by a zero because a one sets neither, no reset line, and FIPS PUB 140-2's first-word discard and continuous comparison inside `read()`. Measured on the CH32V303VCT6: SYSCLK RUNS THE BLOCK and not the PLL48CLK the chapter names - the word time the same number of core cycles at 144, 96 and 48 MHz, unmoved by USBPRE, and the words coming on the bare HSI - so no clock is refused; the first word after an enable ZERO every time and a stale word standing across a disable, so a restart discards two; DRDY rising over an unchanged word; and THE WORDS NOT TO BE TAKEN AS RANDOM BITS - 900 to 3350 distinct values in 4096 at every rate and pace, a chi-square near 49000 on 255 degrees of freedom over their bytes, the poker and runs tests failing, neither monitor firing - which the driver states and does not condition, and which the suite asserts so that a die whose words pass turns it red |
| [sleep.md](sleep.md) | PWR and the two sleep sites: three low-power modes behind one pair of bits - SLEEPDEEP, which is the CORE's, and PDDS, which is this block's, written and read as ONE PAIR - with a Stop that is ONE MODE WITH TWO PRICES (the regulator's low-power setting and the RAM's low-voltage one, whose interlock the chapter states and the driver enforces) and a Standby that is OFF THE LADDER because every way out of it is a reset; the supply monitor whose eight thresholds are worth TWO DIFFERENT TABLES of millivolts by a bit of the DIE and not of the part number, the wake-up pad PA0 with the one flag it shares with the RTC alarm, what a Standby keeps of the RAM (the class's two readings of one bit, and a number that is the class's largest array and not every part's), the regulator trims that live in the extended register, and the debug module's three low-power bits READ AND NEVER WRITTEN because a store to that CSR resets the part. Then the mapping onto [the power model](../design/power.md) - light to Sleep, standby and deep to a Stop on each regulator, the clock tree put back after one and the tick paused across it by the idle path - and THE COUNT OF ACTIVE BUS MASTERS, this family's answer to a sleep that starves every master but the core: a DMA channel and an attached USB controller each hold one, the kernel's idle path sleeps only at zero and a site refuses to arm above it, so "a program with USB never idles" becomes a mechanism. Plus the timed site, the RTC's alarm as the wake and its counter as the witness of the frozen span. Measured: an idle() that RETURNS AT ONCE over a working master in some 28 core cycles, a 65535-item block running on across eight of them and the CDC console enumerating and carrying bytes with the ordinary kernel loop where the bare WFI beside it in one image still loses every packet; a 200 ms Stop woken by the alarm with SIXTY MICROSECONDS of wake (the block raises its event one count of the ruler late, measured awake and subtracted), 30 us of RTC resynchronization owed at every wake, the tick advancing by none of the sleep and the tree back at the PLL rate two milliseconds later; THIRTY MICROSECONDS MORE at the wake for the low-power regulator, over sixteen Stops of each; a 500 ms deadline met at 502 ms of WALL and of kernel tick through the timed site under a real kernel; the vote round, the standing lock, the deadline guard and the refusal that is the silicon's over a running channel; a pad's line ending a Stop from outside; THREE STANDBYS ALL ENDING IN A RESET - the alarm's line as an interrupt, the same as an event alone, and a PAD'S line as an event, which 2.3.4 does not list as an exit at all - with SBF at every boot, WUF at the alarm's two alone, PORRSTF and never LPWRRSTF, and the .noinit token surviving; and THE INDEPENDENT WATCHDOG NOT COUNTING THROUGH A STOP, its reset arriving 215 ms after a two-second sleep it never interrupted |
| [usb.md](usb.md) | The USB device controller: ST's F1/F0 device peripheral under WCH's names, realizing util/usb's UsbController at the packet - eight endpoint registers whose status and toggle fields are WRITTEN BY XOR while the two completion flags in the same word clear on a zero, a packet memory of 512 bytes seen through a 32-bit window (one halfword every four bytes of address space) whose TOP 128 THE CAN'S FILTER TABLE TAKES, so the budget is a template parameter and a CDC port spends 328 of the 384 that are left; COUNTn_RX both the size the program writes and the count the hardware writes back over it - which is why arming a reception rewrites it, why the bus reset guards against one already standing, and why AN OVERFLOW IS A REPAIR and not only a count, the lost packet leaving the size field ZERO and no completion behind it; the host's suspend answered with FSUSP, because a module left running never hears the wake-up; the pads that are still port A's GPIO pads with the pull-up in another chapter's register; one of the two vectors used and the other left to the double buffer this driver does not offer - and TWO COMPILE-TIME REFUSALS, the 48 MHz the tree's USBPRE must make and the measured floor of the bus under it. Measured: an enumeration to CONFIGURED in 362 to 628 ms with 328 of 384 bytes spent, ten frames in 10 ms and no overflow; a thousand idle() calls costing 0 to 1 ms over the bus-master count an attachment holds; a reconnect answered with three bus resets and the SAME address; 2.87 million bytes echoed byte-exact in ten seconds, 767 KB/s out of the bulk pair and 461 KB/s into it with no ring overrun; the line coding read back as 19200 7E2 with DTR and RTS raised; a host's suspend and its resume both counted, where a module left running reported the suspend alone; and, staged with a bare wfi under a full-speed pour, 616 packet-memory overflows in 50 ms that carried nothing where the awake window carried 22 kB - the endpoint repaired and taking 95 kB in the 200 ms after |
| [vendor/README.md](vendor/README.md) | The documents of record with their revisions - the reference manual that covers four families and the CLASS RULE that divides them, both datasheets' resource and pin tables, the QingKe V4 manual for the V4B and the V4F, the probe's, and the two EVT packages as the vendor's only voice on quirks; the no-errata statement; the candidate revisions waiting to be promoted, and WHAT THE REFERENCE MANUAL'S V2.5 CHANGES, chapter by chapter, beside what the silicon said where it was measured |

## Toolchain

WCH's own `riscv32-wch-elf` gcc 15.2.0 at `/sw/wch-riscv`, the same
compiler the CH32V00x target uses and for the same reason: it is the
only one that emits WCH's proprietary `xw` compressed extension, which
this core carries too (misa reads `0x40901105` - I, M, A, C, user mode
and one non-standard extension). What is NOT shared with that family is
the ABI: **ilp32** here against its ilp32e, because this core has the
full thirty-two registers. The project's `-march` is `rv32imac_xw`,
which their gcc resolves to the `rv32imac_zaamo_zalrsc_xw/ilp32`
multilib. The ISA and the ABI are two columns of each part's row in the
part table, and the CH32V303's are its V4F's own: `rv32imafc_xw` with
**ilp32f** - float arguments and results in the single-precision
registers - which the same gcc resolves to the
`rv32imafc_zaamo_zalrsc_xw/ilp32f` multilib (misa reads `0x40901125`
there, F beside the rest). One project and one crt serve both: the crt
switches the unit on exactly when the compiler says the image was
built with F (`__riscv_flen`), never by a part name.

## Board and build

A **WeAct Studio CH32V203C8T6 core board** - a blue board in the
black-pill shape, not WCH's own EVT: an 8 MHz crystal and a 32.768 kHz
one, a blue LED on **PB2** driven active high, a KEY button on PA0 (the
vendor's page; the pad carries no external resistor, so a press is what
would prove it), and a USB-C connector wired to the chip's own USBD
pads.

And **WCH's CH32V303 evaluation board**
([../boards/ch32v303-evt.md](../boards/ch32v303-evt.md)) for the
CH32V303VCT6: an 8 MHz crystal on the LQFP100's own oscillator pins -
PD0 and PD1 being pads of their own on that package - and a 32.768 kHz
one on PC14/PC15, two LEDs and a KEY on a row of pins that a jumper
takes to a pad - PB2 and PA0, as on the WeAct board, though these LEDs
hang from 3.3 V and light with the pad LOW - and two USB connectors on
PA11/PA12, which on this part belong to the host/device controller: it
has no device one.

```bash
(cd ch32vx03 && cmake --build --preset ch32v203c8-release --target console)
(cd ch32vx03 && cmake --build --preset ch32v203c8-release --target console-upload)
(cd ch32vx03 && cmake --build --preset ch32v303vc-release --target console)
brio flash <board> console  # the same thing through the bench's one command
```

The part number selects the part definition `device.hpp` asks for, the
linker script, the board type, the ISA and the ABI, from one table
([cmake/ch32vx03-parts.cmake](../../ch32vx03/cmake/ch32vx03-parts.cmake)):
the thirteen parts - the CH32V203's nine and the CH32V303's four - are
named there whether or not a board exists for them, because the table
is a statement about the family.

The four **32 KB parts** split a suite too big for them: a suite that
declares its groups of letters (`// build: groups = abc,de`) builds
there as one image per group - `<app>-1`, `<app>-2`, ... - each
registering its own letters and carrying only their verdict prose,
while every other part builds the same source as one image with every
letter (design/overview.md, "A suite's image fits the family's
smallest chip").

## The probe and the upload

A WCH-LinkE (firmware 2.16) over the **two-wire** debug port this
family has - PA13 = SWDIO, PA14 = SWCLK - not the CH32V00x's single
wire; on the CH32V303 board, a WCH-Link of the CH549 kind at firmware
2.12 on the same two pads, whose serial bridge forwards in blocks and
can fall behind after a burst sent into a closed port
([../probes/wch-link.md](../probes/wch-link.md)). WCH's OpenOCD fork at
`/sw/wch-openocd` is the only OpenOCD that speaks the probes' SDI
transport.

**`reset run` does not start the program.** In that fork the verb
leaves the hart sitting at the reset vector with every peripheral at
its reset value - measured here: the program counter still zero and the
clock tree untouched four hundred milliseconds later - so an image
flashed that way is in the chip and NOT running, and the board looks
dead. The upload target and `brio flash` both use `reset halt` followed
by `resume`.

## Serial console

The probe's own serial pins go to **PA9 (USART1_TX)** and **PA10
(USART1_RX)**, the instance's default mapping, so one cable carries the
debug port and the console. 115200 8N1:

```bash
brio console <board>
```

A program that enables a line and binds no handler for it looks DEAD:
the crt's weak default handler is a spin loop, with no fault and no
message. What names the line in a minute is the probe -
`openocd ... -c halt -c "reg pc" -c "reg mcause"` shows the program
counter in `default_handler` and the interrupt number in mcause.

## What the silicon taught the stratum

- **`ebreak` with no debugger goes to the BREAKPOINT vector, not the
  exception one.** The table has both - the exception entry at index 3
  and the breakpoint one at index 9 - and the silicon takes the second,
  on the V4B and the V4F alike, while mcause reports exception code 3,
  this core's number for a breakpoint: the vector index and the
  exception code are different numbers here. A program that wants a
  crash recorded binds both.
- **corecfgr (CSR 0xBC0) is zero at the reset vector.** The QingKe V4
  manual says it configures the pipeline and branch prediction, gives
  no bit table, and says the products set it in their startup file;
  WCH's own writes 0x1F, and so does this crt. What those five bits
  buy is not the same on the two cores: two cycles in 36811 of a timed
  loop on the CH32V203C8T6's V4B, and on the CH32V303VCT6's V4F the same
  loop half to eight tenths of a per cent SLOWER with them than without
  ([platform.md](platform.md)).
- **The V4F's floating-point unit is off out of reset, and an interrupt
  pays for it in software.** mstatus.FS is 00 at the reset vector and
  every FP instruction traps in that state, so the crt of an image built
  with F switches it on - and its own write to fcsr leaves the unit
  Dirty, which is what main() finds. The hardware prologue saves the
  sixteen integer caller-saved registers and nothing else (the V4
  manual's 3.4, note 3): under the ilp32f ABI the compiler saves the
  f-registers a handler clobbers in the handler's own code, and ALL
  TWENTY of them as soon as the handler calls a function it cannot see
  into. Measured on the CH32V303VCT6: a round trip of 56 cycles with no
  f-register, 67 with a float multiply-add in the body, 101 for a
  handler that calls out - and the USART transport's own handler, which
  calls its ring's functions out of line, carries the twenty saves in
  the platform suite's image for that part. A handler that must stay
  cheap there stays a leaf, or its callees inline.
- **The global mask has no shadow; a line's own disable does.** The
  manual's V2.5 revision adds a note asking for a `fence.i` after a
  mask. Measured on the CH32V303VCT6 with a line pended by hand and
  masked at every distance from the pend: an interrupt not yet taken
  when the `csrrci` on mstatus.MIE retires is never taken after it - 0
  of 5000 races, with and without a `fence.i`, with and without the
  hardware prologue - while a store into the line's own PFIC_IRER lets
  it through up to three instructions later, 750 races of 750, a
  `fence.i` after the store changing nothing. The platform's guard is
  the `csrrci` ([platform.md](platform.md)).
- **The reference manual's vector table is the union of four
  families.** Its table 9-2 names TIM8 at entries 59..62, which belong
  to the CH32V30x; the CH32V203's tail is USBFS, its wake-up, UART4 and
  the eighth DMA channel, and the CH32V203RB's is different again. The
  crt states the per-class truth
  ([startup_ch32vx03.S](../../ch32vx03/src/glue/startup_ch32vx03.S)) -
  and for the CH32V303 it is WCH's own startup file for the class in
  every number but two: that file leaves entries 58 and 84 zero, and the
  silicon raises both. EXTI line 18's software trigger pends PFIC line
  58 and line 20's pends 84, read over the debug port with the core
  halted, where line 17's pends the RTC alarm's 57 as the table says.
  The manual's V2.5 revision draws one table per class, and what it
  gets right and wrong against the silicon is in
  [vendor/README.md](vendor/README.md).
- **The clock task must park on the HSI before it touches the PLL.**
  PLLMUL, PLLSRC and PLLXTPRE are writable only while the PLL is off,
  and the PLL refuses to stop while it is the system clock - so a tree
  that arrives at `init()` with the PLL already running takes every one
  of those writes in silence and keeps the rate it had, while every
  divisor in the program is computed for the rate it asked for.
  Measured both ways: the failure staged on purpose, and the fix
  recovering 144 MHz within a millisecond of re-entry.
- **The PLL's input divider is in another block - and on the other class it
  is not even the same divider.** RCC says only which root feeds the PLL;
  whether the HSI arrives whole or halved is `EXTEN_CTR.HSIPRE`, which the
  RCC chapter never mentions and which is 0 out of reset - so a program that
  only wrote RCC registers would find every rate half of what it asked for.
  The HSE's own divider IS an RCC bit (PLLXTPRE), and what it divides BY
  follows the device class: one or two on every part up to the CH32V203C8
  and on every CH32V303, four or eight on the CH32V203RB, whose oscillator
  is 32 MHz.
- **PB1 is not the datasheet's 144 MHz.** The block diagram rates both
  peripheral buses at the core's own ceiling; WCH's own clock code sets
  PPRE1 = /2 at every rate it offers, and only the bare-HSE path leaves
  it undivided. This stratum caps PB1 at 72 MHz. Where the real ceiling
  lies between those two numbers is not known.
- **UART4's pads are the part's, not the family's.** The manual has two
  remap tables for it; the one that starts at PC10/PC11 belongs to the
  bigger classes, and the CH32V203C8 is named explicitly in the other,
  whose default pads are PB0 and PB1. The CH32V303 reads the first, and
  its UART4 carries a DMA run over PC10/PC11 ([dma.md](dma.md)).
- **Two AFIO fields the manual gives the whole family are the class's.**
  USART3's remap and TIM2's internal-trigger bit are read-only at zero
  on the CH32V203C8T6 and take a write on the CH32V303VCT6, which is
  where the driver refuses and offers them ([pin.md](pin.md)); on the
  CH32V303VCT6 TIM2's ITR1 carries TIM8's TRGO whichever value the
  field holds ([tim.md](tim.md)).
- **An EXTI line reaches a peripheral through its EVENT enable, not its
  interrupt enable.** Twenty edges on a pad started no conversion with
  the line merely sensed, twenty with EXTI_EVENR set, and none with
  EXTI_INTENR set instead - while the line's own handler ran for all
  twenty, on both parts. Neither chapter says so; the next block that takes an EXTI
  trigger will meet it - and the CH32V303's DAC takes its EXTI line 9
  trigger the same way ([dac.md](dac.md)).
- **The USB pads are still GPIO pads.** D- and D+ are PA11 and PA12 and
  the USB device controller reaches them through port A: with that
  port's clock closed - its reset state - the pull-up still works (it
  lives in EXTEN), so the host sees a device attach and starts
  enumerating, and then every packet fails. WCH's own `USB_Port_Set()`
  opens the port's clock and leaves the two pads floating inputs before
  it touches the pull-up.
- **The CH32V303 has no USB device controller**, against RM ch. 21's
  opening, which applies that controller to the whole family. Its
  clock gate, bit 23 of RCC_APB1PCENR, is not there: that register
  written with all ones reads back 0x3A7EC9FF, every gate the manual
  names for this part answering and bit 23 alone of them silent, and the
  block's registers read zero. The part's one full-speed controller is
  the host/device one of ch. 23 (its control register reads its reset
  value, 0x06), on PA11/PA12 - which the datasheet's alternate-function
  table gives the OTG_FS pair and WCH's own board labels USBFS. So the
  CH32V203's USB console is not this part's, and `has_usbd` is false in
  its part table.
- **THE USB CONTROLLER DOES NOT SURVIVE THE CORE'S SLEEP**, and the
  manual says it should. Chapter 2's table gives Sleep as "core clock
  off, no effect on other clocks" and its prose as "the core stops
  running and all peripherals are still running"; measured on the
  CH32V203C8T6, with the core in WFI or in the platform's WFE idle, the
  controller cannot reach its packet memory. An armed bulk endpoint
  receives nothing - the host's bytes are lost, `cdc_rx` stays at zero
  and the packet-memory overflow counter climbs one per attempt - and an
  enumeration never gets past its first control transfer. The same image
  with a loop that only `step()`s enumerates, configures and carries
  bytes with not one overflow. Both idle forms fail and a spinning loop
  works, so it is the core being asleep and not the idiom. A program
  that uses USB therefore does not SLEEP, and it is the platform that
  enforces it: an attached controller counts itself a bus master and
  `idle()` does not sleep while the count stands ([sleep.md](sleep.md)).
  [usb_probe](../../ch32vx03/src/apps/usb_probe.cpp) is the instrument
  that measured it and still shows both sides in one image - with the
  count in place its WFE reaches CONFIGURED with no overflow while its
  WFI never gets past the first stage and counts sixteen. WCH's own
  reference implementation fails the same way, which is what settles
  whose the finding is: the EVT's SimulateCDC example, built with the
  vendor's compiler and flags and unchanged but for one `wfi` at the end
  of its loop, never reaches CONFIGURED - SET_ADDRESS gets through, the
  descriptor reads do not, the overflow counted 18 to 26 times - and the
  vendor's own `__WFE()` fails the same, while a busy-wait of the same
  length in the same loop works; a variant that sleeps only once
  configured loses every one of the 4096 bytes the host writes; and the
  loss happens with the core woken at least once per USB frame (the
  frame interrupt at 1 kHz and a 10 kHz timer armed), so it is neither
  the depth nor the length of the sleep. Measured on the host-to-device
  direction; the absence of an erratum is, once again, evidence of
  nothing. The debug module's keep-HCLK-in-Sleep bit could not be tried:
  a `csrw` to CSR 0x7C0 from the running program resets the part.
- **In Sleep, no bus master but the core gets a cycle.** A
  memory-to-memory DMA started right before a `wfi` on the CH32V203C8T6
  moves nine to twelve bytes - the pipeline between the enable and the
  sleep - and nothing more until the core wakes, while a timer on the
  peripheral bus and the core's own counter count the whole sleep; woken
  once a millisecond it moves eleven bytes a wake. The clocks run; the
  bus matrix serves the core alone - and on the CH32V303VCT6 the same,
  on both of its controllers: a memory-to-memory block moves 48010 words
  in 2 ms with the core spinning and 37 across 1.9 ms of the platform's
  idle() with the bus-master count set aside, six of them before the
  core slept, and DMA2's first channel 23 bytes across one idle() and
  120 across ten. The CH32V203's USB device controller reaching into its
  packet memory is such an access, which is why a transfer with a
  payload fails in Sleep and one without (SET_ADDRESS) passes - and why
  no software mitigation short of staying awake works, measured on the
  vendor's own example: unmasking the overflow interrupt wakes the core
  but the packet is gone and the host does not retry a failed bulk
  burst; staying awake for ten or a hundred milliseconds after the
  overflow recovers nothing of the burst in flight; holding the endpoint
  at NAK across the sleep still overflows; re-validating the endpoint
  finds it valid. What does work is not sleeping and dividing HCLK
  instead: the device enumerates and carries data with HCLK at 96, 48
  and 24 MHz and the USB clock at 48, and fails at 12 MHz with the
  sleeping case's own signature. So on this family a program that moves
  data through the bus - USB, a DMA-fed transport - does not sleep; it
  slows down, to no less than 24 MHz of HCLK. And that is a MECHANISM
  and not a rule the programmer keeps: [sleep.md](sleep.md).
- **A DMA channel serves every request wired to it.** A channel's rows
  are an OR of the peripherals' requests, switched by each peripheral's
  DMA bit: a UART4 receiver left enabled made a timer's transfer on
  DMA2's channel 3 move all sixteen items at once. Stopping the channel
  withdraws nothing; turning the peripheral off does ([dma.md](dma.md)).
- **A conversion takes 12.5 cycles beyond its sampling time, not the
  manual's 11** - both datasheets, and the CH32V303VCT6 at all four long
  sampling codes ([adc.md](adc.md)).
- **On the CH32V203C8 the sample-and-hold tracks the selected channel
  between conversions; on the CH32V303VCT6 it does not.** A 40 kOhm
  pull-up converted right after a grounded channel in one scanned
  sequence reads 1060 counts at 1.5 sampling cycles and 4095 at 239.5;
  parked on that same channel it reads full scale at every sampling
  time. The sampling time is what a SEQUENCE pays for its multiplexer
  moving, while the CH32V303VCT6 reads 1053 at 1.5 cycles and the same
  after 200 us of park.
- **A break input held at its level raises its flag again as soon as it
  is cleared**, which the chapter does not say: on the CH32V303VCT6's
  TIM8 a break vector that returns while BKIN still stands is taken
  again at once, and nothing else runs until the pad falls. A handler
  that may see a standing break masks its own interrupt
  ([tim.md](tim.md)).
- **The RNG's words are not random bits, and SYSCLK is its clock.** On
  the CH32V303VCT6 four thousand words hold 900 to 3350 distinct values
  at every rate and pace tried, a chi-square over their bytes lands near
  49000 on 255 degrees of freedom and FIPS PUB 140-2's poker and runs
  tests fail - while neither of the block's two monitors fires; and the
  block runs on SYSCLK, not on the PLL48CLK chapter 29 names, its word
  time the same number of core cycles at 144, 96 and 48 MHz and unmoved
  by the USB prescaler. The driver hands the words out unconditioned
  and says so ([rng.md](rng.md)).
- **The CH32V303VCT6 of the bench behaves as a lot the manual's lot
  notes restrict.** Five registers the CH32V30x_D8's notes give only to
  lots whose penultimate sixth digit is not zero are absent on it -
  TIMx_AUX's dual-edge capture, ADCx_AUX's short sampling times,
  ADC_DUTY_SEL, EXTEN_CTR2's high-speed bits and ADC2's DMA request -
  and the one restriction the notes give the other lots is present: DMA1
  wraps a span crossing a 64 KB boundary inside the page it started in.
  A program cannot read its lot - the notes key on digits of a lot
  number no register states - so each feature is a verb that ASKS THE
  DIE, writing its bit and reading it back before anything else
  (TIMx_AUX kept no bit on TIM1 and TIM2 alike, and the dual-edge verb
  answered false), and the restriction is kept on every lot
  ([dma.md](dma.md), [adc.md](adc.md), [tim.md](tim.md),
  [opa.md](opa.md)).
- **An absent register can be a MIRROR.** On that die EXTEN_CTR2's
  address reads and writes EXTEN_CTR: a high-speed bit set there lands
  in EXTEN_CTR's USB bits and a whole word rewrites the regulator trims.
  Reading a bit back cannot tell that from a register that is there;
  reading the WORD can, and the amplifier driver does it before any
  write ([opa.md](opa.md)).

## Not covered yet

Driver gaps, each with its reason:

- **CAN**: the one chapter still open, waiting for the pass across every
  platform that brings a second transceiver to the desk; it arrives
  with its own document and its own suite.
- **TKEY, WCH's capacitive touch sensing (ch. 13), DECLINED**: the block
  is an ADC mode - the converter's charge transfer against a pad's
  capacitance - and what it produces is a key press, which is an
  application and not a driver. Its two bits are named in the register
  map and nothing writes them.
- **SDIO, the CH32V303RC's and VC's card interface (ch. 28)**: the
  evaluation board has no card socket, so a driver would have nothing to
  answer to; the block's base, its clock gate and its vector are in the
  register map and nothing drives them.
- **The FSMC, the CH32V303VC's static memory controller (ch. 26)**: a
  memory on the bus is what the chapter wants and the board carries
  none; the base and the gate are in the map, and its windows are read
  only by the DMA chapter, as addresses that raise no transfer error
  ([dma.md](dma.md)).
- **The CH32V305 and the CH32V307**, the CH32V30x_D8C class the same
  datasheet covers: another device class - a second PLL and a third,
  the high-speed USB, the Ethernet, a second CAN - that no board here
  carries, so the part table does not name them.

Implemented but not bench-verified:

- **The eleven parts other than the CH32V203C8 and the CH32V303VC.**
  Every one has its table, its linker script and its preset, and the
  whole stratum compiles for all thirteen both ways the hardware
  prologue can be built (`brio check ch32vx03`); the CH32V203C6 preset
  links every image and is the 32 KB / 10 KB tier's guard, and the
  CH32V203RB's console links with its seventy-word vector table. What
  would measure them is a board.
- **The KEY buttons on PA0**: no program has pressed either. The WeAct
  board's is the vendor's claim, and there the pad is USART2's CTS and
  carries the wake edge of [sleep.md](sleep.md)'s suite when a second
  board drives it; the evaluation board's ties its pin to ground
  through 10 kOhm when pressed and the board gives it no pull-up (its
  schematic).
  What would read either is a press, which [pin.md](pin.md)'s suite
  keeps a letter for, by name.
- **The lot-keyed features on a die of the other lots.** TIMx_AUX's
  dual-edge capture, ADCx_AUX's short sampling times, ADC_DUTY_SEL,
  EXTEN_CTR2's high-speed bits and ADC2's DMA request are verbs that ask
  the die, and the CH32V303VCT6 answered no to each; what would measure
  them is a CH32V303 of a lot whose penultimate sixth digit is not zero.
