# brio

A small modern-C++ framework for bare-metal microcontrollers, built
around a cooperative **active-object kernel**: event queues, flat state
machines, time events, one loop, one stack, no heap, no virtual
functions, nothing resolved at run time that the compiler could resolve
first. Written in C++23 (gnu++23), header-only, in one flat namespace
`brio` ("con brio" - the musical marking for liveliness).

The kernel knows nothing about the silicon it runs on. It runs on an
8-bit AVR, on ARM Cortex-M cores (M0+, M4 and M33), on RISC-V cores
(WCH's QingKe and Raspberry Pi's Hazard3) and on the host - the target
table below names every family and the chip it is measured on. Each
family after the first compiled the kernel and its services unchanged,
which was the design's promise.

## What an application looks like

```cpp
using P = brio::AvrPlatform;             // the one place the target is named

struct Toggle {};                        // events are small plain structs
struct SetPeriod { uint16_t ticks; };

struct Blinker : brio::Fsm<Blinker, Toggle, SetPeriod> {
    static inline brio::EventQueue<Event, 4, P> queue;                 // its own queue
    static inline brio::TimeEvent<P, Blinker, Toggle> heartbeat{Toggle{}};

    static void init() { Led::output(); start(&running); }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { heartbeat.arm_every(brio::ticks_from_ms<P>(500));
                              return handled(); },
            [](Toggle)      { Led::toggle(); return handled(); },
            [](SetPeriod p) { heartbeat.arm_every(p.ticks); return handled(); },
            [](auto)        { return unhandled(); });
    }
};

int main() {
    /* clock, timebase, sei() - target glue */
    brio::Tenuto<P, Blinker, Supervisor>::run();   // priority = pack order
}
```

Somewhere else, `brio::post<Blinker>(SetPeriod{...})` changes the
cadence - from another active object, from a time event, or from an
ISR: same call, always safe, never blocking.

## The ideas

- **Active objects, run-to-completion.** An AO owns a queue and reacts
  to one event at a time; it never blocks, waits or polls. The kernel
  serves the highest-priority non-empty queue, one event per turn, and
  sleeps when nothing is pending. The only concurrency is ISR vs main
  loop, and an ISR may do exactly one kernel thing: post an event.
- **Everything is a type, resolved at compile time.** AOs, drivers and
  the kernel are monostate classes selected by type; priorities,
  subscriptions and reply channels are template parameters. No tables
  walked at run time; RAM is exactly what is declared.
- **Contracts are concepts.** What the kernel needs from an AO
  (`ActiveObject`), from the machine (`Platform`), what a text sink
  must offer (`ByteSink`) - stated as C++20 concepts, checked where a
  type is used, with errors that name the requirement.
- **Events are values, per-AO variants.** Each AO declares its own
  `std::variant` of small trivially-copyable structs; events are copied
  into queues (a few bytes, one brief critical section). Plain shared
  structs are the lingua franca between publisher and subscriber; no
  global signal enum. Handlers dispatch with `match(e, lambdas...)`.
- **Timers post events; failures leave a breadcrumb.** A time event
  posts to its owner AO in main context (never runs user code in an
  ISR); a full queue is a sizing mistake counted, never blocking;
  `panic()` writes a reset-surviving record before any LED blinks.

The full rationale, decision by decision, is in
[docs/design/](docs/design/) - start with
[overview.md](docs/design/overview.md) and
[kernel.md](docs/design/kernel.md).

## Layering and portability

![brio strata](docs/design/architecture.svg)

<sub>[open the diagram full size](https://raw.githubusercontent.com/uliano/brio/main/docs/design/architecture.svg) (zoomable in the browser)</sub>

The include prefix makes a file's portability readable at a glance,
and every stratum under `brio/` is a row here:

| Stratum | Contains | Depends on |
|---------|----------|------------|
| `kernel/` | queues, scheduler, FSM, delivery, time events, panic - pure logic | nothing of brio |
| `util/` | services on top of the kernel: `SerialPort`, `BusMaster` (SPI/I2C arbiter), `print`, `Ring`, line parsers | `kernel/` |
| `gfx/` | drawing: the three kinds of surface, the primitives over a write-only base, fonts and opaque text, the pen | nothing of brio - not even the kernel |
| `avrdx/` | everything that knows `avr/io.h`: clock, pins, UART, SPI, TWI, ticker, `AvrPlatform` | `kernel/`, `util/` |
| `cortexm/` | what ARM designed into every Cortex-M and the families below share: NVIC + PRIMASK guard, the SysTick ticker, the microsecond busy-wait on SysTick's counter | `util/` (and the including family's device header) |
| `pl011/`, `pl022/`, `dw_apb_i2c/` | IP strata: a peripheral DESIGN written once and knowing no chip - ARM's PrimeCell UART and SSP, Synopsys's DesignWare I2C - each stating a concept the family's own traits type satisfies | `kernel/`, `util/` |
| `samc21/` | everything that knows `sam.h` (Cortex-M0+): clock tree, pins, SERCOM UART, `SamPlatform`; its NVIC and ticker are `cortexm/`'s | `kernel/`, `util/`, `cortexm/` |
| `stm32g0/` | everything that knows `stm32g0xx.h` (Cortex-M0+): RCC/PLL, GPIO, USART, `Stm32g0Platform`; its NVIC and ticker are `cortexm/`'s | `kernel/`, `util/`, `cortexm/` |
| `stm32f4/` | everything that knows `stm32f4xx.h` (Cortex-M4F, brio's first ARMv7-M): RCC/PLL with the regulator scale and over-drive and the dynamic clock, GPIO, USART, EXTI, the RTC, the DMA, the timers, ADC/DAC, SPI/I2S, I2C and FMPI2C, the USB OTG core, PWR with the sleep sites, the flash engine, CRC/RNG/bxCAN, the FMC with its SDRAM, the LTDC and the DMA2D, `Stm32f4Platform`; its NVIC and ticker are `cortexm/`'s | `kernel/`, `util/`, `cortexm/` |
| `ch32v00x/` | everything that knows the CH32V00x (QingKe V2C, RV32EC - the smallest core brio runs on): its own register map (no vendor header), the clock, the pads and their remaps, USART, SPI, I2C, DMA, the timers, the ADC and the OPA, flash and the two watchdogs, sleep, the PFIC guard, the STK ticker, `Ch32v00xPlatform` | `kernel/`, `util/` |
| `ch32vx03/` | everything that knows the CH32V203 (QingKe V4B, RV32IMAC): its own register map again, over the STM32F1's peripheral generation under WCH's names - the clock tree, the pads and AFIO's remap columns, EXTI, the USARTs, SPI, I2C, DMA, the timers, the two converters and the two amplifiers, the USB device controller, flash, CRC, the RTC and the backup domain, PWR with its sleep sites, `Ch32vx03Platform` | `kernel/`, `util/` |
| `rp2040/` | everything that knows the RP2040 (a Cortex-M0+ pair): the clock generators, the pads, the timer, the watchdog and the resets, the DMA, PWM, the ADC, the RTC, the PIO, the QSPI flash, the USB device controller, the two cores and a kernel on each, `Rp2040Platform` | `kernel/`, `util/`, `cortexm/`, `pl011/`, `pl022/`, `dw_apb_i2c/` |
| `rp2350/` | everything that knows the RP2350 (a Cortex-M33 pair AND a Hazard3 RISC-V pair over one set of peripherals, one of the two running): the clock tree, the pads, the two system timers, the watchdog and the resets, the DMA, PWM, the ADC, the PIO, the SHA-256 and TRNG blocks, the OTP read side, the bootrom's function table, the quad-SPI flash, the USB device controller, the second core, `Rp2350Platform` - with `core.hpp` the one file that asks which processor is in the socket | `kernel/`, `util/`, `cortexm/` (its Arm half), `pl011/`, `pl022/`, `dw_apb_i2c/` |
| `host/` | `HostPlatform`: the native test "target" (virtual clock, recording idle/break), and the simulated devices the host suites and programs run against | `kernel/` |

Targets are siblings, never meet in one binary, and are the only place
where hardware headers, ISR vector names and tick rates live. Nothing
above them uses `#ifdef` to tell targets apart: where behaviour must
differ, the target states a fact (`ticks_per_second`, `atomic_width`)
and generic code chooses with `if constexpr` or a concept.

Two kinds of stratum sit between `util/` and the targets, and both are
born under the same rule - **at the SECOND family that carries the
thing, with both copies in hand, and gated by the images**: every
release image of the family that had it first must be byte-identical
before and after the extraction. A CORE stratum (`cortexm/`) holds what
a processor family's designer specified and every vendor ships
unchanged; an IP stratum (`pl011/`, `pl022/`, `dw_apb_i2c/`) holds a
peripheral block licensed by more than one vendor and dropped into
their chips register for register. Neither knows a chip: what a family
owes an IP stratum is a traits type satisfying the concept that stratum
states - where the registers are, its reset, its interrupt line, its
clock, which pads are legal - while the family's own header keeps the
public names an application writes.

A target is either **supported** - its peripheral chapters are
implemented and bench-verified, and everything `kernel/` and `util/`
claim holds there - or **in bring-up**, where the stratum exists, part
of it is proven on silicon, and the rest is still to be validated. This
table is the one place that question is answered: no source file carries
its own list of the targets it was tried on. Where a contract's
realizations differ - and where they are one interface - is the
REALIZATIONS TABLES, one per design page, indexed in
[docs/design/overview.md](docs/design/overview.md) ("One interface
where it can, its exceptions where a reader looks").

| Target | State | Bench silicon | Notes |
|--------|-------|---------------|-------|
| AVR DA/DB (`avrdx/`) | supported | AVR128DB48 | avr-gcc 16.2, see [docs/avrdx/README.md](docs/avrdx/README.md) |
| SAM C21 (`samc21/`) | supported | ATSAMC21J18A | arm-none-eabi-gcc 16.2, SysTick tick at 1000 Hz against the AVR's 1024 - the kernel tick's opacity, exercised for real; see [docs/samc21/README.md](docs/samc21/README.md) |
| STM32G0 (`stm32g0/`) | supported | STM32G0B1RE, STM32G071RB, STM32G031K8 | arm-none-eabi-gcc 16.2, HSI16 x PLL at 64 MHz, the third clock model (shared bus prescalers + per-peripheral enables) and a tickless timebase option; see [docs/stm32g0/README.md](docs/stm32g0/README.md) |
| CH32V00x (`ch32v00x/`) | supported | CH32V006K8U6 | WCH's riscv32 gcc 15.2 with its `xw` extension, RV32EC (sixteen registers, 8 KB of RAM - the smallest core brio runs on), HSI x2 at 48 MHz, its own register map with no vendor header, the console on the WCH-Link's own serial; the CH32V003F4P6 (16 KB, 2 KB, no multiplier) is the family's second and last part, supported on the same stratum with its own part table, presets and ISA - every chapter tiered for it, its suites green on the board as group images, the buses on the wire against a peer, the smallest silicon brio runs on; see [docs/ch32v00x/README.md](docs/ch32v00x/README.md) |
| CH32V203 / CH32V303 (`ch32vx03/`) | supported; the CH32V303 in bring-up | CH32V203C8T6 (a WeAct core board), CH32V303VCT6 (WCH's evaluation board) | WCH's riscv32 gcc 15.2 with its `xw` extension, RV32IMAC on the QingKe V4B and RV32IMAFC on the V4F (the FPU, the hard-float ABI, the part table stating each part's ISA) - the second WCH family and a bigger core than the CH32V00x's, with the STM32F1's peripheral generation under WCH's names; its own register map with no vendor header, the nine parts of the V203 series and the four of the V303 in one part table under three device classes, the PLL at 144 MHz from the internal RC or from the board's crystal, the console on the WCH-Link's own serial AND on the chip's own USB (util/usb's CDC over the USBD controller, which is ST's device peripheral under WCH's names); every chapter of the reference manual's plan has its document and its suite green on the board, the two buses on the wire against a peer board, and CAN is the one chapter still open, waiting for the cross-platform CAN pass; the power model here rests on a finding of this silicon's own, that in a sleep of any depth no bus master but the core gets a cycle, so a count of active bus masters keeps the idle path awake while a DMA channel or the USB controller is working; the 32 KB tier has a preset of its own as the link guard, and a suite too big for it builds there as one image per group of letters; the CH32V303 joins as the stratum's third device class (CH32V30x_D8, its four parts in the same part table) on the QingKe V4F - RV32IMAFC with the ilp32f ABI, the crt switching the FPU on for an image built with F - its platform and clock measured on WCH's evaluation board, where the bus in Sleep stalls as on the CH32V203 and there is no USB device controller at all, the part's one full-speed controller being chapter 23's host/device block; see [docs/ch32vx03/README.md](docs/ch32vx03/README.md) |
| RP2040 (`rp2040/`) | supported | RP2040 (a Raspberry Pi Pico, a WeAct board) | arm-none-eabi-gcc 16.2, the third Cortex-M0+ family on `cortexm/` - two cores, a kernel on each with the inbox bridge between them, or one kernel on core 0; the USB device stack with a CDC console on the chip's own connector; the 12 MHz crystal through the PLL at 125 MHz, the fourth clock model (a generator per clock domain, a separate peripheral clock, no bus prescaler); the pico-sdk's device description vendored, its own crt and boot stage; see [docs/rp2040/README.md](docs/rp2040/README.md) |
| RP2350 (`rp2350/`) | in bring-up | RP2350 in the QFN-80 package (a WeAct RP2350B core board) | BOTH INSTRUCTION SETS OVER ONE STRATUM: arm-none-eabi-gcc 16.2 on the Cortex-M33 pair and a self-built riscv32-unknown-elf gcc 16.2 on the Hazard3 pair, one set of drivers, one source per suite, and two presets per build type - which pair runs is decided by the IMAGE_DEF block in the image the bootrom finds, with no button, no fuse and no second-stage bootloader; a suite is green when it is green on both halves; the pico-sdk's RP2350 device description vendored in its own include root, the two crts and the linker script the project's own, and the PL011, PL022 and DesignWare blocks reached through the IP strata this chip shares with the RP2040; every chapter of the datasheet's plan has its document and its suite green on both halves - the platform, the clock tree, the pads, the PL011, the timers, the watchdog and the resets, the DMA, the PWM, the SPI, the I2C, the PIO, the ADC, the USB device controller with a CDC console on the chip's own connector, the SHA-256 and TRNG blocks, the OTP read side, the bootrom's table, the external flash with its FlashMedia, the second core with a kernel of its own, and POWMAN with the always-on timer and the sleep sites - and what remains is in the documents' gap lists, the HSTX and the Cortex-M33's coprocessors first among them; see [docs/rp2350/README.md](docs/rp2350/README.md) |
| STM32F4 (`stm32f4/`) | supported | STM32F429ZI, STM32F446RE, STM32F411CE | arm-none-eabi-gcc 16.2 with the hard-float ABI, brio's first ARMv7-M family on the same `cortexm/` core files; the PLL at 180 MHz in over-drive on two boards and 100 MHz on the third, the APB prescalers unpinned (a rate per bus); every chapter of the three reference manuals with its document and its suite green on every board it builds for, against the devices the boards carry (a gyroscope, a touch controller, an SDRAM, a panel) and the kernel console on the black pill's own USB connector; see [docs/stm32f4/README.md](docs/stm32f4/README.md) |
| host (`host/`) | supported | - | doctest suites, `cd test && ctest --preset host`, see [docs/host/README.md](docs/host/README.md) |

## Building and testing

The framework in `brio/` is header-only, included directly. The
builds are sibling CMake projects, one per target, all peers (the repo
root is not a CMake project): `avrdx/`, `samc21/`, `stm32g0/`,
`stm32f4/`, `ch32v00x/`, `ch32vx03/`, `rp2040/`, `rp2350/` and
`host/` each auto-discover one `main()` per `src/apps/<app>.cpp` at
configure time - an app may pin build options such as its console baud
with `// build: monitor_speed = 115200` header lines - and `test/`
holds the host unit tests (a configure has exactly one compiler).

```bash
(cd test    && ctest --preset host)                                       # host tests: kernel, queues, FSM, time events, buses, ring...
(cd avrdx   && cmake --build --preset avr128db48-release --target <app>)  # build one AVR app (release, -Os)
(cd avrdx   && cmake --build --preset avr128db48-release --target <app>-upload)   # flash it over UPDI
(cd samc21  && cmake --build --preset samc21j-release --target <app>)     # build one SAM app
(cd samc21  && cmake --build --preset samc21j-release --target <app>-upload)      # flash it over SWD
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>)  # build one STM32G0 app
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>-upload)   # flash it over the ST-LINK
(cd ch32v00x && cmake --build --preset ch32v006k8-release --target <app>-upload)    # flash it through the WCH-Link
(cd rp2350  && cmake --build --preset rp2350-arm-release --target <app>)    # build one RP2350 app for the Cortex-M33
(cd rp2350  && cmake --build --preset rp2350-riscv-release --target <app>)  # ... and the same source for the Hazard3
```

With more than one board on the desk the bench has one command,
`bin/brio` (put `bin/` on the PATH): `brio list` shows the boards the
manifest knows and which are plugged in, `brio flash <board> <app>`
builds for that board's type and flashes it whatever its probe,
`brio run <board> <letter>` drives a test suite's console and judges
its summary, `brio check <stratum>` runs the family compile fixtures
and `brio prose` the prose net. `brio --help` lists the rest.

Everything target-specific - toolchain, board, probe, debugger, its
quirks - is documented per target in each target's folder under [docs/](docs/).
The apps are the framework's test bench: disposable by design, they
document themselves in their own header comment (`brio apps` lists
them), and what a suite needs wired is in that comment. The boards and
the probes brio is tested with are [docs/boards/](docs/boards/) and
[docs/probes/](docs/probes/).

## Status

The kernel and the services above it compile unchanged on every target
the table above names, and are bench-tested on the families marked
supported: the kernel
loop and time events, a serial console over `SerialPort`, an arbitrated
SPI bus (a display and a touch controller sharing one) and an
arbitrated I2C bus (a DAC written and read back, an ADC measured
through SPI), the power model with its sleep sites, the flash storage
classes, and the analog and metering services. Each target's own
chapters are covered by the reference suites its documentation names.

brio is still revised freely: nothing below the kernel contract is
promised stable, and where a limitation can be removed by rewriting
what sits below, the rewrite wins (the governing rule in
[overview.md](docs/design/overview.md)). What has settled is measured
rather than declared: the kernel and the services above it reached each
new family unchanged, and the contracts a program can lean on are the
ones a realizations table shows realized on family after family - the
AO contract, the bus vocabularies, the power model,
the storage classes, the analog and metering services, the clock model.
Clean-room with respect to QP: the
concepts come from Samek's book, never the QP source.

## License

brio is released under the MIT license ([LICENSE](LICENSE)): use it,
change it, ship it, keep the notice with it. The vendored components
under [third_party/](third_party/) keep their own licenses - doctest
(MIT), CMSIS-Core, cmsis-device-g0, cmsis-device-f4 and the SAM C21
DFP (Apache-2.0), and the pico-sdk subset (BSD-3-Clause).
