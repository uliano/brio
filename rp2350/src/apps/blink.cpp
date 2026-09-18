// blink - first light on an RP2350, and THE SAME SOURCE ON BOTH
// ARCHITECTURES: the kernel over one active object whose periodic time
// event toggles the board's LED, built once for the Cortex-M33 pair and
// once for the Hazard3 pair (the rp2350-arm-* and rp2350-riscv-* presets
// of this project). If it blinks under both, the crt, the IMAGE_DEF, the
// clock tree, the timebase, the interrupt path and the kernel are alive
// on both halves of the chip.
//
// THERE IS NO CONSOLE YET on this target, so the app reports through its
// own globals instead: a debugger reads them by name out of the ELF
// while the program runs (the fork reads memory under either
// architecture without halting the core). What each one says is in its
// comment below - the tick count, the three clock rates the chip's own
// frequency counter measured, the chip's identity, and the fraction of
// the time the loop is awake with nothing to do.
//
// WHAT IT PROVES, and what a reader should look for:
//  - brio_ticks advancing by 1000 a second is the kernel timebase,
//    which is SysTick on the Arm build and the RISC-V platform timer on
//    the other, under one name (brio/rp2350/ticker.hpp);
//  - brio_clk_sys_hz reading 150 MHz and brio_clk_ref_hz 12 MHz is the
//    crystal and the system PLL, measured by the chip against its own
//    crystal and not by arithmetic;
//  - brio_toggles advancing twice a second is the kernel's time event,
//    and GP25 following it is the pad, the pad's ISOLATION LATCH
//    dropped, and SIO;
//  - brio_awake_permille in the single digits is idle() really sleeping.
//
// The LED is on GP25 and the KEY on GP23 on the WeAct RP2350B board; no
// wire of the bench is touched.
//
// build: boards = weact2350b,weact2350b-rv

#include <stdint.h>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/sysinfo.hpp"
#include "rp2350/ticker.hpp"

#include "kernel/event_queue.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"

using namespace brio;

// ---- the board ------------------------------------------------------------

using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;

using P = Rp2350Platform<>;
using Led = Pin<25>;

// ---- what a debugger reads -------------------------------------------------
//
// Plain globals, volatile so every store lands in memory where a probe
// can see it, and `used` so nothing here is dropped for having no reader
// inside the program.

extern "C" {
/// The kernel's tick count, mirrored by the tick handler: 1000 a second.
[[gnu::used]] volatile uint32_t brio_ticks = 0;
/// LED toggles: two a second.
[[gnu::used]] volatile uint32_t brio_toggles = 0;
/// Whether the clock task reached the rate it claims.
[[gnu::used]] volatile uint32_t brio_clock_ok = 0;
/// The three rates, as the chip's own frequency counter measured them
/// against the crystal (0 = the counter said the source was dead).
[[gnu::used]] volatile uint32_t brio_clk_sys_hz = 0;
[[gnu::used]] volatile uint32_t brio_clk_ref_hz = 0;
[[gnu::used]] volatile uint32_t brio_clk_peri_hz = 0;
/// CHIP_ID whole, and the package the chip reports (60 or 80 pins).
[[gnu::used]] volatile uint32_t brio_chip_id = 0;
[[gnu::used]] volatile uint32_t brio_package_pins = 0;
/// Which core this is, and which architecture the image was built for
/// (0 = Cortex-M33, 3 = Hazard3, the values ARCHSEL_STATUS reports).
[[gnu::used]] volatile uint32_t brio_core = 0;
[[gnu::used]] volatile uint32_t brio_arch = 0;
/// How many times the spare interrupt line raised in software has been
/// served: one, and the whole interrupt path is proven - the controller,
/// the vector table (or, on the RISC-V half, the crt's dispatch through
/// Hazard3's own controller) and a handler bound BY THE SAME NAME on
/// both architectures.
[[gnu::used]] volatile uint32_t brio_spare_irq = 0;
/// The idle measurement, taken once before the kernel starts: how many
/// idle() calls covered two hundred ticks, how long that span was, how
/// much of it the loop was awake, and the same as a fraction in parts
/// per thousand.
[[gnu::used]] volatile uint32_t brio_idle_calls = 0;
[[gnu::used]] volatile uint32_t brio_idle_span_us = 0;
[[gnu::used]] volatile uint32_t brio_idle_awake_us = 0;
[[gnu::used]] volatile uint32_t brio_awake_permille = 0;
}

// ---- the active object -----------------------------------------------------

struct Toggle {};

/// The one AO: a periodic time event every 10 ms, the LED toggled every
/// twenty-fifth of them. The period is that short so that a debugger
/// reading brio_toggles sees the KERNEL running - the queue, the
/// dispatch and the time-event list - and not only the tick handler.
struct Blinker {
    using Event = Toggle;

    static inline EventQueue<Event, 2, P> queue;
    static inline TimeEvent<P, Blinker, Toggle> heartbeat{Toggle{}};

    static void init() {
        (void)Led::output(false);
        heartbeat.arm_every(ticks_from_ms<P>(10));
    }

    static void dispatch(const Event&) {
        if (++phase >= 25u) {
            phase = 0;
            Led::toggle();
            brio_toggles = brio_toggles + 1u;
        }
    }

    static inline uint8_t phase = 0;
};

using Kernel = Tenuto<P, Blinker>;

// ---- the vector binding ----------------------------------------------------
//
// ONE NAME, BOTH ARCHITECTURES: on the Arm build this is SysTick's
// vector, on the RISC-V build the machine timer trap the crt routes here
// (rp2350/src/glue/). The mirror store is what a debugger reads.

extern "C" void isr_systick() {
    Ticker::tick();
    brio_ticks = Ticker::ticks();
}

// The sixth spare line, which reaches no hardware on purpose (datasheet
// 3.2) and exists to be raised in software. The name is the same on both
// architectures - a macro for the slot isr_irq46 - and so is the line
// number, the two architectures sharing one interrupt numbering.
extern "C" void isr_spare_0() {
    brio_spare_irq = brio_spare_irq + 1u;
    Irq::clear_pending(SPARE_IRQ_0_IRQn);
}

// ---- the measurements taken once, before the kernel runs -------------------

/// Two hundred ticks of doing nothing: how many idle() calls cover them,
/// and how much of the span the core spent awake. The ruler is the
/// microsecond counter (rp2350/mtime.hpp), which runs off clk_ref and
/// through any sleep this program can enter.
static void measure_idle() {
    const uint32_t t0 = Ticker::ticks();
    const uint32_t u0 = Mtime::micros();
    uint32_t slept = 0;
    uint32_t calls = 0;
    while (Ticker::ticks() - t0 < 200u) {
        const uint32_t a = Mtime::micros();
        P::idle();
        slept += Mtime::micros() - a;
        ++calls;
    }
    const uint32_t span = Mtime::micros() - u0;
    brio_idle_calls = calls;
    brio_idle_span_us = span;
    brio_idle_awake_us = span - slept;
    brio_awake_permille = span != 0u ? (span - slept) * 1000u / span : 1000u;
}

int main() {
    brio_clock_ok = SysClock::init() ? 1u : 0u;

    // The microsecond ruler: the kernel timebase stands on it on the
    // RISC-V half, and it is what the idle measurement is timed with on
    // both.
    (void)Mtime::start(clock);
    (void)Ticker::init(clock);

    const ChipId id = ChipId::read();
    brio_chip_id = (static_cast<uint32_t>(id.revision) << 28) |
                   (static_cast<uint32_t>(id.part) << 12) |
                   (static_cast<uint32_t>(id.manufacturer) << 1) | 1u;
    brio_package_pins = static_cast<uint32_t>(ChipId::package_sel());
    brio_core = P::core_id();
    brio_arch = core_kind == CoreKind::hazard3 ? 3u : 0u;

    brio_clk_sys_hz = SysClock::count_hz(CountSource::clk_sys).value_or(0u);
    brio_clk_ref_hz = SysClock::count_hz(CountSource::clk_ref).value_or(0u);
    brio_clk_peri_hz = SysClock::count_hz(CountSource::clk_peri).value_or(0u);

    (void)Led::output(false);
    enable_interrupts();

    // The interrupt path, end to end, on a line that reaches nothing:
    // enabled in the controller, raised in software, served once.
    Irq::enable(SPARE_IRQ_0_IRQn);
    Irq::set_pending(SPARE_IRQ_0_IRQn);

    measure_idle();

    Kernel::run();
}
