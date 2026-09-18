// PWM family smoke TU: the block with BOTH of its interrupt lines, a
// slice's every verb, and the five tasks - the four that want a pad
// instantiated on pins every package brings out, the ones that want a
// slice above the seventh behind the package fact the stratum states, so
// that this file compiles for the QFN-60 (where slices 8..11 reach no
// pad) as it does for the QFN-80.
#include "rp2350/clock.hpp"
#include "rp2350/pwm.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;

static_assert(pwm_slice_count == 12u);
static_assert(pwm_irq_lines == 2u);

// The two interrupt lines are two system lines, and the numbering is the
// same on both architectures (3.8.4.2).
static_assert(Pwm::irq<0>() == PWM_IRQ_WRAP_0_IRQn);
static_assert(Pwm::irq<1>() == PWM_IRQ_WRAP_1_IRQn);
static_assert(Pwm::reset_bit == ResetBlock::pwm);

// Every slice's wrap is its own data request, twelve of them.
static_assert(PwmSlice<0>::dreq == Dreq::pwm_wrap0);
static_assert(PwmSlice<8>::dreq == Dreq::pwm_wrap8);
static_assert(PwmSlice<11>::dreq == Dreq::pwm_wrap11);

// The package decides which slices have a pad, and nothing else about
// them: the register file is the die's.
static_assert(pwm_slice_has_pads(7));
static_assert(pwm_slice_has_pads(11) == (gpio_count > 32u));

/// One slice, every verb it has, on both interrupt lines.
template <uint8_t n>
void slice_verbs() {
    using S = PwmSlice<n>;

    // The five registers, READ (a cast to void of a volatile reference
    // would access nothing).
    const uint32_t regs = S::csr() + S::div() + S::ctr() + S::cc() + S::top_reg();
    (void)regs;
    (void)S::cc_address();
    (void)S::top_address();

    (void)S::configure({.mode = PwmDivMode::free_running, .divider = {2, 8}, .top = 999,
                        .phase_correct = true, .invert_a = true, .invert_b = true});
    (void)S::configure({.mode = PwmDivMode::level_high, .divider = {0, 0}, .top = 0xFFFF});
    (void)S::configure({.mode = PwmDivMode::rising_edge});
    (void)S::configure({.mode = PwmDivMode::falling_edge});
    const PwmSliceConfig back = S::config();
    (void)back.mode;
    (void)pwm_slice_config_valid(back);
    (void)pwm_csr_of(back);
    (void)pwm_period_sixteenths(back);
    (void)pwm_output_hz(SysClock::hz, back);

    S::enable(true);
    (void)S::enabled();
    S::enable(false);

    S::level(0, 100);
    S::level(1, 200);
    S::levels(100, 200);
    (void)S::level(0);
    (void)S::level(1);
    S::top(999);
    (void)S::top();
    S::counter(0);
    (void)S::counter();
    (void)S::advance_phase();
    (void)S::advance_phase(10u);
    (void)S::retard_phase();
    (void)S::retard_phase(10u);

    S::template interrupt<0>(true);
    S::template interrupt<1>(true);
    S::interrupt(false);                 // the default line
    (void)S::template pending<0>();
    (void)S::template pending<1>();
    (void)S::pending();
    (void)S::raw_pending();
    S::clear_pending();
}

/// The tasks that want a pad, on pins of the slice `n` - instantiated
/// only where the package brings those pads out, which is what makes
/// this file compile for the QFN-60 too.
template <uint8_t pin_a, bool bonded>
void pad_tasks() {
    if constexpr (bonded) {
        constexpr uint8_t pin_b = pin_a + 1u;
        using Out = PwmOutput<pin_a, 999>;
        using OutB = PwmOutput<pin_b, 999>;
        using Pair = PwmPair<pin_a, pin_b, 999>;
        using Edges = PwmEdgeCounter<pin_b>;
        using Level = PwmLevelCounter<pin_b>;

        static_assert(Out::max == 1000u);
        static_assert(Out::channel == 0u && OutB::channel == 1u);
        static_assert(PwmChannel<Out>);
        static_assert(PwmChannel<OutB>);

        (void)Out::setup();
        (void)Out::setup(PwmDivider{4, 0}, true, true);
        (void)Out::setup_hz(SysClock{}, 10'000u);
        (void)Out::setup_hz(SysClock{}, 10'000u, true, true);
        Out::attach();
        Out::duty(500);
        (void)Out::duty();
        Out::release();

        OutB::attach();
        OutB::duty(1000);
        OutB::release();

        (void)Pair::setup();
        (void)Pair::setup(PwmDivider{4, 0}, 100, true);
        Pair::duty(400);
        (void)Pair::dead_time();
        Pair::release();

        (void)Edges::setup();
        (void)Edges::setup(true, PwmDivider{0, 0}, 0xFFFF, PinPull::up);
        Edges::run(true);
        Edges::restart();
        (void)Edges::count();
        Edges::release();

        (void)Level::setup();
        (void)Level::setup(PwmDivider{0, 0}, 0xFFFF, PinPull::down);
        Level::run(true);
        Level::restart();
        (void)Level::count();
        Level::release();
    }
}

/// The wrap as a tick, on either line and on a slice of either half of
/// the block - no pad is wanted, so the QFN-60's four padless slices are
/// exactly what this task is for there.
template <uint8_t n, uint8_t line>
void tick_task() {
    using Tick = PwmPeriodicTick<n, line>;
    static_assert(Tick::flag == PwmSlice<n>::bit);
    static_assert(Tick::irq() == Pwm::irq<line>());
    (void)Tick::setup(PwmDivider{1, 0}, 1249);
    (void)Tick::setup(PwmDivider{1, 0}, 1249, false);
    (void)Tick::setup_hz(SysClock{}, 1000u);
    (void)Tick::setup_hz(SysClock{}, 1u);          // the solved-divider path
    Tick::stop();
}

void block_verbs() {
    (void)Pwm::reset();
    (void)Pwm::released();
    Pwm::hold();

    Pwm::start(Pwm::all_slices);
    (void)Pwm::running();
    Pwm::stop(Pwm::all_slices);

    (void)Pwm::raw_pending();
    Pwm::clear_pending(Pwm::all_slices);

    Pwm::interrupts<0>(PwmSlice<0>::bit, true);
    Pwm::interrupts<1>(PwmSlice<11>::bit, true);
    Pwm::interrupts(PwmSlice<3>::bit, false);      // the default line
    (void)Pwm::interrupts<0>();
    (void)Pwm::interrupts<1>();
    (void)Pwm::pending<0>();
    (void)Pwm::pending<1>();
    (void)Pwm::pending();
    Pwm::force<0>(PwmSlice<0>::bit, true);
    Pwm::force<1>(PwmSlice<11>::bit, false);
    Pwm::force(PwmSlice<1>::bit, false);
    (void)Pwm::isr<0>();
    (void)Pwm::isr<1>();
    (void)Pwm::isr();

    Irq::enable(Pwm::irq<0>());
    Irq::enable(Pwm::irq<1>());
    Irq::disable(Pwm::irq<1>());
}

void pwm() {
    block_verbs();

    slice_verbs<0>();
    slice_verbs<7>();
    slice_verbs<8>();      // a slice the RP2040 has not, and its registers
    slice_verbs<11>();     // exist in both packages of this chip

    // Slice 6 on GPIO 12/13 and slice 7 on GPIO 14/15: pads every package
    // of this chip brings out.
    pad_tasks<12, true>();
    pad_tasks<14, true>();
    // Slice 8 on GPIO 32/33, and the repeat of the same channels on GPIO
    // 40/41: the QFN-80's alone.
    pad_tasks<32, pwm_slice_has_pads(8)>();
    pad_tasks<40, pwm_slice_has_pads(8)>();

    tick_task<3, 0>();
    tick_task<3, 1>();
    tick_task<11, 0>();    // a padless slice as a repeating timer
    tick_task<11, 1>();
}
