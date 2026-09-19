// TRNG family smoke TU: every verb of the entropy source instantiated
// once, the compile-time configuration checked both ways, and the four
// status bits pinned to the register description.
#include <array>
#include <type_traits>

#include "rp2350/trng.hpp"

using namespace brio;

// The four sources RNG_ISR, RNG_ICR and RNG_IMR share (12.12.5), in the
// order the register lists them.
static_assert(TrngFlag::ehr_valid == 0x1u);
static_assert(TrngFlag::autocorr_error == 0x2u);
static_assert(TrngFlag::crngt_error == 0x4u);
static_assert(TrngFlag::von_neumann_error == 0x8u);
static_assert(TrngFlag::all == 0xFu);
// The two a program can clear and go round again on; AUTOCORR_ERR is not
// one of them, which is the whole shape of this driver's error handling.
static_assert(TrngFlag::recoverable == (TrngFlag::crngt_error | TrngFlag::von_neumann_error));
static_assert((TrngFlag::recoverable & TrngFlag::autocorr_error) == 0u);

static_assert(Trng::reset_block == ResetBlock::trng);
static_assert(Trng::irq == TRNG_IRQ_IRQn);
static_assert(Trng::entropy_bits == 192);
static_assert(Trng::min_sample_cycles_no_von_neumann == 17);

// THE DEFAULT SAMPLING INTERVAL IS A TIME, NOT A CYCLE COUNT. 12.12.2's
// "sample count settings of 20-25" is written with no clock rate beside
// it, and measured on this silicon an interval under about 800 ns stops
// the block on AUTOCORR_ERR - so the default is `trng_default_sample_ns`
// at the family's highest clk_sys, which is at or over the floor at
// every rate below it.
static_assert(TrngConfig{}.chain <= 1u);
static_assert(trng_min_sample_ns == 800u);
static_assert(trng_default_sample_ns >= 2u * trng_min_sample_ns);
static_assert(TrngConfig{}.sample_cycles == trng_sample_cycles_for(clk_sys_max_hz));
static_assert(TrngConfig{}.sample_cycles == 300u);
static_assert(TrngConfig{}.autocorrelation && TrngConfig{}.crngt && TrngConfig{}.von_neumann);

// The arithmetic rounds UP, because the floor is a minimum, and never
// answers zero.
static_assert(trng_sample_cycles_for(150'000'000u, 2'000u) == 300u);
static_assert(trng_sample_cycles_for(12'000'000u, 2'000u) == 24u);
static_assert(trng_sample_cycles_for(150'000'000u, trng_min_sample_ns) == 120u);
static_assert(trng_sample_cycles_for(1u, 1u) == 1u);

// What the settings check admits and what it refuses.
static_assert(trng_config_legal(TrngConfig{}));
static_assert(trng_config_legal(TrngConfig{.chain = 3, .sample_cycles = 100}));
static_assert(!trng_config_legal(TrngConfig{.chain = 4}));
static_assert(!trng_config_legal(TrngConfig{.sample_cycles = 0}));
static_assert(!trng_config_legal(TrngConfig{.sample_cycles = 16, .von_neumann = false}));
static_assert(trng_config_legal(TrngConfig{.sample_cycles = 17, .von_neumann = false}));

// 192 bits is six words, which is what the read returns.
static_assert(std::tuple_size_v<decltype(TrngEntropy{}.words)> == 6);
static_assert(Trng::entropy_bits == 6 * 32);

void trng_verbs() {
    (void)Trng::init();
    (void)Trng::init(TrngConfig{.chain = 0, .sample_cycles = 20});
    (void)Trng::init<TrngConfig{.chain = 1, .sample_cycles = 25}>();
    (void)Trng::init<TrngConfig{.sample_cycles = 17, .von_neumann = false}>();
    (void)Trng::regs().RNG_ISR;

    Trng::start();
    (void)Trng::running();
    (void)Trng::busy();
    (void)Trng::valid();
    (void)Trng::status();
    Trng::clear(TrngFlag::recoverable);
    Trng::interrupt_mask(TrngFlag::all);
    (void)Trng::interrupt_mask();

    (void)Trng::read();
    (void)Trng::read_blocking();
    (void)Trng::read_blocking(1000u);
    (void)(Trng::last_error() == TrngError::not_ready);
    (void)(Trng::last_error() == TrngError::autocorrelation);
    (void)(Trng::last_error() == TrngError::crngt);
    (void)(Trng::last_error() == TrngError::von_neumann);
    (void)(Trng::last_error() == TrngError::misconfigured);
    (void)Trng::recoverable_failures();

    Trng::stop();
    (void)Trng::reset_bit_counter();
    Trng::sw_reset();
    (void)Trng::recover();
    (void)Trng::recover(TrngConfig{.chain = 2});

    const Trng::AutocorrStats stats = Trng::autocorr_stats();
    (void)stats.tries;
    (void)stats.failures;
    Trng::clear_autocorr_stats();
    (void)Trng::bist();
    (void)Trng::version();
    (void)Trng::ehr_is_192_bits();
    (void)Trng::has_autocorrelation();
    (void)Trng::has_crngt();
    (void)Trng::chain();
    (void)Trng::sample_cycles();
    (void)Trng::debug_control();
    (void)Trng::debug_mode();
    Trng::release();
}

// The ISR body an app binds to `isr_trng`.
void trng_isr_body() {
    const TrngEvent e = Trng::isr();
    (void)e.ready;
    (void)e.autocorr_error;
    (void)e.crngt_error;
    (void)e.von_neumann_error;
}
