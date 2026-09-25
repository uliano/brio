// mcu: ch32v303rc ch32v303vc
// Random number generator family smoke TU: the three registers of RM
// ch. 29 and every verb of `Rng` - on the two parts that carry the block
// (the CH32V303 datasheet's table 2-1-1: the RC and the VC). The
// negative refuses the type everywhere else. No clock is asked: SYSCLK
// runs the block at every rate (rng.hpp's fact 1).
#include <stddef.h>

#include "ch32vx03/rng.hpp"

using namespace brio;

// ---- the register map (table 29-1) ------------------------------------------
static_assert(offsetof(RngRegs, CR) == 0x00);
static_assert(offsetof(RngRegs, SR) == 0x04);
static_assert(offsetof(RngRegs, DR) == 0x08);
static_assert(rng_base == 0x40023C00u);

// ---- the bits (29.3.1, 29.3.2) ----------------------------------------------
static_assert(rng_rngen == 0x04u && rng_ie == 0x08u);
static_assert(rng_drdy == 0x01u && rng_cecs == 0x02u && rng_secs == 0x04u);
static_assert(rng_ceis == 0x20u && rng_seis == 0x40u);
static_assert(rng_latched == (rng_ceis | rng_seis));
// The two latched flags sit four places above their current statuses,
// which is how the vendor's own library addresses them.
static_assert(rng_ceis == (rng_cecs << 4) && rng_seis == (rng_secs << 4));

// ---- the part fact and the vector -------------------------------------------
static_assert(device::has_rng);
static_assert(Rng::irq == Irq::rng);
static_assert(static_cast<uint8_t>(Rng::irq) == 63u);
static_assert(irq_exists(Rng::irq));
static_assert(rcc_hb_rng == (1UL << 9));

// ---- every verb ------------------------------------------------------------
void rng_verbs() {
    // the gate and the control register
    Rng::clock(true);
    (void)Rng::clock();
    Rng::enable(true);
    (void)Rng::enabled();
    Rng::interrupt(true);
    (void)Rng::interrupt();
    (void)Rng::regs();

    // the statuses and the latched flags
    (void)Rng::status();
    (void)Rng::ready();
    (void)Rng::seed_error();
    (void)Rng::clock_error();
    (void)Rng::seed_error_flag();
    (void)Rng::clock_error_flag();
    Rng::clear_seed_error();
    Rng::clear_clock_error();

    // the words
    (void)Rng::value();
    (void)Rng::read();
    (void)Rng::read_blocking();
    (void)Rng::read_blocking(16u);
    (void)Rng::last_error();

    // bring-up and recovery, which ask nothing of the clock tree
    (void)Rng::init();
    (void)Rng::recover();
    (void)Rng::discard_first();
    (void)Rng::discard_first(8u);

    // the ISR body and the teardown
    const RngEvent e = Rng::isr();
    (void)e.ready;
    (void)e.seed_error;
    (void)e.clock_error;
    Rng::release();
}

// The reasons a word is not handed out are four, and distinct.
static_assert(RngError::not_ready != RngError::seed_error);
static_assert(RngError::clock_error != RngError::repeated);
static_assert(RngError::repeated != RngError::not_ready);
