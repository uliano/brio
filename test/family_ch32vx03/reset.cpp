// Reset family smoke TU: the six flags as RCC_RSTSCKR carries them, the
// read-and-clear verb, the core's reset request, the trap registers and
// the cause byte built from them, the reporter that resets and the fault
// body that records.
#include "ch32vx03/platform.hpp"
#include "ch32vx03/reset.hpp"
#include "kernel/panic.hpp"

using namespace brio;

using P = Ch32vx03Platform<>;

static_assert(ResetFlag::low_power == (1UL << 31));
static_assert(ResetFlag::software == (1UL << 28));
static_assert(ResetFlag::power == (1UL << 27));
static_assert(ResetFlag::pin == (1UL << 26));
static_assert((ResetFlag::all & rstsckr_rmvf) == 0u, "RMVF is not a flag");
static_assert((ResetFlag::all & (rcc_lsion | rcc_lsirdy)) == 0u,
              "the LSI bits are the clock tree's");
static_assert((ResetFlag::all & (1UL << 25)) == 0u, "bit 25 is reserved on this family");
static_assert(ResetFlag::watchdog == (ResetFlag::window_watchdog | ResetFlag::independent_watchdog));

// The register's own reset value: a power-on raises both of these.
static_assert((ResetFlag::power | ResetFlag::pin) == 0x0C000000UL);

// The reset request is the core's, and it is keyed.
static_assert(pfic_key3 == 0xBEEF0000UL);
static_assert(pfic_rstsys == (1UL << 7));

// The cause byte: the interrupt bit above, the code below.
static_assert(fault_interrupt_bit == 0x80u);
static_assert(!fault_was_interrupt(static_cast<uint8_t>(FaultCause::breakpoint)));
static_assert(fault_code(static_cast<uint8_t>(FaultCause::illegal_instruction)) == 2u);
static_assert(fault_was_interrupt(fault_interrupt_bit | 12u));
static_assert(fault_code(fault_interrupt_bit | 12u) == 12u);
static_assert(static_cast<uint8_t>(FaultCause::ecall_machine) == 11u);
static_assert(mcause_interrupt == (1UL << 31));

void reset_verbs() {
    (void)Reset::flags();
    Reset::clear_flags();
    (void)Reset::take_flags();
}

void trap_verbs() {
    (void)machine_cause();
    (void)machine_epc();
    (void)machine_tval();
    (void)fault_context();
    (void)fault_cause_name(fault_context());
}

[[noreturn]] void reset_now() { Reset::software(); }

[[noreturn]] void die() { panic<P, ResetReporter>(PanicCode::queue_overflow, 1); }

/// Both entry paths of the fault body: the core's own account of the
/// cause, and a byte the handler has chosen itself.
[[noreturn]] void fault_body() { fault_reset<P>(); }
[[noreturn]] void fault_body_labelled() { fault_reset<P>(0x51); }
