// PFIC family smoke TU: the global mask that is a CSR bit, the guard
// built on it, every per-line verb of the controller, and the handler
// attribute in BOTH of its expansions - the fixture compiles this file
// with BRIO_CH32_HPE=0 and =1, which is the only way one spelling with
// two meanings gets proven.
#include "ch32vx03/pfic.hpp"

using namespace brio;

static_assert(mstatus_mie == (1u << 3));

// The controller's banks are eight words wide, so a line number is a
// word index and a bit: the tail of this family's table (up to 69 on the
// larger device class) fits the second word.
static_assert(static_cast<uint32_t>(Irq::systick) >> 5 == 0u);
static_assert(static_cast<uint32_t>(Irq::usart1) >> 5 == 1u);
static_assert(device::vector_count <= 8u * 32u);

void pfic_verbs() {
    (void)interrupts_enabled();
    enable_interrupts();
    disable_interrupts();

    Pfic::enable(Irq::systick);
    (void)Pfic::enabled(Irq::systick);
    Pfic::set_pending(Irq::software);
    (void)Pfic::pending(Irq::software);
    Pfic::clear_pending(Irq::software);
    (void)Pfic::active(Irq::usart1);
    Pfic::disable(Irq::systick);

    // The line whose number moves with the device class, reached by name.
    Pfic::enable(Irq::uart4);
    Pfic::disable(Irq::uart4);
}

// The guard, nested: the inner one must leave interrupts masked on its
// way out when the outer one found them masked.
uint32_t guarded_sum(const volatile uint32_t* p) {
    InterruptGuard outer;
    uint32_t total = *p;
    {
        InterruptGuard inner;
        total += *p;
    }
    return total;
}

// A vector binding, which is what the attribute is for: the app's one
// piece of vendor glue, and the thing that differs between the two HPE
// builds.
extern "C" BRIO_CH32_INTERRUPT void systick_handler() {
    Pfic::clear_pending(Irq::systick);
}
