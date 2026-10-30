// mcu: stm32g0b1xx stm32g071xx stm32g031xx stm32g030xx
// Rubato runs on a preemptive platform only: Stm32g0Platform has no
// ceiling and no scheduling request, so the kernel refuses it.
#include "kernel/event_queue.hpp"
#include "kernel/rubato.hpp"
#include "stm32g0/platform.hpp"

using P = brio::Stm32g0Platform<>;

struct Ao {
    struct Event { uint8_t n; };
    static inline brio::EventQueue<Event, 2, P> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

void f() { (void)brio::Rubato<P, Ao>::activate(255); }
