// Rubato runs on a preemptive platform only: Stm32f4Platform has no
// ceiling and no scheduling request, so the kernel refuses it.
// mcu: stm32f446xx stm32f429xx stm32f411xe
#include "kernel/event_queue.hpp"
#include "kernel/rubato.hpp"
#include "stm32f4/platform.hpp"

using P = brio::Stm32f4Platform<>;

struct Ao {
    struct Event { uint8_t n; };
    static inline brio::EventQueue<Event, 2, P> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

void f() { (void)brio::Rubato<P, Ao>::activate(); }
