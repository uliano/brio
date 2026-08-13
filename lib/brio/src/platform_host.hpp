/*
 * platform_host.hpp
 *
 * Host (native test) implementation of the brio Platform concept: the
 * critical section only tracks its nesting depth (tests are single
 * threaded), the clock is a plain counter the test advances by hand, and
 * idle()/break_here() record their calls so tests can assert on them.
 * Time becomes deterministic arithmetic.
 *
 * All state is static inline so tests reach it without plumbing; call
 * reset() at the start of each test case.
 */

#pragma once

#include <stdint.h>

#include "platform.hpp"

namespace brio {

struct HostPlatform {
    class CriticalSection {
    public:
        CriticalSection() { ++depth; }
        ~CriticalSection() { --depth; }
        CriticalSection(const CriticalSection&) = delete;
        CriticalSection& operator=(const CriticalSection&) = delete;

        /// Current nesting depth; must be back to 0 when no guard is alive.
        static inline uint8_t depth = 0;
    };

    static inline uint32_t ticks = 0;       ///< advanced by the test
    static inline uint32_t idle_calls = 0;  ///< how many times idle() ran
    static inline uint32_t break_calls = 0; ///< how many times break_here() ran

    static void idle() { ++idle_calls; }
    static void break_here() { ++break_calls; }
    static uint32_t now() { return ticks; }

    static void reset() {
        ticks = 0;
        idle_calls = 0;
        break_calls = 0;
        CriticalSection::depth = 0;
    }
};

static_assert(Platform<HostPlatform>);

} // namespace brio
