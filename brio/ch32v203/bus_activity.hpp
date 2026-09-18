/*
 * bus_activity.hpp
 *
 * HOW MANY BUS MASTERS OTHER THAN THE CORE ARE WORKING RIGHT NOW - one
 * number, kept by the drivers that make one work and read by the
 * platform's idle path and by the sleep sites.
 *
 * WHY THIS FAMILY NEEDS IT. Measured on the silicon and against the
 * reference manual, which says the opposite: in the Sleep of RM 2.3.2
 * ("the core stops running and all peripherals are still running") NO
 * BUS MASTER BUT THE CORE GETS A CYCLE. A memory-to-memory DMA started
 * just before the sleep moves the handful of items already in its
 * pipeline and then nothing until the core wakes; the USB device
 * controller cannot reach its packet memory, so an armed endpoint
 * loses the host's bytes and an enumeration dies in its first control
 * transfer. The clocks run - the timers and the core's own counter
 * count the whole sleep - and no software mitigation short of staying
 * awake works (docs/ch32v203/README.md carries the measurements, taken
 * on this driver and on the vendor's own example as the oracle).
 *
 * So on this family A SLEEP OF ANY DEPTH IS LEGAL ONLY WHILE THE CORE
 * IS THE ONLY MASTER ON THE BUS, and that is a fact the requester of a
 * sleep does not have: the DMA and the USB driver do. This counter is
 * how they say it, once, in a place both they and the platform can
 * reach without either including the other.
 *
 * WHO COUNTS. A DMA channel while it is enabled (ch32v203/dma.hpp's
 * `DmaChannel::enable()` counts the EN transition, so every engine and
 * every task built on a channel is counted exactly once through it),
 * and the USB device controller from the moment its pull-up goes up to
 * the moment it comes down (ch32v203/usb.hpp's `Usbd::connect()`).
 * Nothing else on this part is a bus master.
 *
 * WHAT A CHANNEL THAT HAS FINISHED COUNTS AS. EN stays SET when a
 * non-circular block completes on this silicon - only software clears
 * it - so a channel whose owner has not disabled it still counts, and
 * a program that loads a channel and never takes it down does not
 * sleep again. That is the honest answer and not a leak: the count
 * says what the registers say, and `Dma::any_enabled()` asked of the
 * silicon would say the same. The engines of dma.hpp disable at
 * completion, so a transport pays nothing for it.
 *
 * WHAT IS BOUGHT WITH IT. The rule that used to be written in prose -
 * "a program that uses USB never idles" - becomes a MECHANISM: a
 * program is free to call the kernel's idle path, and the silicon's
 * truth decides whether that path sleeps or returns at once
 * (ch32v203/platform.hpp's idle()). The cost is one byte read per idle
 * and a guarded increment inside verbs that are already storing to a
 * peripheral register.
 *
 * WHY A COUNT AND NOT A POLL. `Dma::any_enabled()` already answers the
 * DMA's half by reading eight configuration registers, but the USB's
 * half is not one register's business (the pull-up lives in EXTEN and
 * the controller's state in its own block), and the idle path pays for
 * whichever answer it asks for on EVERY pass. One byte in RAM is the
 * cheapest true answer, and it is the only one a third master could
 * join without the idle path learning its name.
 *
 * THE FAILURE DIRECTION IS DELIBERATE. The counter saturates at 255 and
 * floors at zero: an unbalanced increment leaves the program AWAKE for
 * ever, which wastes current, and an unbalanced decrement can only be
 * reached by a driver decrementing what it never incremented. There is
 * no verb to reset it, because a leak is a bug to be found and not a
 * state to be papered over - `active()` is what a suite reads to find
 * one.
 */

#pragma once

#include <stdint.h>

#include "ch32v203/pfic.hpp"

namespace brio {

/**
 * The count, monostate. Increment and decrement are read-modify-write
 * on a byte two contexts touch (a channel enabled in the loop, a
 * completion handler disabling it), so both run under the platform's
 * own critical section; the READ is a plain volatile load, which is
 * atomic on this core and is what the idle path pays.
 */
struct BusActivity {
    BusActivity() = delete;

    /// How many masters other than the core are working. Zero is the
    /// only value at which a sleep of any depth is legal here.
    static uint8_t active() { return *const_cast<const volatile uint8_t*>(&count_); }

    /// A master started. Saturates rather than wrapping: see the file
    /// header for why that is the safe direction.
    static void entered() {
        InterruptGuard guard;
        if (count_ < 0xFFu) {
            ++count_;
        }
    }

    /// A master stopped.
    static void left() {
        InterruptGuard guard;
        if (count_ != 0u) {
            --count_;
        }
    }

private:
    static inline uint8_t count_ = 0;
};

} // namespace brio
