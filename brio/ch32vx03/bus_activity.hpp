/*
 * bus_activity.hpp
 *
 * How many bus masters other than the core are working right now - one
 * number, kept by the drivers that make one work and read by the
 * platform's idle path and by the sleep sites, which on this family
 * sleep only while it is zero. A program never writes it; it reads it
 * through the platform, as docs/ch32vx03/sleep.md's "How to use it"
 * spells it:
 *
 *     if (P::bus_masters_active() != 0) { ... a transfer is up: no sleep of any depth ... }
 *
 * Which context calls what. `active()` is one volatile byte load, legal
 * from any context, inside a masked window too; it is what the idle path
 * pays on every pass. `entered()` and `left()` are the drivers' alone,
 * called by the verbs that set or clear a DMA channel's EN and by the
 * two USB controllers' `connect()` - from main(), an active object's
 * handler or a completion handler - and each masks interrupts (the
 * platform's critical section, `InterruptGuard`) around one test and one
 * increment or decrement of the byte. Nothing here waits.
 *
 * Why this family needs it. Measured on the silicon and against the
 * reference manual, which says the opposite: in the Sleep of RM 2.3.2
 * ("the core stops running and all peripherals are still running") no
 * bus master but the core gets a cycle. A memory-to-memory DMA started
 * just before the sleep moves the handful of items already in its
 * pipeline and then nothing until the core wakes; the USB device
 * controller cannot reach its packet memory, so an armed endpoint
 * loses the host's bytes and an enumeration dies in its first control
 * transfer. The clocks run - the timers and the core's own counter
 * count the whole sleep - and no software mitigation short of staying
 * awake works (docs/ch32vx03/README.md carries the measurements, taken
 * on this driver and on the vendor's own example as the oracle).
 *
 * So on this family a sleep of any depth is legal only while the core
 * is the only master on the bus, and that is a fact the requester of a
 * sleep does not have: the DMA and the USB driver do. This counter is
 * how they say it, once, in a place both they and the platform can
 * reach without either including the other.
 *
 * Who counts. A DMA channel while it is enabled: ch32vx03/dma.hpp's
 * controller ledger (`Dma<c>::count()` and `uncount()`, one byte a
 * channel) takes the count when a verb sets EN and gives it back when a
 * verb clears it, and every such verb goes through it -
 * `DmaChannel::enable()` and `stop()`, the engines' block start and end,
 * `Dma<c>::stop_all()` - so a channel holds one count at most whichever
 * engine or task drives it. And a USB controller from the moment its
 * pull-up goes up to the moment it comes down - the device controller
 * through ch32vx03/usb.hpp's `Usbd::connect()`, the host/device
 * controller through ch32vx03/usbfs.hpp's `Usbfs::connect()`. Nothing
 * else on this part is a bus master.
 *
 * What a channel that has finished counts as. EN stays set when a
 * non-circular block completes on this silicon - only software clears
 * it - so a channel whose owner has not disabled it still counts, and
 * a program that loads a channel and never takes it down does not
 * sleep again. That is the honest answer and not a leak: the count
 * says what the registers say, and `Dma<1>::any_enabled()` asked of the
 * silicon would say the same - but for a transfer error, which clears
 * EN in hardware while the ledger keeps the count until a verb takes the
 * channel down (the ledger's comment in dma.hpp). The engines of dma.hpp
 * disable at completion, so a transport pays nothing for it.
 *
 * What is bought with it. The rule "a program that uses USB never
 * idles" is not prose a program has to keep: it is a mechanism. A
 * program is free to call the kernel's idle path, and the silicon's
 * truth decides whether that path sleeps or returns at once
 * (ch32vx03/platform.hpp's idle()). The cost is one byte read per idle
 * and a guarded increment inside verbs that are already storing to a
 * peripheral register.
 *
 * Why a count and not a poll. `Dma<1>::any_enabled()` already answers
 * the DMA's half by reading the configuration register of every channel
 * of every controller - eight on the CH32V203, eighteen on the
 * CH32V303 - but the USB's
 * half is not one register's business (the pull-up lives in EXTEN and
 * the controller's state in its own block), and the idle path pays for
 * whichever answer it asks for on every pass. One byte in RAM is the
 * cheapest true answer, and it is the only one a third master could
 * join without the idle path learning its name.
 *
 * The failure direction is deliberate. The counter saturates at 255 and
 * floors at zero: an unbalanced increment leaves the program awake for
 * ever, which wastes current, and an unbalanced decrement can only be
 * reached by a driver decrementing what it never incremented. There is
 * no verb to reset it, because a leak is a bug to be found and not a
 * state to be papered over - `active()` is what a suite reads to find
 * one.
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/pfic.hpp"

namespace brio {

/**
 * The count, monostate (a class with only static members, never
 * instantiated). Increment and decrement are read-modify-write on a
 * byte two contexts touch (a channel enabled in the loop, a completion
 * handler disabling it), so both run under the platform's own critical
 * section; the read is a plain volatile load, which is atomic on this
 * core and is what the idle path pays.
 */
struct BusActivity {
    BusActivity() = delete;

    /// How many masters other than the core are working. Zero is the
    /// only value at which a sleep of any depth is legal here. One load,
    /// from any context; a program asks it through the platform's
    /// `bus_masters_active()`.
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
