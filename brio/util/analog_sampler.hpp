/*
 * analog_sampler.hpp (util)
 *
 * AnalogSampler: the active object (an object with its own event queue,
 * dispatched by the kernel loop) that owns one analog-to-digital
 * converter, walks a fixed list of inputs and publishes every result
 * as a value event to the active objects that subscribe to it. A
 * program that wants its readings as events, each labelled with the
 * input it was taken on, names the converter, the inputs and the
 * subscribers, binds the converter's result vector, and picks a pace.
 * The smallest use, in the AVR's spelling:
 *
 *     using Sampler = brio::AnalogSampler<Adc, P, brio::Subscribers<Monitor, Alarm>,
 *                                         Loop{}, brio::AdcInput::temp, brio::AdcInput::vdd_div10>;
 *
 *     // in main, before Tenuto runs: the owner configures the converter
 *     Adc::init<adc_cfg>(clock);
 *     // in the program's handler for the converter's result vector:
 *     ISR(ADC0_RESRDY_vect) { brio::post<Sampler>(brio::Sampled{Adc::resrdy(), Adc::selected()}); }
 *     // and, from an event handler, the software pace:
 *     Sampler::start_every(brio::ticks_from_ms<P>(ms));
 *
 * Each subscriber handles AnalogSample{index, value}: index is the
 * input's position in the list, value the raw counts, and what they
 * mean is the subscriber's (it converts with util/analog.hpp). The page
 * this service belongs to is docs/design/analog.md, "The sampler is a
 * usage type, not an application".
 *
 * Who calls what, and from where:
 *  - the converter's result-ready interrupt handler body (the driver's)
 *    returns the value, and the program's vector binding posts
 *    Sampled{value, input} to the sampler, reading the input code in
 *    the same handler. post() copies the event and never waits
 *    (kernel/post.hpp); a full queue drops it and counts it in
 *    Sampler::queue.overflows();
 *  - the sampler runs in the kernel loop, in main context: its init()
 *    (called by Tenuto, in list order, before the first event) claims
 *    every input and selects the first; a Sampled whose input code is
 *    in the list is published as an AnalogSample and the next input
 *    selected, one whose code is not is dropped and counted
 *    (unknown_inputs()) and the input after the last known one is
 *    selected; each SamplerTick of the
 *    software pace starts one conversion. Nothing here waits or masks
 *    beyond what the converter's own select and start do;
 *  - start_every(), stop() and running_every() drive a TimeEvent, so
 *    they run in main context only (kernel/time_event.hpp), never in an
 *    interrupt handler;
 *  - subscribers receive AnalogSample by value, copied into their own
 *    queues by publish(): nothing is lent, nothing to give back.
 *
 * A usage type, not an application: what it knows is "a list of inputs,
 * a pace, samples out"; what it does not know is what the samples mean,
 * where the pace comes from, or which silicon converts.
 *
 * The converter is a type satisfying AnalogConverter (a target's own
 * ADC; a fake in host tests, test/test_analog_sampler): it can start
 * one conversion, say which input is selected, and select each input
 * of the list (the inputs are values of the converter's own input
 * vocabulary - pin tags, internal sources - given as template
 * arguments, so selection is an overload resolved at compile time, and
 * an input the converter cannot take is a compile error).
 *
 * Two paces, one sampler:
 *  - software: start_every(ticks) arms a periodic TimeEvent; every tick
 *    starts one conversion. Kernel-tick granularity (>= 1 ms on a 1024
 *    Hz ticker), no hardware beyond the converter.
 *  - hardware: the application routes any event generator (a PIT
 *    divider, a timer overflow, a pin, ...) to the converter's start
 *    input, in that target's own vocabulary, and never calls
 *    start_every: the sampler only receives results.
 *    That is the precise pace - jitter-free, in standby too - and the
 *    sampler does not care which generator: the pace is the app's
 *    choice, made where the hardware is wired.
 *
 * Attribution by the reported code, never by timing. Sampled carries
 * the input code the converter had selected for that conversion (read
 * in the interrupt handler, with the value): attribution never depends
 * on the sampler's dispatch being on time - a late dispatch, a hardware
 * pace faster than the dispatch, or a queue overflow can delay or lose
 * samples, never mislabel them. On each Sampled the sampler publishes
 * AnalogSample{index, value} and selects the input after the one just
 * reported, so the next conversion walks the list. A code not in the
 * list is dropped and counted (unknown_inputs()).
 *
 * The owner's duties stay with the application: configuring the
 * converter before init() (reference, accumulation: the sampler does
 * not reconfigure), pausing a hardware pace before a dynamic clock
 * change (docs/design/clock.md), and never selecting an input behind
 * the sampler's back.
 *
 * Two optional verbs of the converter, used when it has them: claim(in)
 * - the sampler claims each input of its list once at init (a pin's
 * digital input buffer off), so that a select is the mux store alone;
 * and a constant `sample_bits`, the width of what the converter delivers
 * as configured - Sampled.value is 16 bits, and a converter declaring
 * more (an unscaled sum of many samples) is refused at compile time.
 *
 * A converter whose input mux is not buffered (a select landing during
 * a conversion corrupts it) keeps the promise in its own select: under
 * a hardware pace the driver aborts the conversion in flight before it
 * moves the mux and re-arms the pace after - one dropped or aborted
 * trigger at worst, never a result under another input's label (the
 * AVR EA's adc.hpp; docs/design/analog.md, "The sampler is a usage
 * type, not an application", gives the sweep that measured it).
 *
 * The contract wants a converter that reports one result per interrupt
 * with the selected input readable from a register - which a converter
 * carrying a hardware sequencer and DMA also satisfies, because the
 * sampler uses neither: it walks its own list and reads the code the
 * driver reports. A source that delivers a whole block per interrupt is
 * a different economy and belongs to util/block_stream.hpp instead.
 * Pacing from a hardware event survives everywhere, because the sampler
 * never names the trigger: the app's wiring does, and that is target
 * glue. (docs/design/overview.md, "Authority of util/".)
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <utility>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"

namespace brio {

/// What AnalogSampler needs of a converter type.
template <typename C>
concept AnalogConverter = requires {
    { C::start() } -> std::same_as<void>;
    { C::selected() } -> std::convertible_to<uint8_t>;
};

/// ...and of each input in its list: selectable, and with a constexpr
/// code - the one selected() reports while it is in effect.
template <typename C, auto in>
concept SamplerInput = requires {
    { C::select(in) } -> std::same_as<void>;
    { std::integral_constant<uint8_t, C::input_code(in)>{} };
};

/// Posted by the application's interrupt glue: one conversion result
/// and the input code the converter reported for it, both read in the
/// same handler.
struct Sampled {
    uint16_t value;
    uint8_t input;
};

/// The sampler's own periodic tick (software pace).
struct SamplerTick {};

/// Published to the subscribers: the raw result of input `index` (its
/// position in the sampler's list). Meaning and scaling are theirs.
struct AnalogSample {
    uint8_t index;
    uint16_t value;
};

/// The width a converter declares for what it delivers (its constant
/// `sample_bits`), or 16 where it declares none.
template <typename C>
constexpr uint8_t converter_sample_bits() {
    if constexpr (requires { C::sample_bits; }) {
        return static_cast<uint8_t>(C::sample_bits);
    } else {
        return 16;
    }
}

template <AnalogConverter C, Platform P, typename Subs, auto... inputs>
    requires (sizeof...(inputs) > 0) && (SamplerInput<C, inputs> && ...)
class AnalogSampler
    : public Fsm<AnalogSampler<C, P, Subs, inputs...>, Sampled, SamplerTick> {
    using Base = Fsm<AnalogSampler<C, P, Subs, inputs...>, Sampled, SamplerTick>;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    static constexpr uint8_t input_count = sizeof...(inputs);

    static_assert(converter_sample_bits<C>() <= 16,
                  "AnalogSampler: Sampled.value is 16 bits and this converter delivers more as "
                  "configured (an unscaled sum of many samples) - scale the sum, or accumulate fewer");

    /// Depth: results arrive from the interrupt handler at the pace; a
    /// slow dispatch elsewhere queues a few. Overflows are counted by the
    /// queue (queue.overflows()), and a dropped Sampled is a lost sample,
    /// never a mislabelled one.
    static inline EventQueue<Event, 4, P> queue;

    /// Claims every input of the list (where the converter has claim()),
    /// selects the first and waits for results. Called by Tenuto, in
    /// main context, before the first event; the converter must already
    /// be configured (its own init) by the owner. Starts no conversion
    /// and arms no pace.
    static void init() {
        index_ = 0;
        unknown_ = 0;
        (claim(inputs), ...);
        select_index(0);
        Base::start(&running);
    }

    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Software pace: one conversion every `period_ticks` kernel ticks
    /// (main context only, like every TimeEvent). Calling it again
    /// re-paces; stop() disarms the pace (results of a conversion already
    /// started still arrive and are published); running_every() says
    /// whether the software pace is armed. A hardware pace is the
    /// application's to start and stop, not these.
    static void start_every(uint32_t period_ticks) { tick_.arm_every(period_ticks); }
    static void stop() { tick_.disarm(); }
    static bool running_every() { return tick_.armed(); }

    /// Results whose input code is not in the list: dropped, no
    /// AnalogSample published, and the input after the last one whose
    /// code WAS in the list is selected all the same (the second input,
    /// when no known code has come since init()). Saturating at 255.
    static uint8_t unknown_inputs() { return unknown_; }

private:
    static Status running(const Event& e) {
        return match(e,
            [](Sampled s) {
                const uint8_t i = index_of(s.input);
                if (i < input_count) {
                    index_ = i;
                    publish(Subs{}, AnalogSample{i, s.value});
                } else if (unknown_ != 0xFF) {
                    ++unknown_;
                }
                select_index(static_cast<uint8_t>((index_ + 1) % input_count));
                return Base::handled();
            },
            [](SamplerTick) { C::start(); return Base::handled(); },
            [](auto) { return Base::unhandled(); });
    }

    /// C::claim(in) where the converter has it: once, at init.
    template <typename In>
    static void claim(In in) {
        if constexpr (requires { C::claim(in); }) {
            C::claim(in);
        }
    }

    /// C::select(inputs[i]) - the pack unrolled into a chain of compares.
    static void select_index(uint8_t i) {
        uint8_t k = 0;
        ((k++ == i ? (C::select(inputs), void()) : void()), ...);
    }

    static constexpr uint8_t index_of(uint8_t code) {
        constexpr uint8_t codes[] = {static_cast<uint8_t>(C::input_code(inputs))...};
        for (uint8_t i = 0; i < input_count; ++i) {
            if (codes[i] == code) return i;
        }
        return input_count;
    }

    static constexpr bool codes_unique() {
        constexpr uint8_t codes[] = {static_cast<uint8_t>(C::input_code(inputs))...};
        for (uint8_t i = 0; i < input_count; ++i) {
            for (uint8_t j = static_cast<uint8_t>(i + 1); j < input_count; ++j) {
                if (codes[i] == codes[j]) return false;
            }
        }
        return true;
    }
    static_assert(codes_unique(), "AnalogSampler: the same input twice in the list");

    static inline TimeEvent<P, AnalogSampler, SamplerTick> tick_{SamplerTick{}};
    static inline uint8_t index_ = 0;
    static inline uint8_t unknown_ = 0;
};

} // namespace brio
