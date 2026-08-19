/*
 * analog_sampler.hpp (util)
 *
 * AnalogSampler: the active object that owns ONE analog-to-digital
 * converter, walks a fixed list of inputs and publishes every result
 * as a value event. A usage type, not an application: what it knows is
 * "a list of inputs, a pace, samples out"; what it does not know is
 * what the samples mean (the subscribers convert with util/analog.hpp),
 * where the pace comes from, or which silicon converts.
 *
 * The converter is a type satisfying AnalogConverter (avrdx/adc.hpp's
 * Adc<0> on AVR DA/DB; a fake in host tests): it can start one
 * conversion, say which input is selected, and select each input of
 * the list (the inputs are values of the converter's own input
 * vocabulary - pin tags, internal sources - given as template
 * arguments, so selection is an overload resolved at compile time).
 *
 * Two paces, one sampler:
 *  - SOFTWARE: start_every(ticks) arms a periodic TimeEvent; every tick
 *    starts one conversion. Kernel-tick granularity (>= 1 ms on a 1024
 *    Hz ticker), no hardware beyond the converter.
 *  - HARDWARE: the application routes any event generator (a PIT
 *    divider, a timer overflow, a pin, ...) to the converter's start
 *    input (on AVR: EventChannel<n>::source(gen); Adc<0>::start_on(ch))
 *    and never calls start_every: the sampler only receives results.
 *    That is the precise pace - jitter-free, in standby too - and the
 *    sampler does not care WHICH generator: the pace is the app's
 *    choice, made where the hardware is wired.
 *
 * The result path is the kernel's ISR contract: the driver's
 * result-ready ISR body returns the value, the application's vector
 * binding posts Sampled{value, input} to this AO:
 *
 *   ISR(ADC0_RESRDY_vect) { post<Sampler>(Sampled{Adc<0>::resrdy(), Adc<0>::selected()}); }
 *
 * Sampled carries the input CODE the converter had selected for that
 * conversion (read in the ISR, with the value): attribution never
 * depends on the sampler's dispatch being on time - a late dispatch,
 * a hardware pace faster than the dispatch, or a queue overflow can
 * delay or lose samples, never mislabel them. On each Sampled the
 * sampler publishes AnalogSample{index, value} (index = position in
 * the list) and selects the NEXT input, so the next conversion walks
 * the list. A code not in the list is dropped and counted.
 *
 * The owner's duties stay with the application: configuring the
 * converter before init() (reference, accumulation: the sampler does
 * not reconfigure), pausing a hardware pace before a dynamic clock
 * change (docs/design/clock.md), and never selecting an input behind
 * the sampler's back.
 *
 * Validated on: AVR DA/DB (Adc<0>) and the host fake. The contract
 * carries that silicon's shape - ONE result per interrupt and the
 * selected input readable from a register. On a converter with a
 * hardware sequencer and DMA (ATSAM, STM32) the natural delivery is a
 * block per interrupt and the selected input is a count the driver
 * keeps; the walk here becomes vestigial or the type changes. The
 * pace-by-hardware-event idea survives everywhere (an event system on
 * ATSAM; a fixed, short list of timer TRGO triggers on STM32/CH32):
 * the sampler never names the trigger, the app's wiring does and is
 * target glue. (docs/design/overview.md, "Authority of util/".)
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

/// Posted by the application's ISR glue: one conversion result and the
/// input code it was taken on.
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

template <AnalogConverter C, Platform P, typename Subs, auto... inputs>
    requires (sizeof...(inputs) > 0) && (SamplerInput<C, inputs> && ...)
class AnalogSampler
    : public Fsm<AnalogSampler<C, P, Subs, inputs...>, Sampled, SamplerTick> {
    using Base = Fsm<AnalogSampler<C, P, Subs, inputs...>, Sampled, SamplerTick>;

public:
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    static constexpr uint8_t input_count = sizeof...(inputs);

    /// Depth: results arrive from the ISR at the pace; a slow dispatch
    /// elsewhere queues a few. Overflows are counted by the queue.
    static inline EventQueue<Event, 4, P> queue;

    /// Selects the first input and waits. The converter must already be
    /// configured (init) by the owner.
    static void init() {
        index_ = 0;
        unknown_ = 0;
        select_index(0);
        Base::start(&running);
    }

    static void dispatch(const Event& e) { Base::dispatch(e); }

    /// Software pace: one conversion every `period_ticks` kernel ticks
    /// (main context only, like every TimeEvent).
    static void start_every(uint32_t period_ticks) { tick_.arm_every(period_ticks); }
    static void stop() { tick_.disarm(); }
    static bool running_every() { return tick_.armed(); }

    /// Results whose input code is not in the list (dropped).
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
