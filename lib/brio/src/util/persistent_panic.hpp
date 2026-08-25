/*
 * persistent_panic.hpp
 *
 * The panic breadcrumb, kept across a power cycle.
 *
 * kernel/panic.hpp writes the PanicRecord into the platform's
 * reset-surviving storage - on a microcontroller that is a .noinit
 * object in SRAM, and SRAM is promised across nothing at all. It
 * survives a watchdog or software reset in practice; it does not
 * survive the plug being pulled, which is precisely the failure a
 * field unit comes back from with nothing to say.
 *
 * PersistentPanic is the reporter that closes that gap: one panic
 * Reporter that copies the record into a nonvolatile store, and one
 * boot-side verb that takes it back out. It composes with the rest of
 * the reporter vocabulary - "save, then reset and report at next boot"
 * is two reporters chained, and the breadcrumb was already written to
 * SRAM before any reporter ran, so nothing is lost either way.
 *
 * WHY POLLED, AND WHY THAT IS NOT A CONTRADICTION. util/nv_writer.hpp
 * exists so that nothing ever busy-waits on nonvolatile memory. A panic
 * reporter is the one context where that rule is inverted: interrupts
 * are masked for good (panic() holds a critical section that is never
 * destroyed), the kernel will never run again, and there is nothing
 * left to be responsive to. The record is written here with bounded
 * polled waits, and being slow is the correct behaviour - the only
 * thing that matters is that the bytes are down before the reporter
 * hands over.
 *
 * SIZE. Eight bytes of the store (four of header, four of record).
 * ENDURANCE. Written once per panic and cleared once per boot that
 * finds one, so the wear is one erase/write per failure, not per run.
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "util/nv_record.hpp"

namespace brio {

/**
 * A panic reporter that stores the record in nonvolatile memory, and
 * the boot-side verb that reads it back.
 *
 *   using Panic = brio::PersistentPanic<brio::EepromStore, 0>;
 *   ...
 *   brio::panic<P, Panic>(PanicCode::assert_failed, 7);   // at the fault
 *   ...
 *   auto old = Panic::take();                             // at boot
 *
 * `offset` is where the eight bytes live in the store; nothing else may
 * claim them.
 */
template <NvStore S, uint16_t offset = 0, uint8_t version = 1>
struct PersistentPanic {
    using Record = NvRecord<PanicRecord, S, offset, version>;

    PersistentPanic() = delete;

    static constexpr uint16_t begin = Record::begin;
    static constexpr uint16_t end = Record::end;

    /// The panic Reporter interface (kernel/panic.hpp): write the record
    /// and return, so a further reporter in the chain (a reset, a
    /// blinker) can do its own part. Interrupts are already masked by
    /// panic() when this runs.
    static void report(PanicCode code, uint8_t context) {
        (void)Record::store(PanicRecord{panic_magic,
                                        static_cast<uint8_t>(code), context});
    }

    /// Store a record that was not produced by panic() - the breadcrumb
    /// a boot found in SRAM, on its way to somewhere it will still be
    /// tomorrow. True when it reads back as what was asked for.
    static bool save(const PanicRecord& r) {
        (void)Record::store(r);
        const std::optional<PanicRecord> back = Record::load();
        return back && back->magic == r.magic && back->code == r.code &&
               back->context == r.context;
    }

    /// Peek without consuming.
    static std::optional<PanicRecord> peek() {
        const std::optional<PanicRecord> r = Record::load();
        if (r && r->magic != panic_magic) {
            return std::nullopt;
        }
        return r;
    }

    /// Boot-side fetch-and-clear, the twin of take_panic_record(): a
    /// stored record is returned ONCE and the slot is invalidated with a
    /// single byte write. A second call returns nothing.
    static std::optional<PanicRecord> take() {
        const std::optional<PanicRecord> r = peek();
        if (r) {
            (void)Record::clear();
        }
        return r;
    }

    /// True when a record is waiting.
    static bool pending() { return peek().has_value(); }
};

} // namespace brio
