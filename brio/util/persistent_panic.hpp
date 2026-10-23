/*
 * persistent_panic.hpp
 *
 * The panic breadcrumb, kept across a power cycle. A program gets one
 * panic reporter that copies the record of a failure into a
 * nonvolatile store, and one verb that takes it back out at the next
 * boot, after a power cycle as after a reset.
 *
 * The smallest use names a store (an NvStore, util/nv_record.hpp: the
 * EEPROM behind avrxt/nvm.hpp's EepromStore on the AVR) and an offset,
 * passes the type to panic() as its reporter, and fetches the record
 * first thing at boot:
 *
 *     using Panic = brio::PersistentPanic<brio::EepromStore, 0>;
 *     brio::panic<P, Panic>(PanicCode::assert_failed, 7);   // at the fault
 *     auto old = Panic::take();                             // at boot
 *
 * Who calls what, and from where:
 *  - report() is called by panic() (kernel/panic.hpp), with interrupts
 *    masked for good, and writes with bounded polled waits (below). It
 *    runs only where the platform's break_here() returns; where that
 *    instruction faults with no debugger attached, the fault handler is
 *    the path that runs and the reporter is reached only if that
 *    handler calls it (kernel/panic.hpp says which cores do which);
 *  - take(), peek(), pending() and save() are the boot side, called
 *    from main (take() first thing at boot); take() and save() wait
 *    for the store's writes as NvRecord::clear() and NvRecord::store()
 *    do (util/nv_record.hpp).
 * Nothing here takes a critical section of its own.
 *
 * kernel/panic.hpp writes the PanicRecord into the platform's
 * reset-surviving storage - on a microcontroller that is a .noinit
 * object in SRAM, and SRAM is promised across nothing at all. It
 * survives a watchdog or software reset in practice; it does not
 * survive the plug being pulled, which is precisely the failure a
 * field unit comes back from with nothing to say.
 *
 * PersistentPanic is the reporter that closes that gap. It composes
 * with the rest of the reporter vocabulary - "save, then reset and
 * report at next boot" is two reporters chained (a reporter of the
 * program's own whose report() calls this one's and then resets), and
 * the breadcrumb was already written to SRAM before any reporter ran,
 * so nothing is lost either way.
 *
 * Why polled, and why that is not a contradiction. util/nv_writer.hpp
 * exists so that nothing ever busy-waits on nonvolatile memory. A panic
 * reporter is the one context where that rule is inverted: interrupts
 * are masked for good (panic() holds a critical section that is never
 * destroyed), the kernel will never run again, and there is nothing
 * left to be responsive to. The record is written here with bounded
 * polled waits, and being slow is the correct behaviour - the only
 * thing that matters is that the bytes are down before the reporter
 * hands over.
 *
 * Size. Eight bytes of the store (four of header, four of record). On
 * a store that takes a run (util/nv_record.hpp's NvRunStore) whose
 * write unit holds all eight - an `offset` on a unit boundary of eight
 * bytes or more - report() is one operation; on a byte store it is one
 * per differing byte. Measured on the AVR EA: one operation of 2.73 ms
 * where the byte path takes eight, 21.4 ms (docs/design/nv-journal.md,
 * "The EEPROM spelling's store: a byte, and a run where the silicon
 * writes one").
 * Endurance. Written once per panic and cleared once per boot that
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
 *   using Panic = brio::PersistentPanic<Store, 0>;   // any NvStore
 *   ...
 *   brio::panic<P, Panic>(PanicCode::assert_failed, 7);   // at the fault
 *   ...
 *   auto old = Panic::take();                             // at boot
 *
 * `offset` is where the eight bytes live in the store; nothing else may
 * claim them (nothing checks that). `version` is the record's
 * NvRecord version byte.
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
    /// panic() when this runs. The store's answer is not checked here:
    /// there is nobody left to tell.
    static void report(PanicCode code, uint8_t context) {
        (void)Record::store(PanicRecord{panic_magic,
                                        static_cast<uint8_t>(code), context});
    }

    /// Store a record that was not produced by panic() - the breadcrumb
    /// a boot found in SRAM, on its way to somewhere it will still be
    /// tomorrow (the record take_panic_record<P>() returned, say). Main
    /// context, waits for the store's writes. True when it reads back
    /// as what was asked for.
    static bool save(const PanicRecord& r) {
        (void)Record::store(r);
        const std::optional<PanicRecord> back = Record::load();
        return back && back->magic == r.magic && back->code == r.code &&
               back->context == r.context;
    }

    /// The stored record without consuming it: nothing when the store
    /// holds no valid record or one without the panic magic. Reads only.
    static std::optional<PanicRecord> peek() {
        const std::optional<PanicRecord> r = Record::load();
        if (r && r->magic != panic_magic) {
            return std::nullopt;
        }
        return r;
    }

    /// Boot-side fetch-and-clear, the twin of take_panic_record(): a
    /// stored record is returned once and the slot is invalidated with a
    /// single byte write (NvRecord::clear(), which waits for it). A
    /// second call returns nothing once the clear went through: its
    /// answer is not read here, so a store that refused the write leaves
    /// the record for the next call, and a report() torn right after the
    /// magic byte brings the consumed record back (nv_record.hpp,
    /// store()'s comment).
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
