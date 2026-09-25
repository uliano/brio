/*
 * sim_spi_host.hpp
 *
 * A SPI HOST MADE OF RAM, AT THE LEVEL OF THE REQUEST. The same static
 * surface a family's `SpiHost<n>` offers to the arbiter
 * (util/bus_master.hpp) and to a device driver, with no register block
 * behind it: its seam is the REQUEST, and what it does with one is clock
 * the bytes into whatever device sits on the select line the request
 * names.
 *
 * WHY THAT IS THE RIGHT SEAM HERE, and not the register block
 * sim_pl022.hpp models. Those two headers answer different questions. A
 * simulated PL022 proves that ONE DRIVER knows no chip, so it has to be
 * the chip - the registers at their offsets, the reset values, the order
 * of the acts of a bring-up. This file proves nothing about a driver of
 * the silicon; it is the WORLD on the other side of a bus, so that a
 * device driver (a display over devices/dcs_link.hpp, a converter, a
 * touch controller) can be judged against a device made of RAM with no
 * silicon anywhere. docs/design/simulation.md places it: a device the
 * program reaches through its own driver, never a channel into the
 * kernel.
 *
 * WHAT IT MODELS. The two phases of the descriptor in one select window
 * with the D/C line flipping between them, the select active LOW, the
 * setup time COUNTED and never spent, a null `tx` clocking 0xFF and a
 * null `rx` discarding, a frame of two bytes where the request asks for
 * one, and a select line with nothing on it reading 0xFF - which is a
 * bus with no device on it and not an error, exactly as a real one would
 * be. Both completion styles of the contract are here, because the
 * arbiter distinguishes them and a test of an asynchronous transfer
 * needs the one that does not answer inside `start()`.
 *
 * WHAT IT DOES NOT MODEL: the wire and the time on it. There is no
 * clock rate, no mode, no setup or hold, no bit - a mode the device
 * disagrees with changes nothing here, and a rate is a number that
 * travels in the request and is never spent. A test that wants those
 * wants silicon. What this world is exact about is the BYTES and their
 * ORDER, which is what a device driver is made of.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include <concepts>
#include <type_traits>

#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/spi_bus.hpp"

namespace brio {

/**
 * The rate, the mode and the frame size, under names OF THIS SIM'S OWN.
 * They are deliberately not `SpiClock`/`SpiMode`/`SpiDataSize`: the
 * namespace is flat and holds one of each, and a family's compile
 * fixture may want this header beside its own `spi.hpp`.
 */
enum class SimSpiClock : uint8_t {
    div2 = 0, div4, div8, div16, div32, div64, div128, div256,
};

enum class SimSpiMode : uint8_t { mode0 = 0, mode1 = 1, mode2 = 2, mode3 = 3 };

enum class SimSpiDataSize : uint8_t { bits8 = 0, bits16 = 1 };

/// Which completion style the host gives a request that did not ask to
/// be polled. See `SimSpiHost::completion()`.
enum class SimSpiCompletion : uint8_t {
    immediate,   ///< every request completes inside start(): start() returns true
    deferred,    ///< an unpolled request is HELD: start() returns false, finish() runs it
};

/**
 * The pins of this imaginary board, and the running stamp that puts the
 * acts of a transaction in order. A pin remembers the level it was last
 * driven to and WHEN - and a byte clocked ticks the same stamp - so a
 * test can assert that the select fell before the first byte and rose
 * after the last, which is a thing no counter of transactions can say.
 */
struct SimSpiBench {
    SimSpiBench() = delete;

    static constexpr uint8_t pin_count = 16;

    static inline bool level[pin_count]{};        ///< true = driven high
    static inline uint32_t set_at[pin_count]{};
    static inline uint32_t clear_at[pin_count]{};
    static inline uint32_t sets[pin_count]{};
    static inline uint32_t clears[pin_count]{};

    /// Ticks on every recorded act: a pin driven, a byte clocked.
    static inline uint32_t stamp = 0;

    static uint32_t tick() { return ++stamp; }

    static void reset() {
        for (uint8_t i = 0; i < pin_count; ++i) {
            level[i] = false;
            set_at[i] = 0;
            clear_at[i] = 0;
            sets[i] = 0;
            clears[i] = 0;
        }
        stamp = 0;
    }
};

/**
 * A pin named at run time, as a request carries one. The engine drives
 * it; the board remembers. `clear()` ASSERTS a select, which is active
 * low on every bus this descriptor was written for.
 */
struct SimSpiPinRef {
    uint8_t pin = 0xFFu;

    constexpr bool valid() const { return pin < SimSpiBench::pin_count; }

    void set() const {
        if (valid()) {
            SimSpiBench::level[pin] = true;
            SimSpiBench::set_at[pin] = SimSpiBench::tick();
            ++SimSpiBench::sets[pin];
        }
    }
    void clear() const {
        if (valid()) {
            SimSpiBench::level[pin] = false;
            SimSpiBench::clear_at[pin] = SimSpiBench::tick();
            ++SimSpiBench::clears[pin];
        }
    }
};

/**
 * What sits on a select line: three wires as three verbs. The framing
 * adapter of a simulated panel (`SimDcsSerial`, host/sim_dcs_panel.hpp)
 * is one; a counting stub in a suite is another.
 */
template <typename M>
concept SimSpiDeviceModel = requires(M& m, bool b, uint8_t byte) {
    { m.select(b) };                            // true asserts: the wire goes low
    { m.dc(b) };                                // true = data, false = command
    { m.byte(byte) } -> std::same_as<uint8_t>;  // eight bits out, eight back
};

/// One frame's worth of the byte-level trace: what the wires carried.
struct SimSpiTraceEntry {
    uint8_t cs = 0xFFu;    ///< the select pin the request named
    bool dc = false;       ///< the D/C level this byte was clocked under
    uint8_t mosi = 0xFFu;
    uint8_t miso = 0xFFu;
};

/**
 * The host itself: a TYPE and never an object, as a family's is.
 *
 * `n` is the instance, so `SimSpiHost<0>` and `SimSpiHost<1>` are two
 * buses with two device tables and two sets of counters, the way two
 * instances of a family's host are. `trace_depth` is the byte-level
 * trace's ring, kept small because a test asserts a choreography and
 * not a transfer.
 */
template <uint8_t n, uint16_t trace_depth = 64>
class SimSpiHost {
public:
    SimSpiHost() = delete;

    using PinRef = SimSpiPinRef;

    static constexpr uint8_t number = n;
    /// What a line nobody drives reads: the pull-up, and the only thing
    /// a bus with no device on it can say.
    static constexpr uint8_t idle_level = 0xFF;
    /// How many devices one of these buses holds.
    static constexpr uint8_t device_slots = 4;

    /**
     * The transaction descriptor, field for field and spelling for
     * spelling what docs/design/spi-bus.md's "Common to all" names, plus
     * the rate and the frame size an STM32F4 or a PL022 Request carries
     * - so a driver written over one of those compiles over this one and
     * a frame-size check has something to check.
     */
    struct Request {
        PinRef cs;   ///< asserted low around the transaction
        PinRef dc;   ///< display D/C line; null = no such pin
        /// Microseconds between the CS assertion and the first clock.
        /// COUNTED here and never spent: there is no time in this world.
        uint8_t cs_setup_us = 0;
        /// Phase 1, sent with DC low; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        uint8_t cmd_len;   ///< in FRAMES
        /// Phase 2 out, null = 0xFF dummies; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; LENT until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint16_t len;   ///< phase 2 length, in FRAMES
        ReplyTo<SpiDone> reply;

        SimSpiClock clock = SimSpiClock::div16;
        SimSpiMode mode = SimSpiMode::mode0;
        SimSpiDataSize bits = SimSpiDataSize::bits8;

        /// Completion style: false = the engine answers later, true =
        /// POLLED inside start().
        bool polled = false;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- what is on the bus -------------------------------------------------

    /**
     * Put `model` on the select line `cs`. A line already taken is
     * replaced; false when the pin is not a pin or the table is full.
     * The model is held by reference and must outlive the bus.
     */
    template <SimSpiDeviceModel Model>
    static bool attach(PinRef cs, Model& model) {
        if (!cs.valid()) {
            return false;
        }
        Device* slot = find(cs.pin);
        if (slot == nullptr) {
            slot = free_slot();
        }
        if (slot == nullptr) {
            return false;
        }
        slot->pin = cs.pin;
        slot->object = static_cast<void*>(&model);
        slot->select = [](void* o, bool low) { static_cast<Model*>(o)->select(low); };
        slot->dc = [](void* o, bool high) { static_cast<Model*>(o)->dc(high); };
        slot->byte = [](void* o, uint8_t mosi) { return static_cast<Model*>(o)->byte(mosi); };
        return true;
    }

    static bool detach(PinRef cs) {
        Device* slot = cs.valid() ? find(cs.pin) : nullptr;
        if (slot == nullptr) {
            return false;
        }
        *slot = Device{};
        return true;
    }

    static void detach_all() {
        for (uint8_t i = 0; i < device_slots; ++i) {
            devices_[i] = Device{};
        }
    }

    // ---- the engine ---------------------------------------------------------

    /**
     * Run the transaction, or hold it.
     *
     * A POLLED request always completes inside this call and returns
     * true, in either completion mode. An unpolled one completes here
     * too while the mode is `immediate` - the synchronous completion the
     * contract allows, and the arbiter then sends the reply itself; in
     * `deferred` mode it is HELD instead and this returns false, which
     * is the engine saying "I run on my interrupt". The test is then the
     * app's ISR glue: `finish()` performs the transfer and hands back
     * the status a `TransferDone` would carry.
     *
     * The arbiter starts one transaction at a time, so a second held
     * request is not a case the contract allows; it replaces the first.
     */
    static bool start(const Request& r) {
        if (r.polled) {
            ++polled_;
        } else {
            ++pumped_;
        }
        if (completion_ == SimSpiCompletion::deferred && !r.polled) {
            held_ = r;
            has_held_ = true;
            return false;
        }
        perform(r);
        return true;
    }

    /// The last completion's code. Nothing here can fail - there is no
    /// wire to fail on and no engine to wedge - so it is always spi_ok.
    static uint8_t status() { return status_; }

    static bool pending() { return has_held_; }

    /// Perform the held transfer and answer the status the app's ISR
    /// glue would put in a `TransferDone`. Nothing held: the last
    /// status, unchanged.
    static uint8_t finish() {
        if (has_held_) {
            has_held_ = false;
            perform(held_);
        }
        return status_;
    }

    /// The timed arbiter's verb: drop whatever is held. Nothing is
    /// half-done here - a transfer is atomic inside `perform()` - so
    /// there is no select to raise and no interrupt to silence.
    static void recover() {
        has_held_ = false;
        status_ = spi_ok;
    }

    /// The shape of the other engines' release: the bus goes back to
    /// nothing attached and nothing counted.
    static void release() {
        detach_all();
        reset_counters();
        has_held_ = false;
        status_ = spi_ok;
    }

    // ---- the two completion styles ------------------------------------------

    static SimSpiCompletion completion() { return completion_; }
    static void completion(SimSpiCompletion c) { completion_ = c; }

    // ---- what a test reads --------------------------------------------------

    static uint32_t transactions() { return transactions_; }
    static uint32_t bytes_clocked() { return bytes_clocked_; }
    /// Transactions whose select line had no device on it.
    static uint32_t unaddressed() { return unaddressed_; }
    static uint32_t polled_requests() { return polled_; }
    static uint32_t pumped_requests() { return pumped_; }
    /// Every `cs_setup_us` the bus was asked for, added up and not spent.
    static uint32_t waited_us() { return waited_us_; }
    /// The stamp of the first and the last byte of the last transaction,
    /// on the same ruler the pins are stamped with.
    static uint32_t first_byte_at() { return first_byte_at_; }
    static uint32_t last_byte_at() { return last_byte_at_; }

    static uint16_t trace_count() {
        return trace_total_ < trace_depth ? static_cast<uint16_t>(trace_total_) : trace_depth;
    }

    /// The k-th of the frames still in the ring, oldest first.
    static SimSpiTraceEntry trace(uint16_t k) {
        if (k >= trace_count()) {
            return {};
        }
        const uint32_t oldest = trace_total_ <= trace_depth
                                    ? 0u
                                    : trace_total_ - static_cast<uint32_t>(trace_depth);
        return trace_[(oldest + k) % trace_depth];
    }

    static void reset_counters() {
        transactions_ = 0;
        bytes_clocked_ = 0;
        unaddressed_ = 0;
        polled_ = 0;
        pumped_ = 0;
        waited_us_ = 0;
        first_byte_at_ = 0;
        last_byte_at_ = 0;
        trace_total_ = 0;
    }

private:
    /// A device on a select line, type-erased: an object and the three
    /// verbs, made by a template. A run-time table of unlike types is
    /// exactly the case function pointers are for.
    struct Device {
        uint8_t pin = 0xFFu;
        void* object = nullptr;
        void (*select)(void*, bool) = nullptr;
        void (*dc)(void*, bool) = nullptr;
        uint8_t (*byte)(void*, uint8_t) = nullptr;
    };

    static Device* find(uint8_t pin) {
        for (uint8_t i = 0; i < device_slots; ++i) {
            if (devices_[i].object != nullptr && devices_[i].pin == pin) {
                return &devices_[i];
            }
        }
        return nullptr;
    }

    static Device* free_slot() {
        for (uint8_t i = 0; i < device_slots; ++i) {
            if (devices_[i].object == nullptr) {
                return &devices_[i];
            }
        }
        return nullptr;
    }

    static constexpr uint8_t frame_bytes(SimSpiDataSize bits) {
        return bits == SimSpiDataSize::bits16 ? 2u : 1u;
    }

    /// One whole tenure: the select down, the command frames under a low
    /// D/C, the data frames under a high one, the select up. The D/C is
    /// placed BEFORE the select falls, as every engine of this project
    /// does, because a device latches it with the first clock.
    static void perform(const Request& r) {
        status_ = spi_ok;
        ++transactions_;
        Device* d = r.cs.valid() ? find(r.cs.pin) : nullptr;
        if (d == nullptr) {
            ++unaddressed_;
        }
        const uint32_t total = static_cast<uint32_t>(r.cmd_len) + r.len;
        if (total == 0u) {
            return;   // a zero-length request completes with the wire untouched
        }

        first_byte_at_ = 0;
        last_byte_at_ = 0;
        const bool first_is_data = (r.cmd_len == 0u);
        drive_dc(d, r, first_is_data);
        r.cs.clear();   // active low: clear() asserts
        if (d != nullptr) {
            d->select(d->object, true);
        }
        waited_us_ += r.cs_setup_us;

        clock_phase(d, r, r.cmd.get(), nullptr, r.cmd_len, false);
        drive_dc(d, r, true);
        clock_phase(d, r, r.tx.get(), r.rx.get(), r.len, true);

        r.cs.set();
        if (d != nullptr) {
            d->select(d->object, false);
        }
    }

    /// The D/C wire is driven by the pin the request names, and by
    /// nothing else: a request with a null `dc` leaves the device's D/C
    /// where it was, because in this world there is no other hand on
    /// that wire.
    static void drive_dc(Device* d, const Request& r, bool high) {
        if (high) {
            r.dc.set();
        } else {
            r.dc.clear();
        }
        if (d != nullptr && r.dc.valid()) {
            d->dc(d->object, high);
        }
    }

    /// `frames` frames of `frame_bytes()` bytes each, the buffer's own
    /// order - so a 16-bit frame is two of its bytes and the first of
    /// them is the high one, which is what every engine here clocks.
    static void clock_phase(Device* d, const Request& r, const uint8_t* tx, uint8_t* rx,
                            uint16_t frames, bool data) {
        const uint32_t count = static_cast<uint32_t>(frames) * frame_bytes(r.bits);
        for (uint32_t i = 0; i < count; ++i) {
            const uint8_t mosi = tx != nullptr ? tx[i] : idle_level;
            const uint8_t miso = d != nullptr ? d->byte(d->object, mosi) : idle_level;
            if (rx != nullptr) {
                rx[i] = miso;
            }
            ++bytes_clocked_;
            record(r.cs.pin, data, mosi, miso);
        }
    }

    static void record(uint8_t cs, bool data, uint8_t mosi, uint8_t miso) {
        const uint32_t at = SimSpiBench::tick();
        if (first_byte_at_ == 0u) {
            first_byte_at_ = at;   // perform() zeroed it before this tenure
        }
        last_byte_at_ = at;
        trace_[trace_total_ % trace_depth] = SimSpiTraceEntry{cs, data, mosi, miso};
        ++trace_total_;
    }

    static inline Device devices_[device_slots]{};
    static inline Request held_{};
    static inline bool has_held_ = false;
    static inline SimSpiCompletion completion_ = SimSpiCompletion::immediate;
    static inline uint8_t status_ = spi_ok;

    static inline uint32_t transactions_ = 0;
    static inline uint32_t bytes_clocked_ = 0;
    static inline uint32_t unaddressed_ = 0;
    static inline uint32_t polled_ = 0;
    static inline uint32_t pumped_ = 0;
    static inline uint32_t waited_us_ = 0;
    static inline uint32_t first_byte_at_ = 0;
    static inline uint32_t last_byte_at_ = 0;

    static inline SimSpiTraceEntry trace_[trace_depth]{};
    static inline uint32_t trace_total_ = 0;
};

}   // namespace brio
