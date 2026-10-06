/*
 * stream.hpp
 *
 * Compile-time stream concepts: a transport is a TYPE, not a virtual
 * interface, and costs nothing to pass around.
 *
 * A transport does not inherit from anything: it just provides static
 * `write_byte` / `read_byte` with try semantics (return false when the
 * byte cannot be accepted / no byte is available). Services (print.hpp,
 * proto/) are templated on the transport type and constrain it with these
 * concepts, so everything dispatches at compile time and inlines.
 *
 * THE RUN IS THE UNIT, THE BYTE ITS DEGENERATE CASE. A transport that
 * queues into a ring or a FIFO moves a whole run for the price of one:
 * the bytes copied into the free room it already owns, and the
 * transmitter nudged at most twice a run - an interrupt armed, an engine
 * pumped, a FIFO written - where a byte at a time pays that nudge per
 * byte. So a sink may also offer `write_bulk(run)` (BulkSink below), and
 * print.hpp hands it every string and every formatted number whole; the
 * source's mirror is the run lent IN PLACE (SpanSource below), which is
 * how SerialPort drains a receive ring. Both are OPTIONAL: a sink with
 * the byte verb alone - a test capture, a simulated port - stays a sink,
 * and every service falls back to the byte for it.
 *
 * Monostate transports (all-static classes such as Uart<n>) are passed
 * around as empty tag instances: `constexpr Uart<2> serial;` costs nothing
 * and lets call sites read naturally: print(serial, ...).
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <span>

namespace brio {

/// A sink accepts bytes: write_byte() returns false when it cannot (yet).
template <typename S>
concept ByteSink = requires(uint8_t b) {
    { S::write_byte(b) } -> std::same_as<bool>;
};

/// A sink that takes a RUN: write_bulk() queues as many of the bytes as
/// fit, oldest first, NEVER BLOCKS, and returns how many it took - short
/// of the run's length when the room ran out, zero when there was none.
/// Where the transmitter starts on a byte, it is started on the run's
/// FIRST byte and the rest queued behind it, with at most two nudges a
/// run: a transport whose interrupt drains a ring pushes the first byte
/// and nudges as write_byte() does, then copies the rest and nudges
/// again behind it, its handler having perhaps disarmed on the ring that
/// byte emptied; one that writes an idle FIFO directly writes the run's
/// head into it and arms once behind what it queued. Where it starts on a
/// block - an engine, a USB packet - the run is queued whole and the
/// transmitter nudged once behind it. A refused run does what a refused
/// write_byte() does on the same transport: whatever keeps a caller that
/// spins on the refusal from spinning on a transmitter nobody drains, and
/// nothing more.
template <typename S>
concept BulkSink = ByteSink<S> && requires(std::span<const uint8_t> run) {
    { S::write_bulk(run) } -> std::same_as<uint32_t>;
};

/// A source yields bytes: read_byte() returns false when none is pending.
template <typename S>
concept ByteSource = requires(uint8_t &b) {
    { S::read_byte(b) } -> std::same_as<bool>;
};

/// A source that lends its received bytes IN PLACE: read_span() is the
/// contiguous run ready to be read - it never wraps, the rest of a
/// wrapped ring coming on the next call - and consume(n) releases the
/// first n of it, oldest first, clamped to what is queued. These are the
/// consumer half of util/ring.hpp under the ring's own names, and the
/// ring's rules hold: consumer side only, a run valid until the
/// consumer's next operation on the source - and, where the producer is
/// the hardware (a HardwareRing behind the transport), valid only as
/// long as the producer has not lapped it: consume() then answers
/// whether the run was still intact when released, and a reader that
/// acted on the run before releasing it (SerialPort's lines) acts on
/// that answer. Over a plain Ring consume() answers nothing.
template <typename S>
concept SpanSource = requires(uint32_t n) {
    { S::read_span() } -> std::convertible_to<std::span<const uint8_t>>;
    S::consume(n);
};

/// A SpanSource that reports the GAPS in its stream: rx_skips() counts
/// every byte the line carried that the receive ring will not deliver -
/// dropped on a full ring, lost to a hardware overrun, discarded for a
/// frame or parity error, skipped with a lap the consumer missed - since
/// the start, NEVER CLEARED (modulo 2^32), and moved on those rare paths
/// alone. A reader that carries state across its runs compares it from
/// one run to the next, because a run handed out after a gap is the
/// stream after it, which would otherwise read as following on. Where
/// the ring knows where each gap falls (util/ring.hpp's GapRing and
/// HardwareRing) the count moves exactly between the run before the gap
/// and the run after it. A transport whose consume() answers bool (a
/// HardwareRing behind it, whose producer can lap a run while it is held)
/// must be one of these.
template <typename S>
concept SkippingSource = SpanSource<S> && requires {
    { S::rx_skips() } -> std::convertible_to<uint32_t>;
};

/// A full duplex transport is both.
template <typename S>
concept ByteTransport = ByteSink<S> && ByteSource<S>;

} // namespace brio
