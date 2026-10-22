/*
 * stream.hpp
 *
 * The requirements a byte transport meets so that the services above it
 * (print.hpp, serial_port.hpp, proto/) can be written once: what a sink
 * that accepts bytes and a source that yields them must provide. They
 * are C++ concepts, compiler-checked sets of requirements on a type, so
 * a transport is a type and not a virtual interface: it inherits from
 * nothing and costs nothing to pass around, and a service templated on
 * it is resolved and inlined when the program is compiled. A program
 * meets this header through the services, not directly: it writes
 * `print(serial, ...)` and names a transport of its family; a driver
 * author meets it when writing a transport.
 *
 * The smallest contract is two static functions with try semantics:
 * `write_byte(b)` returns false when the byte cannot be accepted yet,
 * `read_byte(b)` returns false when no byte is available. Neither waits.
 * Who calls which is the transport's rule, stated by every family the
 * same way and checked by nothing: the write and read verbs belong to
 * main context (setup, or an active object's dispatch), because each
 * ring behind them has one producer and one consumer and the transport's
 * interrupt handler is the other end of both.
 *
 * The run is the unit, the byte its degenerate case. A run is a
 * contiguous stretch of bytes. A transport that queues into a ring or a
 * FIFO moves a whole run for the price of one: the bytes are copied into
 * the free room it already owns, and the transmitter is nudged at most
 * twice a run (an interrupt armed, an engine pumped, a FIFO written),
 * where a byte at a time pays that nudge per byte. So a sink may also
 * offer `write_bulk(run)` (BulkSink below), and print.hpp hands it every
 * string and every formatted number whole; the source's mirror is the
 * run lent in place (SpanSource below), which is how SerialPort drains a
 * receive ring. Both are optional: a sink with the byte verb alone (a
 * test capture, a simulated port) stays a sink, and every service falls
 * back to the byte for it. The rule and its measured cost per family are
 * docs/design/serial.md, "The run is the unit, the byte its degenerate
 * case".
 *
 * Monostate transports (all-static classes such as Uart<n>, with no
 * per-instance data) are passed around as empty tag instances:
 * `constexpr Uart<2> serial;` costs nothing and lets call sites read
 * naturally: print(serial, ...).
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <span>

namespace brio {

/// A sink accepts bytes: write_byte() queues one byte and returns false,
/// writing nothing, when it cannot yet (the transmit ring is full). It
/// never waits; print.hpp's write_blocking() is the wait, built on it.
template <typename S>
concept ByteSink = requires(uint8_t b) {
    { S::write_byte(b) } -> std::same_as<bool>;
};

/// A sink that takes a run: write_bulk() queues as many of the bytes as
/// fit, oldest first, never waits, and returns how many it took, short
/// of the run's length when the room ran out and zero when there was
/// none. The caller's buffer is copied and is free again when the call
/// returns. Where the transmitter starts on a byte, it is started on the
/// run's first byte and the rest queued behind it, with at most two
/// nudges a run: a transport whose interrupt drains a ring pushes the
/// first byte and nudges as write_byte() does, then copies the rest and
/// nudges again behind it, its handler having perhaps disarmed on the
/// ring that byte emptied; one that writes an idle FIFO directly writes
/// the run's head into it and arms once behind what it queued. Where it
/// starts on a block (an engine, a USB packet) the run is queued whole
/// and the transmitter nudged once behind it. A refused run does what a
/// refused write_byte() does on the same transport: whatever keeps a
/// caller that spins on the refusal from spinning on a transmitter
/// nobody drains, and nothing more.
template <typename S>
concept BulkSink = ByteSink<S> && requires(std::span<const uint8_t> run) {
    { S::write_bulk(run) } -> std::same_as<uint32_t>;
};

/// A source yields bytes: read_byte() copies the oldest pending byte
/// into `b` and returns true, or returns false when none is pending. It
/// never waits.
template <typename S>
concept ByteSource = requires(uint8_t &b) {
    { S::read_byte(b) } -> std::same_as<bool>;
};

/// A source that lends its received bytes in place: read_span() is the
/// contiguous run ready to be read (it never wraps; the rest of a
/// wrapped ring comes on the next call) and consume(n) releases the
/// first n of it, oldest first, clamped to what is queued. These are the
/// consumer half of util/ring.hpp under the ring's own names, and the
/// ring's rules hold: consumer side only, and a run valid until the
/// consumer's next operation on the source. Where the producer is the
/// hardware (a HardwareRing behind the transport) the run is valid only
/// as long as the producer has not lapped it: consume() then answers
/// whether the run was still intact when released, and a reader that
/// acted on the run before releasing it (SerialPort's lines) acts on
/// that answer. Over a plain Ring consume() answers nothing.
template <typename S>
concept SpanSource = requires(uint32_t n) {
    { S::read_span() } -> std::convertible_to<std::span<const uint8_t>>;
    S::consume(n);
};

/// A SpanSource that reports the gaps in its stream: rx_skips() is an
/// epoch, a counter whose value means nothing and whose change means
/// that the stream the receive ring hands out jumped (bytes dropped on a
/// full ring, lost to a hardware overrun, discarded for a frame or
/// parity error, a lap the consumer missed). It is never cleared and
/// moves on those rare paths alone. A reader that carries state across
/// its runs compares it from one run to the next, because a run handed
/// out after a gap is the stream after it, which would otherwise read as
/// following on. Only its change is meaningful: its width bounds the
/// jumps between two compares (2^8 where a SkipRing counts its skips,
/// 2^32 where a vector counts too), and a reader that compares after
/// every look sees each one. Where the ring skips at the look that finds
/// the gap (util/ring.hpp's SkipRing and HardwareRing) the epoch moves
/// between the run before the gap and the run after it, never inside
/// one. A transport whose consume() answers bool (a HardwareRing behind
/// it, whose producer can lap a run while it is held) must be one of
/// these; serial_port.hpp refuses one that is not.
template <typename S>
concept SkippingSource = SpanSource<S> && requires {
    { S::rx_skips() } -> std::convertible_to<uint32_t>;
};

/// A full duplex transport is both a sink and a source.
template <typename S>
concept ByteTransport = ByteSink<S> && ByteSource<S>;

} // namespace brio
