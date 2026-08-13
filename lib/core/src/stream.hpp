/*
 * stream.hpp
 *
 * Compile-time stream concepts - the zero-cost replacement for the old
 * ByteStream virtual interface.
 *
 * A transport does not inherit from anything: it just provides static
 * `write_byte` / `read_byte` with try semantics (return false when the
 * byte cannot be accepted / no byte is available). Services (print.hpp,
 * proto/) are templated on the transport type and constrain it with these
 * concepts, so everything dispatches at compile time and inlines.
 *
 * Monostate transports (all-static classes such as Uart<n>) are passed
 * around as empty tag instances: `constexpr Uart<2> serial;` costs nothing
 * and lets call sites read naturally: print(serial, ...).
 */

#pragma once

#include <stdint.h>
#include <concepts>

namespace dx {

/// A sink accepts bytes: write_byte() returns false when it cannot (yet).
template <typename S>
concept ByteSink = requires(uint8_t b) {
    { S::write_byte(b) } -> std::same_as<bool>;
};

/// A source yields bytes: read_byte() returns false when none is pending.
template <typename S>
concept ByteSource = requires(uint8_t &b) {
    { S::read_byte(b) } -> std::same_as<bool>;
};

/// A full duplex transport is both.
template <typename S>
concept ByteTransport = ByteSink<S> && ByteSource<S>;

} // namespace dx
