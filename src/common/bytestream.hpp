#pragma once

#include <stdint.h>

// Ported from uliano/AVR-Multislope (lib/core/src/bytestream.hpp).
/**
 * @brief Minimal byte-oriented stream interface.
 *
 * This abstraction is intentionally transport-agnostic: protocols can depend
 * on ByteStream instead of depending directly on UART registers/ISRs.
 */
class ByteStream {
public:
    virtual bool write_byte(uint8_t byte) = 0;
    virtual bool read_byte(uint8_t &byte) = 0;

    // Default bulk-write implementation: can be overridden by transports.
    virtual uint8_t write(const uint8_t *buffer, uint8_t len) {
        uint8_t written = 0;
        while (written < len) {
            if (!write_byte(buffer[written])) {
                break;
            }
            ++written;
        }
        return written;
    }

protected:
    ByteStream() = default;
    ~ByteStream() = default;
};
