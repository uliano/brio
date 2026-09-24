// CRC family smoke TU: the block's three registers, every verb, and
// the packing that turns a byte stream into the words this data
// register takes - judged, at compile time, against the software twin
// in util/crc.hpp.
//
// The block is not a per-part fact: neither the reference manual nor
// the datasheet's table 2-1 makes it one, so this TU compiles for all
// nine parts and asks the part table nothing.
#include <stddef.h>

#include "ch32vx03/crc.hpp"

using namespace brio;

// ---- the register map (RM 5.3's table 5-1) ----------------------------------
static_assert(offsetof(CrcRegs, DATAR) == 0x00);
static_assert(offsetof(CrcRegs, IDATAR) == 0x04);
static_assert(offsetof(CrcRegs, CTLR) == 0x08);
static_assert(sizeof(CrcRegs) == 0x0C);
// 5.3.2 names the scratch R8_CRC_IDATAR, which is what decides the
// width of that one store.
static_assert(sizeof(CrcRegs::IDATAR) == 1u);
static_assert(crc_rst == 1u);

// ---- the packing (5.2) ------------------------------------------------------
// The first byte of the stream is the word's most significant.
static_assert(word_be(0x01, 0x02, 0x03, 0x04) == 0x01020304u);
static_assert(word_be(0xFF, 0x00, 0xFF, 0x00) == 0xFF00FF00u);

// ---- the function, in software ----------------------------------------------
// What this unit computes is CRC-32/MPEG-2, whose model is util/crc.hpp's
// and not this family's: the initial value the reset leaves, a word fed
// most significant byte first, and the two spellings agreeing.
static_assert(crc32_ethernet_init == 0xFFFFFFFFu);
static_assert(crc32_ethernet_poly == 0x04C11DB7u);

constexpr uint32_t one_word[1] = {0x01020304u};
constexpr uint8_t four_bytes[4] = {0x01, 0x02, 0x03, 0x04};
static_assert(crc32_ethernet(one_word, 1) == crc32_ethernet_bytes(four_bytes, 4));

// A run taken in pieces is the run taken whole, which is what makes
// feed() chainable and value() non-destructive.
constexpr uint32_t two_words[2] = {0xDEADBEEFu, 0x12345678u};
static_assert(crc32_ethernet(two_words, 2) ==
              crc32_ethernet_word(crc32_ethernet_word(crc32_ethernet_init, two_words[0]),
                                  two_words[1]));

// The empty message is the initial value, which is why feed(p, 0)
// leaves the running result alone.
static_assert(crc32_ethernet(one_word, 0) == crc32_ethernet_init);

// ---- every verb -------------------------------------------------------------
static constexpr uint32_t words[4] = {0x00000000u, 0xFFFFFFFFu, 0xA5A5A5A5u, 0x5A5A5A5Au};
static constexpr uint8_t bytes[8] = {0, 1, 2, 3, 4, 5, 6, 7};

uint32_t crc_verbs() {
    (void)Crc::init();
    Crc::clock(true);
    (void)Crc::clock();
    (void)Crc::reset();

    Crc::feed(0x12345678u);
    Crc::feed(words, 4);
    Crc::feed_bytes(0, 1, 2, 3);
    const uint32_t running = Crc::value();

    Crc::scratch(0x5Au);
    const uint8_t kept = Crc::scratch();

    (void)Crc::regs().DATAR;

    return running ^ Crc::compute(words, 4) ^ Crc::compute_bytes(bytes) ^ kept;
}
