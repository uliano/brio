// CRC calculation unit family smoke TU (RM0090 ch. 4, RM0390 ch. 4,
// RM0383 ch. 3). This block is the same on all twenty-three parts - one
// polynomial, three registers, no options - so what this TU really checks
// is the SOFTWARE MODEL beside it: that `crc32_ethernet` is the function
// the silicon computes, pinned to the published check value of
// CRC-32/MPEG-2, and that the word packing the driver offers makes a byte
// stream's checksum come out the same either way.
#include "stm32f4/crc.hpp"

using namespace brio;

// ---- the polynomial and the model -----------------------------------------------

static_assert(crc32_ethernet_poly == 0x04C11DB7u, "RM0390 4.2's generator");
static_assert(crc32_ethernet_init == 0xFFFFFFFFu, "CRC_DR's reset value, 4.4.1");

// THE PUBLISHED CHECK VALUE. Every CRC catalogue gives CRC-32/MPEG-2 -
// polynomial 0x04C11DB7, initial value 0xFFFFFFFF, no reflection, no final
// XOR - a check value of 0x0376E6E7 over the nine ASCII bytes "123456789".
// That is what pins this model to something outside this repository; the
// bench suite then pins the SILICON to the model.
constexpr uint8_t check_bytes[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
static_assert(crc32_ethernet_bytes(check_bytes, 9) == 0x0376E6E7u,
              "the model is not CRC-32/MPEG-2");

// An empty message is the initial value: nothing fed, nothing changed -
// which is what makes reset-then-read a meaningful answer.
static_assert(crc32_ethernet(nullptr, 0) == crc32_ethernet_init);
static_assert(crc32_ethernet_bytes(nullptr, 0) == crc32_ethernet_init);

// THE WORD IS FOUR BYTES, MOST SIGNIFICANT FIRST. Feeding one word must
// equal feeding its four bytes in that order - the property that lets a
// program checksum a byte stream on hardware that only takes words.
constexpr uint32_t one_word[1] = {0x12345678u};
constexpr uint8_t one_word_bytes[4] = {0x12, 0x34, 0x56, 0x78};
static_assert(crc32_ethernet(one_word, 1) == crc32_ethernet_bytes(one_word_bytes, 4));
static_assert(crc32_ethernet(one_word, 1) == 0xDF8A8A2Bu);

constexpr uint32_t two_words[2] = {word_be('1', '2', '3', '4'), word_be('5', '6', '7', '8')};
static_assert(crc32_ethernet(two_words, 2) == crc32_ethernet_bytes(check_bytes, 8),
              "eight of the check bytes, packed big-endian, are the same message");

// The running value composes: a checksum taken in two halves is the
// checksum of the whole, which is what `feed()` without a `reset()` means.
static_assert(crc32_ethernet_word(crc32_ethernet_word(crc32_ethernet_init, two_words[0]),
                                  two_words[1]) == crc32_ethernet(two_words, 2));

static_assert(word_be(0xAA, 0xBB, 0xCC, 0xDD) == 0xAABBCCDDu);
static_assert(word_be(0, 0, 0, 1) == 1u);

// One word of zeros is the classic 0xC704DD7B - the MPEG-2 residue every
// implementation of this polynomial agrees on.
constexpr uint32_t zero_word[1] = {0u};
static_assert(crc32_ethernet(zero_word, 1) == 0xC704DD7Bu);

// ---- every verb, once ------------------------------------------------------------

void crc_verbs() {
    Crc::init();
    Crc::clock(true);
    (void)Crc::clock();
    (void)Crc::regs().DR;
    Crc::reset();
    Crc::feed(0xDEADBEEFu);
    Crc::feed(two_words, 2);
    Crc::feed_bytes('a', 'b', 'c', 'd');
    (void)Crc::value();
    (void)Crc::compute(two_words, 2);
    Crc::idr(0x5Au);
    (void)Crc::idr();
    Crc::reset_block();
    Crc::clock(false);
}
