// SHA-256 family smoke TU: the accelerator's every verb instantiated
// once, and the constexpr twin in util/sha256.hpp judged against the
// FIPS 180-4 vectors AT COMPILE TIME - which is what makes the bench
// letter a comparison and not an assertion.
#include <array>
#include <span>

#include "rp2350/sha256.hpp"

using namespace brio;

// ---- the twin, checked where no silicon is needed -------------------------

constexpr std::array<uint8_t, 3> abc = {'a', 'b', 'c'};
constexpr Sha256Digest abc_digest = sha256(std::span<const uint8_t>{abc});
static_assert(abc_digest.words[0] == 0xba7816bfu);
static_assert(abc_digest.words[7] == 0xf20015adu);
static_assert(abc_digest.bytes()[0] == 0xbau);
static_assert(abc_digest.bytes()[31] == 0xadu);

// The empty message: one padding block and the standard's own answer.
static_assert(sha256(std::span<const uint8_t>{}).words[0] == 0xe3b0c442u);
static_assert(sha256(std::span<const uint8_t>{}).words[7] == 0x7852b855u);

// The padding lands on a block boundary, and only ever on one of two.
constexpr size_t tail_of(size_t leftover) {
    std::array<uint8_t, sha256_max_tail_bytes> out{};
    std::array<uint8_t, sha256_block_bytes> src{};
    return sha256_tail(std::span<const uint8_t>{src}.subspan(0, leftover), leftover, out);
}
static_assert(tail_of(0) == 64);
static_assert(tail_of(55) == 64);
static_assert(tail_of(56) == 128);
static_assert(tail_of(63) == 128);

// The geometry the driver and the twin share.
static_assert(sha256_block_bytes == 64);
static_assert(sha256_digest_bytes == 32);
static_assert(sha256_max_tail_bytes == 128);

// ---- the block ------------------------------------------------------------

static_assert(Sha256::reset_block == ResetBlock::sha256);
static_assert(Sha256::dreq == Dreq::sha256);
// 12.13.5: the two fields a store into CSR must carry through.
static_assert(Sha256::csr_config_bits ==
              (SHA256_CSR_BSWAP_BITS | SHA256_CSR_DMA_SIZE_BITS));

void sha256_block_verbs() {
    (void)Sha256::init();
    (void)Sha256::regs().CSR;

    Sha256::start();
    Sha256::byte_swap(false);
    Sha256::byte_swap(true);
    (void)Sha256::byte_swap();
    Sha256::dma_size(Sha256DmaSize::bytes);
    Sha256::dma_size(Sha256DmaSize::halfwords);
    Sha256::dma_size(Sha256DmaSize::words);
    (void)(Sha256::dma_size() == Sha256DmaSize::words);

    (void)Sha256::wdata_ready();
    (void)Sha256::sum_valid();
    (void)Sha256::write_error();
    Sha256::clear_write_error();

    (void)Sha256::write_word(0x12345678u);
    (void)Sha256::write_word(0x12345678u, 10u);
    Sha256::write_word_unchecked(0x12345678u);

    std::array<uint8_t, sha256_block_bytes> block{};
    (void)Sha256::write_block(std::span<const uint8_t>{block});
    (void)Sha256::write_block(std::span<const uint8_t>{block}, 10u);

    (void)Sha256::digest();
    (void)Sha256::digest(10u);
    (void)Sha256::hash(std::span<const uint8_t>{block});
    (void)Sha256::hash(std::span<const uint8_t>{}, 10u);
    (void)(Sha256::last_error() == Sha256Error::not_ready);
    (void)(Sha256::last_error() == Sha256Error::no_sum);
    (void)(Sha256::last_error() == Sha256Error::write_lost);
    Sha256::release();
}

// ---- the twin at run time, over the shapes the suite uses ------------------

void sha256_software_verbs() {
    std::array<uint8_t, 4096> buffer{};
    const Sha256Digest d = sha256(std::span<const uint8_t>{buffer});
    (void)d.bytes();
    (void)(d == abc_digest);

    std::array<uint32_t, sha256_digest_words> state = sha256_initial_state;
    sha256_compress(state, std::span<const uint8_t>{buffer}.subspan(0, sha256_block_bytes));
    (void)sha256_word_be(std::span<const uint8_t>{buffer}, 0);
    (void)sha256_round_constants[63];

    std::array<uint8_t, sha256_max_tail_bytes> tail{};
    (void)sha256_tail(std::span<const uint8_t>{buffer}.subspan(0, 3), 3, tail);
}
