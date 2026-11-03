// Host tests for util/dma_owner.hpp: the claims one image may make. The
// proof of each case is that this file compiles - a claim the check refused
// would stop the build, not fail a test - and the test cases then run the
// model's functions so that every engine type below is instantiated, which
// is what makes its claim count. What the check refuses is in neg/, one
// translation unit a case, each run by ctest as a compile that must fail
// with the error naming the slot and both owners, or the owner and both
// slots (test/CMakeLists.txt).
//
// The engines here are models of a family's: a DMA channel bound to a
// request, with the element type a template argument as on every family.
// Run with: ctest --preset host (or ctest --preset host -R test_dma_owner)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>

#include "util/dma_owner.hpp"

namespace {

using brio::dma_claim;
using brio::DmaOwner;
using brio::DmaPeriph;
using brio::DmaSignal;
using brio::DmaSlotId;

using Usart2Tx = DmaOwner<DmaPeriph::usart, 2, DmaSignal::tx>;
using Usart2Rx = DmaOwner<DmaPeriph::usart, 2, DmaSignal::rx>;
using Spi1Tx = DmaOwner<DmaPeriph::spi, 1, DmaSignal::tx>;
using I2c1Rx = DmaOwner<DmaPeriph::i2c, 1, DmaSignal::rx>;
using Tim4Up = DmaOwner<DmaPeriph::tim, 4, DmaSignal::update>;

/// A transmit engine on DMA1's channel `ch`, moving `Elem`s for `Owner`: the
/// claim in the class body, where a family's engine makes it.
template <uint8_t ch, typename Elem, typename Owner>
struct TxEngine {
    static_assert(dma_claim<DmaSlotId<1, ch>, Owner>);
    static inline uint32_t moved = 0;
    static uint32_t start(const Elem*, uint16_t n) {
        moved = moved + n;
        return static_cast<uint32_t>(sizeof(Elem)) * n;
    }
};

/// Two transport types over one engine, as a family's buffered and direct
/// UARTs over one request: neither claims anything of its own.
template <typename Engine>
struct BufferedPort {
    static uint32_t write(const uint8_t* p, uint16_t n) { return Engine::start(p, n); }
};
template <typename Engine>
struct DirectPort {
    static uint32_t write(const uint8_t* p, uint16_t n) { return Engine::start(p, n); }
};

/// A transport that claims for itself, beside its engine's claim: the same
/// key and the same owner, so the same claim, made twice.
template <uint8_t ch, typename Owner>
struct ClaimingPort {
    static_assert(dma_claim<DmaSlotId<1, ch>, Owner>);
    using Engine = TxEngine<ch, uint8_t, Owner>;
    static uint32_t write(const uint8_t* p, uint16_t n) { return Engine::start(p, n); }
};

/// An RP family's channel: any request on any channel, the request named
/// by its owner, so the claim's second half is the datasheets' one channel
/// per DREQ.
template <uint8_t ch, typename Owner>
struct PacedChannel {
    static_assert(dma_claim<DmaSlotId<0, ch>, Owner>);
    static uint8_t channel() { return ch; }
};
using Uart0Tx = DmaOwner<DmaPeriph::uart, 0, DmaSignal::tx>;
using Uart0Rx = DmaOwner<DmaPeriph::uart, 0, DmaSignal::rx>;
using Spi0Tx = DmaOwner<DmaPeriph::spi, 0, DmaSignal::tx>;

/// A channel programmed by hand, owned under the program's own tag.
struct ScopeCopy {};

const uint8_t bytes[4] = {1, 2, 3, 4};
const uint16_t halves[2] = {0x1234, 0x5678};

} // namespace

TEST_CASE("one owner at two element widths is one claim") {
    CHECK(TxEngine<3, uint8_t, Spi1Tx>::start(bytes, 4) == 4u);
    CHECK(TxEngine<3, uint16_t, Spi1Tx>::start(halves, 2) == 4u);
}

TEST_CASE("two transport types over one request are one owner") {
    using Engine = TxEngine<7, uint8_t, Usart2Tx>;
    const uint32_t before = Engine::moved;
    CHECK(BufferedPort<Engine>::write(bytes, 4) == 4u);
    CHECK(DirectPort<Engine>::write(bytes, 2) == 2u);
    CHECK(Engine::moved - before == 6u);
}

TEST_CASE("the same claim made again, by another type, is the same claim") {
    // The port claims channel 7 for USART2's transmit, and its engine claims
    // it again: one specialization, one declaration.
    CHECK(ClaimingPort<7, Usart2Tx>::write(bytes, 1) == 1u);
    CHECK(dma_claim<DmaSlotId<1, 7>, Usart2Tx>);
    static_assert(dma_claim<DmaSlotId<1, 7>, Usart2Tx> && dma_claim<DmaSlotId<1, 7>, Usart2Tx>);
}

TEST_CASE("different slots take different owners, a timer's event and a tag among them") {
    CHECK(TxEngine<6, uint8_t, Usart2Rx>::start(bytes, 1) == 1u);
    CHECK(TxEngine<5, uint16_t, Tim4Up>::start(halves, 1) == 2u);
    CHECK(TxEngine<1, uint8_t, ScopeCopy>::start(bytes, 3) == 3u);
    // The same channel number on the other controller is another slot.
    static_assert(dma_claim<DmaSlotId<2, 7>, I2c1Rx>);
}

TEST_CASE("an owner named in an alias and never used claims nothing") {
    // I2C1's receive on channel 7 would refuse beside USART2's transmit
    // above; named and never instantiated, it is not a claim.
    using Never = TxEngine<7, uint8_t, I2c1Rx>;
    CHECK(sizeof(Never*) == sizeof(void*));
}

TEST_CASE("each request on its one channel, the same pair claimed again") {
    CHECK(PacedChannel<10, Uart0Tx>::channel() == 10u);
    CHECK(PacedChannel<11, Uart0Rx>::channel() == 11u);
    CHECK(PacedChannel<4, Spi0Tx>::channel() == 4u);
    CHECK(PacedChannel<10, Uart0Tx>::channel() == 10u);
}
