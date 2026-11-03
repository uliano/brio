// util/dma_owner.hpp on the AVR DA, DB and EA: the family has no DMA
// controller and no driver claims a slot, but the header is util's, and
// util compiles wherever the kernel does; so the slot check compiles with
// this family's compiler and flags beside its platform, and a claim
// instantiates. The slot keyed here, DmaSlotId<255, 255>, and its owners
// are a fixture's: no driver names them, so no driver's claim can meet them.
#include <stdint.h>

#include "avrxt/platform.hpp"
#include "util/dma_owner.hpp"

using namespace brio;

namespace {

// Fixture-only owners: a driver's request named here would meet that
// driver's own claim on its real slot (the claim binds both ways).
struct FixtureOwner {};
struct FixtureTag {};
static_assert(sizeof(DmaOwner<DmaPeriph::spi, 0, DmaSignal::tx>) == 1u);

template <typename Elem, typename Owner>
struct Bound {
    static_assert(dma_claim<DmaSlotId<255, 255>, Owner>);
    static uint32_t bytes(uint16_t n) { return static_cast<uint32_t>(sizeof(Elem)) * n; }
};

} // namespace

// One owner at two widths, and a tag on a second fixture slot.
uint32_t dma_owner_fixture(uint16_t n) {
    static_assert(dma_claim<DmaSlotId<255, 254>, FixtureTag>);
    return Bound<uint8_t, FixtureOwner>::bytes(n) + Bound<uint32_t, FixtureOwner>::bytes(n);
}
