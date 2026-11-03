// util/dma_owner.hpp beside the CH32V203 and CH32V303's own headers: the slot check
// compiles with this family's compiler and flags, its names meet none of the
// family's in brio's one namespace, and a claim instantiates. The slot keyed
// here, DmaSlotId<255, 255>, and its owners are a fixture's: no driver
// names them, so no driver's claim can meet them.
#include <stdint.h>

#include "ch32vx03/dma.hpp"
#include "ch32vx03/i2c.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/spi.hpp"
#include "ch32vx03/usart.hpp"
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
