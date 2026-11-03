// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// An owner has one slot for the whole image, the claim's second direction
// (docs/design/dma.md, "The check"). On this family each request is wired
// to one channel and the transports refuse any other, so the direction
// meets a program's own tag: one tag owns ONE channel, and a tag claiming
// two channels programmed by hand - the second claim names the tag and
// both DmaSlotId<1, 1> and DmaSlotId<1, 2> - does not compile.
#include "ch32vx03/dma.hpp"

struct Race {};
static_assert(brio::dma_claim<brio::DmaSlotId<1, 1>, Race>);
static_assert(brio::dma_claim<brio::DmaSlotId<1, 2>, Race>);
