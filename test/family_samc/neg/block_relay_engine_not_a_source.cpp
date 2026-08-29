// mcu: samc21e18a samc21g18a samc21j18a
// BlockRelay's sources must satisfy BlockSource - the ping-pong shape
// with ready()/release() and the accounting. The serial engines are the
// same family but NOT that shape (DmaTxEngine drains, it never hands a
// filled block back), and naming one as a stream source must be refused
// at the spelling, not discovered when a dispatch calls a verb that is
// not there.
#include "samc/dmac.hpp"
#include "samc/platform_sam.hpp"
#include "util/block_stream.hpp"
using namespace brio;

struct NullSink {};

void f() {
    (void)BlockRelay<SamPlatform, Subscribers<NullSink>,
                     DmaTxEngine<0, uint16_t>>::source_count;
}
