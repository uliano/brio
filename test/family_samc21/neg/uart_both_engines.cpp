// mcu: samc21e18a samc21g18a samc21j18a
// A Uart takes ONE DMA engine at most. Erratum 1.10.4 (DS80000740S,
// E/G/J at revisions E, F and H): "When using concurrent channels
// triggers, the DMAC write-back descriptors may get corrupted", and the
// one workaround is a single channel. A duplex port's two engines are two
// channels on two triggers the two ends of the wire clock independently,
// and the corrupted live copy was measured walking received bytes into
// the SERCOM's own registers - so the pair is refused at the template
// argument, on distinct channels as here and on a shared one alike.
#include "samc21/dmac.hpp"
#include "samc21/sercom.hpp"
using namespace brio;

constexpr UartPads pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};

using Bad = Uart<0, pads, 64, 256, DmaTxEngine<6>, DmaRxEngine<7>>;

void f() { (void)Bad::tx_idle(); }
