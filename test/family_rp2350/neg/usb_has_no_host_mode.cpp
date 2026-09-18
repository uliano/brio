// REFUSED: putting the controller into host mode. The block does both
// roles (12.7.3.9) and brio has no host side by decision
// (docs/design/usb.md), so MAIN_CTRL.HOST_NDEVICE has no verb here and
// none of the host-only registers - SOF_WR, INT_EP_CTRL, NAK_POLL, the
// fifteen ADDR_ENDP of the polled interrupt endpoints - is reachable.
#include "rp2350/usb.hpp"

using namespace brio;

void become_a_host() { Usb::host_mode(true); }
