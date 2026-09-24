// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// SSOE IS A HOST'S VERB: "Disable SS output in master mode" (RM 20.4.2),
// and a client's select is an INPUT it reads. A client asking for the
// hardware NSS output would drive the wire its host owns.
#include "ch32v203/spi.hpp"

void f() {
    brio::Spi<1>::configure<brio::SpiConfig{.role = brio::SpiRole::client,
                                            .nss = brio::SpiNss::hardware_output}>();
}
