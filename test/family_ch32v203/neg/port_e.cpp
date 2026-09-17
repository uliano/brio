// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// The register map addresses ports A..E, but no part of this series
// bonds a pin of port E: the port itself is refused.
#include "ch32v203/pin.hpp"

using Ghost = brio::Port<'E'>;
void f() { Ghost::clock_on(); }
