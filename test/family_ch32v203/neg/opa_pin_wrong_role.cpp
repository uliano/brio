// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// AN OUTPUT PAD IN AN INPUT'S PLACE. The six pad names of an amplifier
// are one enum so that a configuration cannot be spelled with a pad in
// the wrong role - and the two pad TYPES hold the same line: an OpaIn
// is one of the four inputs, an OpaOut one of the two outputs.
#include "ch32v203/opa.hpp"

using WrongRole = brio::OpaIn<2, brio::OpaPin::out0>;
void f() { WrongRole::claim(); }
