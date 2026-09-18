// This chip carries TWO PL022s (datasheet 12.3), so a third instance
// must not compile.
#include "rp2350/spi.hpp"
using Bad = brio::Pl022<2>;
void f() { (void)Bad::index; }
