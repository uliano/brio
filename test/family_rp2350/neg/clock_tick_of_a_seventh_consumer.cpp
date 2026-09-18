// The TICKS block has six generators and no seventh: a consumer past
// the RISC-V platform timer's would read another block's registers.
#include "rp2350/clock.hpp"
using Bad = brio::TickGenerator<static_cast<brio::TickConsumer>(6)>;
void f() { (void)Bad::start(12u); }
