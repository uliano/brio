// TRNG_CONFIG.RND_SRC_SEL picks one of four inverter chains, not five.
#include "rp2350/trng.hpp"
void f() { (void)brio::Trng::init<brio::TrngConfig{.chain = 4}>(); }
