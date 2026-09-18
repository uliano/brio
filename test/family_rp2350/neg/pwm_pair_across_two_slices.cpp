// REFUSED: a complementary pair across two slices. The pair is the A and
// the B output of ONE slice, which share a counter and a period (table
// 1130); GP12 is slice 6's A and GP15 is slice 7's B.
#include "rp2350/pwm.hpp"

using namespace brio;

void pair() { (void)PwmPair<12, 15, 999>::setup(); }
