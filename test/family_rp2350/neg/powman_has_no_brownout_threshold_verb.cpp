// The brown-out detector's threshold is a supply level and not a
// setting: this tree decodes it and writes none of it.
#include "rp2350/powman.hpp"
void f() { brio::Powman::bod_threshold(9u); }
