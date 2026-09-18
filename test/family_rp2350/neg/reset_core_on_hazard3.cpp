// A Hazard3 hart cannot reset itself: hartreset and ndmreset are the
// Debug Module's, a debugger's and not a program's.
#include "rp2350/reset.hpp"
void f() { brio::Reset::core<brio::CoreKind::hazard3>(); }
