// test_x035_rubato_flash - the bench suite of the preemptive kernel on the
// CH32X035 (test_x035_rubato, whose header comment is the account of its
// letters) built with the scheduler path in the flash: the same source of
// letters, included whole, in an image whose build line below says
// `scheduler = flash`, so the two vectors, the thread entry, the request,
// the scheduler and its queues' verbs run from the code flash behind its
// two wait states and the request is the store alone
// (ch32x035/schedule.hpp). test_x035_rubato itself is the default, the
// path in SRAM. Every bound of the suite follows the image it judges: the
// letters read the placement from ch32x035/schedule.hpp's
// `scheduler_in_ram` and draw each bound with that image's cost model
// (letter c).
//
// build: boards = x035f8
// build: monitor_speed = 115200
// build: scheduler = flash

#include "test_x035_rubato.cpp"
