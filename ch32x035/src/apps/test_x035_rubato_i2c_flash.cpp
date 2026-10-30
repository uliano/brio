// test_x035_rubato_i2c_flash - the bench suite of the CH32X035's I2C stack
// under Rubato (test_x035_rubato_i2c, whose header comment is the account
// of its letters) built with the scheduler path in the flash: the same
// source, included whole, in an image whose build line below says
// `scheduler = flash`, so the two vectors, the thread entry, the request,
// the scheduler and every dispatch the compiler inlines into it run from
// the code flash behind its two wait states (ch32x035/schedule.hpp).
// test_x035_rubato_i2c itself is the default, the path in SRAM. The
// arbiter's dispatch - start() with it, wherever the compiler put it - runs
// from wherever that placement landed, so the two images are judged by the
// same verdicts and their printed figures - the gaps between tenures,
// Pressure's landings, the post-to-reply times - set side by side.
//
// build: boards = x035f8
// build: monitor_speed = 115200
// build: scheduler = flash

#include "test_x035_rubato_i2c.cpp"
