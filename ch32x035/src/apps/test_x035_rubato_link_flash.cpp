// test_x035_rubato_link_flash - the bench suite of the SPI and UART stacks
// of the CH32X035 under Rubato (test_x035_rubato_link, whose header comment
// is the account of its letters) built with the scheduler path in the
// flash: the same source, included whole, in an image whose build line
// below says `scheduler = flash`, so the two vectors, the thread entry, the
// request, the scheduler and every dispatch the compiler inlines into it
// run from the code flash behind its two wait states (ch32x035/schedule.hpp).
// test_x035_rubato_link itself is the default, the path in SRAM. A
// program that chose either placement runs its polled SPI loop inside the
// arbiter's dispatch from wherever that dispatch landed, so the two images
// are judged by the same verdicts and their printed figures - the rungs
// cut short, Pressure's landings, the rates - set side by side.
//
// build: boards = x035f8
// build: monitor_speed = 115200
// build: scheduler = flash

#include "test_x035_rubato_link.cpp"
