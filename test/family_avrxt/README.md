# Family compile check

Smoke translation units proving the avrxt drivers compile for EVERY
package of the AVR DA, DB and EA families - instantiation only, no
hardware, no main(). `brio check avrxt` compiles each `*.cpp` here for
avr128da/db x 28/32/48/64 and avr64ea x 28/32/48 (seconds), and each
`neg/*.cpp` must FAIL for the MCUs its `// mcu:` header line names. A
chapter with two engines (the ADC, NVMCTRL) instantiates the half its
part compiles; a block a part has not got (the TCD, the OP AMP on the
EA) compiles to nothing there and naming it is a negative; so are the
verbs a chapter's EA half declines (NVM version 3's flash and User Row
writes) and the configurations the EA's ADC refuses (an illegal pair
position, a window or a sampler on a sum wider than 16 bits).

Part of every driver's definition of done (CLAUDE.md "Working
discipline"): the bench chip alone masks half the family - a missing
port, instance, register or enum value only shows up on the package
that lacks it. Each driver review adds its TU and its negatives here.

`test/CMakeLists.txt` ignores this directory (it only globs `test_*/
main.cpp`, and this one is named `family`, not `test_family`); these
files are built only by the script.
