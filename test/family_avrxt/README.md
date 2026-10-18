# Family compile check

Smoke translation units proving the avrxt drivers compile for EVERY
package of the AVR DA, DB and EA families - instantiation only, no
hardware, no main(). `brio check avrxt` compiles each `*.cpp` here for
avr128da/db x 28/32/48/64 and avr64ea x 28/32/48 (seconds), and each
`neg/*.cpp` must FAIL for the MCUs its `// mcu:` header line names. A
chapter the AVR EA has no driver for (NVM version 3, its ADC, the CCL)
compiles to nothing there, and its refusal by name is a negative.

Part of every driver's definition of done (CLAUDE.md "Working
discipline"): the bench chip alone masks half the family - a missing
port, instance, register or enum value only shows up on the package
that lacks it. Each driver review adds its TU and its negatives here.

`test/CMakeLists.txt` ignores this directory (it only globs `test_*/
main.cpp`, and this one is named `family`, not `test_family`); these
files are built only by the script.
