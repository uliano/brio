# Family compile check

Smoke translation units proving the avrdx drivers compile for EVERY
package of the AVR DA/DB family - instantiation only, no hardware, no
main(). `tools/check_family.sh` compiles each `*.cpp` here for
avr128da/db x 28/32/48/64 (seconds), and each `neg/*.cpp` must FAIL
for the MCUs its `// mcu:` header line names.

Part of every driver's definition of done (CLAUDE.md "Working
discipline"): the bench chip alone masks half the family - a missing
port, instance, register or enum value only shows up on the package
that lacks it. Each driver review adds its TU and its negatives here.

PlatformIO ignores this directory (`pio test` only discovers `test_*`
folders); these files are built only by the script.
