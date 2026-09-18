# Family compile check (RP2350)

Smoke translation units proving the `rp2350/` drivers compile with the
project's own flags - instantiation only, no hardware, no `main()`.
`brio check rp2350` compiles each `*.cpp` here and requires each
`neg/*.cpp` to FAIL.

WHAT IS SWEPT HERE IS NOT A LIST OF PARTS but the two axes this chip
has: THE ARCHITECTURE (a Cortex-M33 pair and a Hazard3 RISC-V pair over
one set of peripherals, so every TU is compiled by BOTH compilers) and
THE PACKAGE (a QFN-60 with 30 bonded GPIO and four ADC inputs, a QFN-80
with 48 and eight, so every TU is compiled for both). Four builds of
every file, and what they prove together is that no header of the
stratum knows an instruction set and none states a pin count twice.

A negative whose name begins with `qfn60_` is a refusal the SMALLER
package makes - a pad the QFN-60 has not got, which the QFN-80 brings
out perfectly well - so the script builds it for that package. Every
other negative is built for the QFN-80, and must be refused by both
compilers.

Neither `test/CMakeLists.txt` nor `rp2350/CMakeLists.txt` sees this
directory: the script alone builds these files.
