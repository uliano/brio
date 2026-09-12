# Family compile check (RP2040)

Smoke translation units proving the `rp2040/` drivers compile with the
project's own flags against the chip's one device header -
instantiation only, no hardware, no `main()`. `brio check rp2040`
compiles each `*.cpp` here, and each `neg/*.cpp` must FAIL.

One chip, one package, one header: this family has no variants for a
sweep to cross, so what the fixture proves is that every header
instantiates every verb it offers - the ones no app has called yet
included - and that what the headers refuse (a pin outside the bank,
a UART pin the function table does not give that instance, a PLL rate
no exact ratio reaches, the ring oscillator as a clock truth) is
refused at compile time.

Neither `test/CMakeLists.txt` nor `rp2040/CMakeLists.txt` sees this
directory: the script alone builds these files.
