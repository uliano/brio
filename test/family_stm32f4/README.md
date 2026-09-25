# Family compile check (STM32F4)

Smoke translation units proving the `stm32f4/` drivers compile for
EVERY device header the CMSIS pack ships - all twenty-three, from the
F401 to the F479 - instantiation only, no hardware, no `main()`.
`brio check stm32f4` compiles each `*.cpp` here for each of the
twenty-three, and each `neg/*.cpp` must FAIL for the variants its
`// mcu:` header line names. The F429, the F446 and the F411 are the
bench parts; for the twenty headers no board here carries, this sweep
is the only check there is.

What differs across the family, and what this fixture therefore
exercises: the GPIO port set (A, B, C and H everywhere, D and E from
the 64- and 100-pin bondings, F and G on the 144-pin classes, I..K on
the big packages), the serial instance set (USART1/2 everywhere, USART6
everywhere but the 36-pin F410Tx, USART3 and UART4/5 from the F405
class, UART7/8 on the F42x/F43x, F413 and F469 classes, UART9/10 on the
F413 alone) and the FULL/not split that rides on the name (the UARTs
have no synchronous mode, smartcard or flow control), the regulator (one VOS bit on the F405 class, two
elsewhere; the over-drive pair on the F42x/F43x, F446 and F469 classes
alone, and beside it the under-drive field, the
low-voltage-in-deep-sleep pair and the FISSR/FMSSR pair, each present on
its own set of headers), how many WKUPx pins are bonded (one, two or
three) and which pads they are, the PVD's thresholds in volts, the
backup SRAM behind BRE, the flash latency field's width, and THE
FREQUENCY LADDERS -
which the reserve knows for five part classes and refuses to guess for
the rest: on an F401, F410, F412 or F413 header a `Clock` above 16 MHz is
a compile error, and the positive TU proves the 16 MHz reset rate
compiles there while the ladder-dependent rates compile where the ladder
is known.

Neither `test/CMakeLists.txt` nor `stm32f4/CMakeLists.txt` sees this
directory: the script alone builds these files.
