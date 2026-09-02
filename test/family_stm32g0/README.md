# Family compile check (STM32G0)

Smoke translation units proving the `stm32g0/` drivers compile for the
three device headers the desk's boards span - instantiation only, no
hardware, no `main()`. `tools/check_stm32g0.sh` compiles each `*.cpp`
here for the G0B1 (the bench chip, the family's superset), the G071 and
the G031 (seconds), and each `neg/*.cpp` must FAIL for the variants its
`// mcu:` header line names.

What differs across these three, and what this fixture therefore
exercises: the GPIO port set (E only on the G0B1 class), the USART
instance count (2 / 4 / 6), the vector sharing (`USART2_LPUART2_IRQn`
on the G0B1 against `USART2_IRQn` elsewhere), the kernel-clock
multiplexers (USART2SEL absent on the G031) and the FLASH's SECOND BANK
- the G0B1 declares FLASH_CR.BKER/MER2, FLASH_SR.BSY2, the two bank-2
option bits and the bank-2 ECC and protection registers, while the
G071 and G031 declare none of them and do not even carry `ECC2R` as a
member of their `FLASH_TypeDef`. Every fact of that kind is read off
the device header in `brio/stm32g0/device_tables.hpp`, and this fixture
is what proves the probes right on every header.

Neither `test/CMakeLists.txt` nor `stm32g0/CMakeLists.txt` sees this
directory: the script alone builds these files.
