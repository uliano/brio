# Family compile check (STM32G0)

Smoke translation units proving the `stm32g0/` drivers compile for
EVERY device header the CMSIS pack ships - all twelve, the x1 line
(G031/G041, G051/G061, G071/G081, G0B1/G0C1) and the x0 value line
(G030, G050, G070, G0B0) - instantiation only, no hardware, no
`main()`. `tools/check_stm32g0.sh` compiles each `*.cpp` here for each
of the twelve, and each `neg/*.cpp` must FAIL for the variants its
`// mcu:` header line names. The G0B1 is the bench chip and the
family's superset; the G071 and G031 are the desk's other Nucleos; for
the nine headers no board here carries, this sweep is the only check
there is.

What differs across the family, and what this fixture therefore
exercises: the GPIO port set (E only on the G0B1 class), the USART
instance count (2 / 4 / 6) and the FULL/BASIC split that rides on it,
the LPUARTs (one on every x1, two on the G0B1 class, NONE on the value
line), the LPTIMs, DAC, comparators, VREFBUF, TIM2, PVD/PVM, the
programmable BOR and the PCROP/securable-memory option bits (all absent
from the value line), the TIMER SET (TIM4 on the G0B1 class alone,
neither basic timer nor TIM15 on the G031 class), the DMA geometry
(five / seven channels, DMA2 on the G0Bx/G0C1) and the FLASH's SECOND
BANK - the G0B1 declares FLASH_CR.BKER/MER2, FLASH_SR.BSY2, the two
bank-2 option bits and the bank-2 ECC and protection registers, while
the smaller parts declare none of them and do not even carry `ECC2R` as
a member of their `FLASH_TypeDef`.

THE VECTORS ARE THE POINT. An interrupt line's enumerator NAME spells
what shares it (`USART2_LPUART2_IRQn`, `TIM6_DAC_LPTIM1_IRQn`,
`ADC1_COMP_IRQn`, `DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn`...), and a
header declares the shared spelling exactly when it declares the
sharer. IRQn values are enumerators the preprocessor cannot probe, so
`brio/stm32g0/device_tables.hpp` derives each vector from the SHARER'S
PRESENCE (LPUART2_BASE, TIM4_BASE, DAC1_BASE, COMP1_BASE, DMA2_BASE,
FDCAN1_BASE...) and never from a device name - and these fixtures are
HEADER-AGNOSTIC in the same way: each static_assert asks the header
whether the sharer exists and demands the enumerator that follows, so
the same file proves the derivation on all twelve headers. A wrong line
would be a silent Default_Handler spin; a header whose naming did not
follow the rule fails to compile instead.

Neither `test/CMakeLists.txt` nor `stm32g0/CMakeLists.txt` sees this
directory: the script alone builds these files.
