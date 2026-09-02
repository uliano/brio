# cmsis-device-g0 (vendored)

The `Include/` tree of STMicroelectronics' **cmsis-device-g0** at tag
**v1.4.5** (https://github.com/STMicroelectronics/cmsis-device-g0,
Apache-2.0 - see LICENSE.md): the CMSIS device headers for every
STM32G0 part (`stm32g0xx.h` is the umbrella that dispatches on the
`STM32G0B1xx`-style device define, `system_stm32g0xx.h` the tiny
declaration header the device headers include). Vendored so a fresh
clone builds, exactly like `third_party/samc21-dfp/`: arm-none-eabi-gcc
ships no device headers.

NOT vendored: `Source/Templates/` (ST's startup files and
`system_stm32g0xx.c`). brio writes its own crt (`stm32g0/src/glue/`)
and never defines `SystemCoreClock`; the startup template is cited by
the crt for one thing only - the handler NAMES (`USART2_LPUART2_IRQHandler`
and friends), which the device header does not declare and which every
STM32 tool and user knows by that spelling.

The CMSIS-Core headers the device header includes (`core_cm0plus.h`
and its dependencies) come from `third_party/cmsis-core/`.
