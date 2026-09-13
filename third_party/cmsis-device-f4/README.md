# cmsis-device-f4 (vendored)

The `Include/` tree of STMicroelectronics' **cmsis-device-f4** at tag
**v2.6.9** (https://github.com/STMicroelectronics/cmsis-device-f4,
Apache-2.0 - see LICENSE.md): the CMSIS device headers for every
STM32F4 part (`stm32f4xx.h` is the umbrella that dispatches on the
`STM32F429xx`-style device define, `system_stm32f4xx.h` the tiny
declaration header the device headers include). Vendored so a fresh
clone builds, exactly like `third_party/cmsis-device-g0/`:
arm-none-eabi-gcc ships no device headers.

The whole family is here - twenty-three part headers, from the F401
to the F479 - because `brio check stm32f4` compiles the stratum's
smoke TUs against every one of them: the three parts on the bench
(F429, F446, F411) are where the drivers are measured, the other
twenty are where the reserve's presence-keyed derivations are proven
to hold.

NOT vendored: `Source/Templates/` (ST's startup files and
`system_stm32f4xx.c`). brio writes its own crt (`stm32f4/src/glue/`)
and never defines `SystemCoreClock`; the startup template is cited by
the crt for one thing only - the handler NAMES (`USART2_IRQHandler`
and friends), which the device header does not declare and which every
STM32 tool and user knows by that spelling.

The CMSIS-Core headers the device header includes (`core_cm4.h` and
its dependencies) come from `third_party/cmsis-core/`.
