# AGENTS.md

## Project

This repository contains STM32 HAL / STM32CubeIDE examples.

Main project to work on:
examples/stm32f103c8t6_as5600_adc_i2c_compare

## Rules

- Do not commit build outputs:
  - Debug/
  - Release/
  - *.launch
  - .settings/language.settings.xml
- Do not use `git add .`.
- Stage only intended files with `git add -p` or explicit file paths.
- Keep STM32CubeMX `.ioc` and generated code consistent.
- For CubeMX-generated files, avoid unnecessary edits outside USER CODE sections.
- Before finishing, check there are no Git conflict markers:
  grep -RInE '<<<<<<<|>>>>>>>|=======' examples/stm32f103c8t6_as5600_adc_i2c_compare/Core examples/stm32f103c8t6_as5600_adc_i2c_compare/*.ioc
- For DMA work:
  - main.c must call MX_DMA_Init exactly once.
  - MX_DMA_Init must run before MX_SPI1_Init.
  - stm32f1xx_it.c must define DMA1_Channel3_IRQHandler exactly once.
  - spi.c must not contain conflict markers.
  - touch_xpt2046 failures must be propagated instead of pretending raw value 0 is valid.
- STM32CubeIDE build and board testing are done manually by the user.
