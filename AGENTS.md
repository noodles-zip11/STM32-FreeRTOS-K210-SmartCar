# Repository Guidelines

## Project Structure & Module Organization
- `Core/Inc` and `Core/Src`: application code, FreeRTOS tasks, and user drivers (e.g., `motor`, `pid`, `oled`, `mpu6050`).
- `Drivers/`: STM32 HAL/CMSIS vendor sources; treat as upstream.
- `Middlewares/`: third‑party middleware (FreeRTOS and related components).
- `cmake/`: CMake toolchain and CubeMX glue (`cmake/stm32cubemx`).
- `startup_stm32f103xb.s`, `STM32F103XX_FLASH.ld`: startup and linker scripts.
- Generated config: `freertos_car.ioc` and `.mxproject` (CubeMX).

## Build, Test, and Development Commands
This repo is a CMake + GNU Arm Embedded project (Ninja generator). Examples:
```sh
cmake --preset Debug
cmake --build --preset Debug
```
Use `Release` preset for optimized builds. The toolchain file is `cmake/gcc-arm-none-eabi.cmake`. Build output goes under `build/Debug` or `build/Release`.

There is no dedicated “run” command; flashing typically uses your STM32 tool of choice (e.g., ST‑Link, CubeProgrammer) with the built ELF/HEX.

## Coding Style & Naming Conventions
- Follow existing STM32CubeMX style: 2‑space indentation and `/* USER CODE BEGIN */` blocks.
- Put custom code only inside `USER CODE` sections in generated files to avoid regeneration loss.
- Keep module names lowercase with underscores when needed (e.g., `mpu6050.c`, `HC_SR04.c`).

## Testing Guidelines
No project‑specific test harness is present. The CMSIS DSP/NN test suites under `Drivers/` are vendor artifacts and not part of this project’s routine testing. If you add tests, document how to run them and keep them separate from generated code.

## Commit & Pull Request Guidelines
Recent commits use short, descriptive messages in Chinese (e.g., version or feature notes). Keep messages concise and scoped to the change.

For PRs, include:
- What changed and why (1–3 sentences).
- Build configuration used (`Debug`/`Release`) and any toolchain assumptions.
- Hardware notes (board, sensors) if behavior depends on wiring or calibration.

## Configuration & Safety Notes
- Regenerating with CubeMX may overwrite files; re‑check `USER CODE` sections after regeneration.
- Avoid editing vendor directories unless necessary; prefer wrapper code in `Core/`.
