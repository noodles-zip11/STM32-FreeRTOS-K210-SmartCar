cmake_minimum_required(VERSION 3.22)

# 这个文件是工程的“构建入口”：
# - 定义项目名、语言标准、构建类型
# - 声明本工程自己的源码
# - 引入 CubeMX 生成的底层配置（时钟/HAL/启动文件/链接脚本等）
# - 补充链接选项（newlib nano、nosys、OpenOCD FreeRTOS 调试兼容）

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
# 指定 C 标准。STM32 HAL / FreeRTOS 代码一般用 C11 足够，兼容性也好。
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
#if(NOT CMAKE_BUILD_TYPE)
#    set(CMAKE_BUILD_TYPE "Debug")
#endif()

# 当前默认使用 MinSizeRel（体积优先的发布配置）。
# 如果调试更方便，平时也可以改成 Debug，或者直接用 preset 控制。
set(CMAKE_BUILD_TYPE "MinSizeRel")

# Set the project name
set(CMAKE_PROJECT_NAME freertos_car)

# Enable compile command to ease indexing with e.g. clangd
# 生成 compile_commands.json，方便 clangd / VSCode 做跳转和补全。
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
# STM32 工程会用到启动汇编文件，所以这里同时启用 ASM。
enable_language(C ASM)

# Create an executable object type
# 这里列的是“本工程手动维护的源码/头文件”。
# CubeMX 自动生成的主要 HAL/启动文件在下面 add_subdirectory(cmake/stm32cubemx) 里引入。
add_executable(${CMAKE_PROJECT_NAME}
        Core/Inc/dmpmap.h
        Core/Inc/dmpKey.h
        Core/Inc/cJSON.h
        Core/Inc/bmp.h
        Core/Inc/HC_SR04.h
        Core/Inc/inv_mpu.h
        Core/Inc/inv_mpu_dmp_motion_driver.h
        Core/Inc/motor.h
        Core/Inc/mpu6050.h
        Core/Inc/mpuiic.h
        Core/Inc/niming.h
        Core/Inc/oled.h
        Core/Inc/oledfont.h
        Core/Inc/pid.h
        Core/Inc/settings.h
        Core/Src/cJSON.c
        Core/Src/HC_SR04.c
        Core/Src/inv_mpu.c
        Core/Src/inv_mpu_dmp_motion_driver.c
        Core/Src/motor.c
        Core/Src/mpu6050.c
        Core/Src/mpuiic.c
        Core/Src/niming.c
        Core/Src/oled.c
        Core/Src/pid.c
        Core/Src/settings.c
        Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_iwdg.h
)

# Add STM32CubeMX generated sources
# 引入 CubeMX 转换出来的 CMake 子工程（HAL、CMSIS、启动文件、链接脚本等）。
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # 这个文件用于 OpenOCD 识别 FreeRTOS 线程优先级符号，方便在线调试看任务。
    # OpenOCD FreeRTOS thread-awareness compatibility symbol.
    Core/Src/freertos_openocd_compat.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 目前大多数包含路径由 stm32cubemx 子目录目标提供，这里保留给后续自定义扩展。
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 这里可以放全局宏，例如功能开关、版本号、日志级别等。
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
# 某些工具链配置下会错误带上 ob 库（尤其混入 C++ 场景时更常见），这里主动移除。
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)


# 使用 newlib-nano，减少标准库体积（嵌入式常用）。
add_link_options(--specs=nano.specs)

# 使用 nosys，未实现的系统调用由桩函数兜底，避免链接到宿主机系统调用实现。
add_link_options(--specs=nosys.specs)
# 导出 FreeRTOS 调试需要的符号，方便 OpenOCD 线程感知功能。
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,--undefined=uxTopUsedPriority)


# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    # 主工程链接到 CubeMX 生成的底层目标（HAL/CMSIS/启动与链接配置入口）。
    stm32cubemx

    # Add user defined libraries
)
