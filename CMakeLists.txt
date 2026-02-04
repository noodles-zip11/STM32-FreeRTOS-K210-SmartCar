cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
#if(NOT CMAKE_BUILD_TYPE)
#    set(CMAKE_BUILD_TYPE "Debug")
#endif()

set(CMAKE_BUILD_TYPE "MinSizeRel")

# Set the project name
set(CMAKE_PROJECT_NAME freertos_car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
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
        Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_iwdg.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)


add_link_options(--specs=nano.specs)

add_link_options(--specs=nosys.specs)


# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
