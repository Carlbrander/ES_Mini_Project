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
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME MINI_PROJECT_CODE)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/application.c
    Libs/kissfft/src/kfc.c
    Libs/kissfft/src/kiss_fft.c
    Libs/kissfft/src/kiss_fftnd.c
    Libs/kissfft/src/kiss_fftndr.c
    Libs/kissfft/src/kiss_fftr.c
    Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
    Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
    Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c

    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Libs/kissfft/inc
    Libs/CMSIS_DSP

    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_CURRENT_LIST_DIR}/Libs/CMSIS_DSP/libarm_cortexM4lf_math.a

    # Add user defined libraries
)

# Newlib-Nano has float printf formatting disabled by default. 
# Enable it:
target_link_options(
    ${PROJECT_NAME} PRIVATE
    -u _printf_float
)

# Enable additional warnings:
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
