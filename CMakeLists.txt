# Set the minimum required version of CMake for this project
cmake_minimum_required(VERSION 3.12)

# Include Pico SDK initialization function from the environment path
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Define the project name and languages (C, C++, and ASM)
project(main C CXX ASM)

# Set the C and C++ standards to use for the project
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK and make its libraries available
pico_sdk_init()

# Add the main application source file and create an executable
add_executable(${PROJECT_NAME}
    main.c
)

# Generate .map, .bin, .hex, and .uf2 files for flashing
pico_add_extra_outputs(${PROJECT_NAME})

# Link required libraries:
# - pico_stdlib: provides GPIO, sleep/delay, printf, etc.
# - hardware_spi: provides access to the SPI peripheral
target_link_libraries(
    ${PROJECT_NAME}
    pico_stdlib
    hardware_spi
)

# Enable USB as the printf output backend
pico_enable_stdio_usb(${PROJECT_NAME} 1)

# Disable UART for stdio (optional; avoids conflict if UART is unused)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
