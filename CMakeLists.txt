# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build function from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project in PROJECT_NAME variable
project(main C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# Creates a pico-sdk subdir in our proj for libs
pico_sdk_init()

# Link Eigen include directory
include_directories(external/eigen)

add_executable(${PROJECT_NAME}
    main.cpp
)

# Create map, bin, extra, uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(
    ${PROJECT_NAME}
    pico_stdlib
    hardware_spi
)

# Enable exception handling for C++
target_compile_options(${PROJECT_NAME} PRIVATE -fexceptions)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR} )

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

