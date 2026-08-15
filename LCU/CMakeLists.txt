# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(LCU C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(LCU LCU.cpp ../lib/rov/mcp2515.cpp )

pico_set_program_name(LCU "LCU")
pico_set_program_version(LCU "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(LCU 0)
pico_enable_stdio_usb(LCU 1)

# Add the standard library to the build
target_link_libraries(LCU
        pico_stdlib
        hardware_spi)

# Distinct USB product string so tools can tell the two boards apart when
# both are plugged in (they share the same VID/PID otherwise).
target_compile_definitions(LCU PRIVATE
        USBD_PRODUCT="ROV LCU"
)

# Add the standard include files to the build
target_include_directories(LCU PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../lib
)

pico_add_extra_outputs(LCU)

