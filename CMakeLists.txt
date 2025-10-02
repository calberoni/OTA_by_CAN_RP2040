cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(can_ota_pico C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(can_ota
    src/main.cpp
    src/mcp2515.cpp
    src/ota_session.cpp
    src/ota_commands.cpp
    src/ws2812.cpp
    src/flash_writer.cpp
)

pico_generate_pio_header(can_ota ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)

target_include_directories(can_ota
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(can_ota
    pico_stdlib
    hardware_spi
    hardware_pio
    hardware_watchdog
)

pico_enable_stdio_usb(can_ota 1)
pico_enable_stdio_uart(can_ota 0)

pico_add_extra_outputs(can_ota)

