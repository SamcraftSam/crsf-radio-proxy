cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
set(PICO_SDK_PATH "../pico-sdk/")
include(pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(main_app)

# pico_generate_pio_header(main_app ${CMAKE_CURRENT_LIST_DIR}/src/tx.pio)

target_sources(main_app PRIVATE src/main.c)


target_link_libraries(main_app 
        pico_stdlib
        hardware_timer
        )

pico_enable_stdio_usb(main_app 1)
pico_enable_stdio_uart(main_app 0)

pico_add_extra_outputs(main_app)
