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

target_sources(main_app PRIVATE src/main.c
                                src/crsf/crsf.c)


target_include_directories(main_app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src/crsf
    ${CMAKE_CURRENT_LIST_DIR}/src/control
)

target_link_libraries(main_app 
        pico_stdlib
        hardware_uart
        hardware_dma
        hardware_timer
        )

target_compile_definitions(main_app PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000)
target_compile_definitions(main_app PRIVATE
    PICO_STDIO_USB_NOT_BLOCK_ON_BUFFER_FULL=1
)

pico_enable_stdio_usb(main_app 0)
pico_enable_stdio_uart(main_app 0)

pico_add_extra_outputs(main_app)
