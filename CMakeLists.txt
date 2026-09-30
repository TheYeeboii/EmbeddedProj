# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Raspberry Pi Pico board")

# Import the SDK before project() so it selects the ARM toolchain.
include(pico_sdk_import.cmake)

project(rover_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

if (NOT DEFINED WIFI_SSID)
    # set(WIFI_SSID "WEISONG 4706") weisong
    set(WIFI_SSID "CYShared")
endif()

if (NOT DEFINED WIFI_PASSWORD)
    # set(WIFI_PASSWORD "2W73)39f") weisong
    set(WIFI_PASSWORD "Doggies123!!!")
endif()

if (NOT DEFINED MQTT_BROKER_IP)
    # set(MQTT_BROKER_IP "192.168.137.1") weisong
    set(MQTT_BROKER_IP "10.134.134.164")
endif()

add_executable(rover_project
    main.c
    buddy2_motion.c
    buddy3_ir.c
)

target_include_directories(rover_project PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link SDK libraries
target_link_libraries(rover_project
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_adc
    hardware_i2c
    pico_cyw43_arch_lwip_poll
    pico_lwip_mqtt
)

# Wi-Fi and MQTT configuration
target_compile_definitions(rover_project PRIVATE
    WIFI_SSID="${WIFI_SSID}"
    WIFI_PASSWORD="${WIFI_PASSWORD}"
    MQTT_BROKER_IP="${MQTT_BROKER_IP}"
)

# Enable USB serial output (stdio over USB)
pico_enable_stdio_usb(rover_project 1)
pico_enable_stdio_uart(rover_project 0)

# Create UF2 output
pico_add_extra_outputs(rover_project)
