if (NOT DEFINED WIFI_SSID)
    set(WIFI_SSID "WEISONG 4706")
endif()

if (NOT DEFINED WIFI_PASSWORD)
    set(WIFI_PASSWORD "2W73)39f")
endif()

if (NOT DEFINED MQTT_BROKER_IP)
    set(MQTT_BROKER_IP "192.168.137.1")
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