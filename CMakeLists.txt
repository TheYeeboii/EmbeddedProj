add_executable(rover_project
    main.c
)

# Link SDK libraries
target_link_libraries(rover_project 
    pico_stdlib 
    hardware_gpio 
    hardware_pwm 
    hardware_adc
    hardware_i2c
)

# Enable USB serial output (stdio over USB)
pico_enable_stdio_usb(rover_project 1)
pico_enable_stdio_uart(rover_project 0)

# Create UF2 output
pico_add_extra_outputs(rover_project)