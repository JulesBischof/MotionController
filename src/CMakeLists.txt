 add_executable(MotionController
     main.cpp
)

target_include_directories(MotionController PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

 target_link_libraries(MotionController
     pico_stdlib 

     hardware_spi
     hardware_i2c
     hardware_timer

     FreeRTOS-Kernel-Heap4 
     )

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(MotionController 1)
pico_enable_stdio_usb(MotionController 1)

 pico_add_extra_outputs(MotionController)