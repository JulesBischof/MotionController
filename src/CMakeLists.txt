add_executable(MotionController
     main.cpp
)

target_include_directories(MotionController PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# run "sub"-cmakelists.txt's - adds include paths
add_subdirectory(tasks)
add_subdirectory(i2cDevices)
add_subdirectory(spiDevices)
add_subdirectory(miscDevices)
add_subdirectory(services)

# add all .cpp files from folder and save in List
file(GLOB_RECURSE CURRENT_SOURCES *.cpp)
file(GLOB_RECURSE I2C_SOURCES i2cDevices/*.cpp)
file(GLOB_RECURSE SPI_SOURCES spiDevices/*.cpp)
file(GLOB_RECURSE MISC_SOURCES miscDevices/*.cpp)
file(GLOB_RECURSE SERVICE_SOURCES services/*.cpp)

# add sourefiles to Project
target_sources(MotionController PRIVATE ${CURRENT_SOURCES})
target_sources(MotionController PRIVATE ${I2C_SOURCES})
target_sources(MotionController PRIVATE ${SPI_SOURCES})
target_sources(MotionController PRIVATE ${MISC_SOURCES})
target_sources(MotionController PRIVATE ${SERVICE_SOURCES})

# links libraries
target_link_libraries(MotionController PUBLIC
    pico_stdlib 

    hardware_spi
    hardware_i2c
    hardware_uart
    hardware_timer
    hardware_pio
    hardware_pwm

    FreeRTOS-Kernel-Heap4 
    )

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(MotionController 0)
pico_enable_stdio_usb(MotionController 1)

pico_add_extra_outputs(MotionController)