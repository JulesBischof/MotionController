add_library(i2cDevices
        i2cBase.cpp
        i2cBase.hpp
            )


# Add the standard library to the build
target_link_libraries(i2cDevices
        pico_stdlib
        
        hardware_i2c
        )

target_include_directories(i2cDevices PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")