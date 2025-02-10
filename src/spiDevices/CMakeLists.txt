add_library(spiDevices
        spiBase.cpp
        spiBase.hpp
            )


# Add the standard library to the build
target_link_libraries(spiDevices
        pico_stdlib
        hardware_spi
        )

target_include_directories(spiDevices PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")