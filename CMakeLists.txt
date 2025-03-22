# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(MotionController C CXX ASM)

# ================================== FreeRTOS ==========================================
# Set any variables required for importing libraries
SET(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)

# Import those libraries
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)
# ======================================================================================

# enable exceptions
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add subdirectories
add_subdirectory(src)
add_subdirectory(config)
add_subdirectory(tests)

# ====== prain Uart =======

# add prain_uart header files 
target_include_directories(MotionController PUBLIC lib/prain_uart/include)

# add prain_uart source files manually
file(GLOB_RECURSE PRAIN_UART_SOURCES lib/prain_uart/src/*.cpp)
target_sources(MotionController PRIVATE ${PRAIN_UART_SOURCES})

# =========================

# file to the "lib" directory
# add_subdirectory(lib)

pico_set_program_name(MotionController "MotionController")
pico_set_program_version(MotionController "0.1")


# Add the standard include files to the build
target_include_directories(MotionController PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)