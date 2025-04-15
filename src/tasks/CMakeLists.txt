# set include path for Header-Files
target_include_directories(MotionController PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# run "sub"-cmakelists.txt's - adds include paths
add_subdirectory(stm)