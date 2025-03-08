target_include_directories(MotionController PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# add all .cpp files from folder and save in List
file(GLOB_RECURSE TEST_SOURCES *.cpp)

# add sourefiles to Project
target_sources(MotionController PRIVATE ${TEST_SOURCES})