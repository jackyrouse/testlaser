cmake_minimum_required(VERSION 3.10)
project(testlaser)

set(CMAKE_CXX_STANDARD 11)


#include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
#google_initialize_cartographer_project()
#google_enable_testing()

list(APPEND testlaser_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libydlidar_driver.so)

find_package(cartographer REQUIRED)

include_directories(include/
        cartographer_ros/
        /usr/include/eigen3/
        ydlidar
        ${Boost_INCLUDE_DIRS}
        )

file(GLOB_RECURSE ALL_SRCS
        "${PROJECT_SOURCE_DIR}/cartographer_ros/*.cc"
        "${PROJECT_SOURCE_DIR}/cartographer_ros/*.h")
#        "${PROJECT_SOURCE_DIR}/include/*.h")

MESSAGE(STATUS ${ALL_SRCS})

find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)
link_directories(${PROJECT_NAME} ${Boost_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} main.cpp ${ALL_SRCS})

# ${ALL_SRCS} include/imudata.h include/laserdata.h include/Submapdata.h)

target_link_libraries(${PROJECT_NAME} PUBLIC cartographer ${testlaser_LIBRARIES} ${Boost_LIBRARIES} pthread)

