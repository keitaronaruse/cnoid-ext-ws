cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(sources recog-red-wheel-controller.cpp)
set(target RecogRedWheelController)
choreonoid_add_simple_controller(RecogRedWheelController recog-red-wheel-controller.cpp)

target_link_libraries(${target} ${PCL_LIBRARIES} )
