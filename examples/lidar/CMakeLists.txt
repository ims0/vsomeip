cmake_minimum_required (VERSION 2.8.7)
project (vSomeIPLidar)

find_package(Threads REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${VSOMEIP_INCLUDE_DIRS})


# This will get us acces to
#   VSOMEIP_INCLUDE_DIRS - include directories for vSomeIP
#   VSOMEIP_LIBRARIES    - libraries to link against
find_package(${VSOMEIP_NAME})
if (NOT ${VSOMEIP_NAME}_FOUND)
    message("${VSOMEIP_NAME} was not found. Please specify vsomeip_DIR")
endif()

add_executable (lidar_client client.cpp)
target_link_libraries(lidar_client ${VSOMEIP_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable (lidar_server server.cpp)
target_link_libraries(lidar_server ${VSOMEIP_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

