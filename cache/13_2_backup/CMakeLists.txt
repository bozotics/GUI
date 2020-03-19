cmake_minimum_required (VERSION 2.8)
project (camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-O3")

set(raspicam_DIR "/usr/lib/cmake")

find_package (raspicam REQUIRED)
find_package (OpenCV REQUIRED)
find_package (Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable (camera camera.cpp)

target_link_libraries(camera ${OpenCV_LIBS})
target_link_libraries(camera ${raspicam_CV_LIBS})
target_link_libraries(camera ${CMAKE_THREAD_LIBS_INIT})
