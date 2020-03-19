cmake_minimum_required (VERSION 2.8)
project (camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-O3")

find_package (OpenCV REQUIRED)
find_package (Threads REQUIRED)

set(raspicam_DIR "/usr/lib/cmake")

find_package (raspicam REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable (camera camera.cpp camera.h main.cpp debug.cpp)

target_link_libraries(camera ${OpenCV_LIBS})
target_link_libraries(camera ${raspicam_CV_LIBS})
target_link_libraries(camera ${CMAKE_THREAD_LIBS_INIT})
