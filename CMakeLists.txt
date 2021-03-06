#Specify the minimum version of CMake(3.1 is currently recommended by Qt)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set( CMAKE_CXX_FLAGS "-O3") 
# Specify project title
project(GUI)

#set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC TRUE)
find_package(PythonLibs REQUIRED)
# Specify OpenCV folder, and take care of dependenciesand includes
#set(OpenCV_DIR "path_to_folder_of_OpenCVConfig.cmake_file")
find_package(OpenCV REQUIRED)
find_library(libssh REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

IF(APPLE)
    # Fix linking on 10.14+. See https://stackoverflow.com/questions/54068035
    include_directories(/usr/local/include)
ENDIF()

# Take care of Qt dependencies
#find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)



# add required source, header, uiand resource files
add_executable(GUI main.cpp mainwindow.cpp mainwindow.h video.cpp notepad.cpp notepad.h)

#IF(APPLE)
#    # Fix linking on 10.14+. See https://stackoverflow.com/questions/54068035
#    link_directories(/usr/local/lib)
#ENDIF()

# link required libs 
target_link_libraries(GUI Qt5::Widgets Qt5::Core Qt5::Gui -lssh ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
