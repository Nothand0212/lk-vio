cmake_minimum_required(VERSION 3.10)
project(lk_vio)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS} -lpthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb ${CMAKE_CXX_FLAGS}")

set(CMAKE_INSTALL_RPATH "/home/lin/Projects/opencv-4.2.0/install/lib")


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# set(OpenCV_DIR /home/lin/Projects/opencv-4.2.0/install/share/OpenCV)
# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})
# link_directories(${OpenCV_LIB_DIR})
# message("---- LK-VIO ----")
# message(STATUS "OpenCV version: ${OpenCV_VERSION}")
# message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
# message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")


include(cmake/packages.cmake)

include_directories( ${PROJECT_SOURCE_DIR}/include)

add_subdirectory(src)
add_subdirectory(node)

