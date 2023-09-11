cmake_minimum_required(VERSION 3.10)
project(lvio)
# add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O0 -g -ggdb ${CMAKE_CXX_FLAGS} -lpthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


include_directories(${PROJECT_SOURCE_DIR}/include)

# OpenCV 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 链接pthread库
find_package(Threads REQUIRED)

# third party include path
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/eigen)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/g2o)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty)

include_directories(/usr/include/suitesparse)
# GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_library(lvio SHARED
    src/frame.cpp
    src/feature.cpp
    src/mapPoint.cpp
    src/keyFrame.cpp
    src/map.cpp
    src/camera.cpp
    src/orbPattern.cpp
    src/orbExtractor.cpp
    src/setting.cpp
    src/frontEnd.cpp
)

# sudo apt install -y qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3
# csparse
set(CSparse_INCLUDE_DIR /usr/include/suitesparse)
# set(CSparse_DIR /usr/include)
set(CSparse_LIBRARY /usr/lib/x86_64-linux-gnu/libsuitesparse.so)
# set(CMAKE_PREFIX_PATH /usr)
# find_package(CSparse REQUIRED)
# include_directories(${CSPARSE_INCLUDE_DIR})

# # cholmod
# find_package(Cholmod REQUIRED)
# include_directories(${CHOLMOD_INCLUDE_DIRS})
set(Cholmod_INCLUDE_DIR /usr/include/suitesparse)
set(Cholmod_LIBRARY /usr/lib/x86_64-linux-gnu/libsuitesparse.so)

set(g2o_libs
        ${CSPARSE_LIBRARY} ${CHOLMOD_LIBRARY}
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_stuff.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_core.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_dense.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_csparse.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_csparse_extension.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_types_sba.so
)

set(dbow_libs ${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so)
target_link_libraries(lvio glog)

# 添加测试用例
add_executable( test_frame test/test_frame.cpp )
target_link_libraries( test_frame gtest gtest_main ${OpenCV_LIBS} Threads::Threads lvio ${dbow_libs})

add_executable(test_keyFrame test/test_keyFrame.cpp)
target_link_libraries(test_keyFrame gtest gtest_main ${OpenCV_LIBS} Threads::Threads lvio ${dbow_libs})

add_executable(test_feature test/test_feature.cpp)
target_link_libraries(test_feature gtest gtest_main ${OpenCV_LIBS} Threads::Threads lvio ${dbow_libs})

add_executable(test_mapPoint test/test_mapPoint.cpp)
target_link_libraries(test_mapPoint gtest gtest_main ${OpenCV_LIBS} Threads::Threads lvio ${dbow_libs})

add_executable(test_map test/test_map.cpp)
target_link_libraries(test_map gtest gtest_main ${OpenCV_LIBS} Threads::Threads lvio ${dbow_libs})

add_executable(test_camera test/test_camera.cpp)
target_link_libraries(test_camera gtest gtest_main ${OpenCV_LIBS} Threads::Threads lvio ${dbow_libs})

# add_executable(test_orbExtractor test/test_orbExtractor.cpp)
# target_link_libraries(test_orbExtractor gtest gtest_main ${OpenCV_LIBS} Threads::Threads lvio ${dbow_libs})

add_executable(test_setting test/test_setting.cpp)
target_link_libraries(test_setting gtest gtest_main ${OpenCV_LIBS} Threads::Threads lvio  ${dbow_libs})

# 添加测试
add_test( NAME test_frame COMMAND test_frame )