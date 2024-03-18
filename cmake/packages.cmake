list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# third party include path
include_directories(
   
    ${PROJECT_SOURCE_DIR}/thirdparty
    ${PROJECT_SOURCE_DIR}/thirdparty/sophus
    ${PROJECT_SOURCE_DIR}/thirdparty/g2o/build
    ${PROJECT_SOURCE_DIR}/thirdparty/g2o
    ${PROJECT_SOURCE_DIR}/thirdparty/DBoW2
    )


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_VERSION: " ${EIGEN3_VERSION})
message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})

message("---- ---- package.cmake ---- ----")
set(OpenCV_DIR /home/lin/Projects/opencv-4.2.0/install)
set(OpenCV_INCLUDE_DIRS ${OpenCV_DIR}/include)
set(OpenCV_LIB_DIR ${OpenCV_DIR}/lib)
file(GLOB OpenCV_LIBS "${OpenCV_LIB_DIR}/*.so")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# cholmod
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIRS})

# 设置g2o库路径
set(G2O_LIB_DIR "${PROJECT_SOURCE_DIR}/thirdparty/g2o/build/lib")
link_directories(G2O_LIB_DIR)
# 使用file命令找到所有符合条件的.so文件
file(GLOB G2O_LIBS "${G2O_LIB_DIR}/*.so")

set(g2o_libs  
${G2O_LIBS} 
${CSPARSE_LIBRARY} 
${CHOLMOD_LIBRARY}
)
message(STATUS "g2o_libs: ${g2o_libs}")
message(STATUS "CSPARSE_LIBRARY: ${CSPARSE_LIBRARY}")
message(STATUS "CHOLMOD_LIBRARY: ${CHOLMOD_LIBRARY}")

# DBoW2
set(dbow_libs ${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so)
link_directories(${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib)

# Ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  image_transport
  cv_bridge
  )
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message(STATUS "catkin_LIBRARY_DIRS: ${catkin_LIBRARY_DIRS}")
message(STATUS "catkin_LIBRARIES: ${catkin_LIBRARIES}")

