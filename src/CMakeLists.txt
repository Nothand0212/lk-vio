add_library(logger SHARED logger/logger.cpp)
target_link_libraries(logger)

add_library(configuration SHARED common/configuration.cpp)
add_library(param_server SHARED common/param_server.cpp)

add_library(lk_vio SHARED
        lk_vio/system.cpp
        lk_vio/frontend.cpp
        lk_vio/camera.cpp
        lk_vio/orbextractor.cpp
        lk_vio/frame.cpp
        lk_vio/feature.cpp
        lk_vio/keyframe.cpp
        lk_vio/map.cpp
        lk_vio/mappoint.cpp
        lk_vio/orbpattern.cpp
        lk_vio/backend.cpp
        lk_vio/loopclosing.cpp
        lk_vio/ros_utilities.cpp
)
message(STATUS "----G2O Libs: ${g2o_libs}")
message(STATUS "----DBoW Libs: ${dbow_libs}")
message(STATUS "----OpenCV Libs: ${OpenCV_LIBS}")
target_link_libraries(lk_vio ${g2o_libs} ${dbow_libs}  ${GLOG_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} pthread logger configuration param_server )

# test imu_preintegration build error
add_library(imu_preintegration SHARED lk_vio/imu_preintegration.cpp)
target_link_libraries(imu_preintegration  ${OpenCV_LIBS}  pthread   )