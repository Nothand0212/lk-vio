add_executable(offline_kitti_node offline_kitti_node.cpp)
target_link_libraries(offline_kitti_node  lk_vio  ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES} ${dbow_libs} ${g2o_libs} ${OpenCV_LIBS} logger)

add_executable(offline_kitti_ros_node offline_kitti_ros_node.cpp)
target_link_libraries(offline_kitti_ros_node  lk_vio  ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES} ${dbow_libs} ${g2o_libs} ${OpenCV_LIBS} ${catkin_LIBRARIES} logger)
