#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>

#include "common/configuration.hpp"
#include "common/param_server.hpp"
#include "lk_vio/feature.hpp"
#include "lk_vio/frame.hpp"
#include "lk_vio/keyframe.hpp"
#include "lk_vio/map.hpp"
#include "lk_vio/mappoint.hpp"
// TODO: backend optimized points and pose
namespace lk_vio
{
  class Frame;
  class KeyFrame;
  class Map;
  class Feature;
  class MapPoint;

  struct PointXYZ
  {
    double x;
    double y;
    double z;
  };

  class RosUtilities
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    double timestamp_;  // s

  private:
    ros::NodeHandle nh_;

    ros::Publisher pose_pub_;

    image_transport::Publisher image_pub_;

    ros::Publisher current_points_pub_;
    ros::Publisher global_points_pub_;
    ros::Publisher key_frame_points_pub_;

    ros::Publisher current_path_pub_;
    ros::Publisher key_frame_path_pub_;

    sensor_msgs::PointCloud current_points_;
    sensor_msgs::PointCloud key_frame_points_;
    sensor_msgs::PointCloud global_points_;

    nav_msgs::Path global_path_;
    nav_msgs::Path key_frame_path_;

    std::shared_ptr<Map> map_;


  private:
    void updateGlobalPath( const geometry_msgs::PoseStamped &pose );
    void publishMap();

  public:
    // RosUtilities( const common::Configuration &config );
    RosUtilities( const common::ParamServer &config );
    ~RosUtilities() = default;
    void publishPointCloud( const std::vector<PointXYZ> &points );
    void addCurrentFrame( const std::shared_ptr<lk_vio::Frame> &frame, const std::vector<uchar> &status );

    geometry_msgs::PoseStamped extractPoseFromFrame( const std::shared_ptr<lk_vio::Frame> &frame );
    void                       publishPose( const geometry_msgs::PoseStamped &pose );

    void setMap( const std::shared_ptr<lk_vio::Map> map );
  };


}  // namespace lk_vio