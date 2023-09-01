#ifndef LVIO_FRAME_H
#define LVIO_FRAME_H

#include <Eigen/Core>
#include <vector>

#include "memory"
#include "mutex"
#include "opencv2/opencv.hpp"
#include "sophus/se3.hpp"

namespace lvio
{
class Feature;

class Frame
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 可以像PCL库的点云指针一样用
  typedef std::shared_ptr<Frame> Ptr;

  Frame()  = default;
  ~Frame() = default;

  Frame( const cv::Mat &left_img, const cv::Mat &right_img, const double &time_stamp );

  // @brief 设置该帧的世界位姿T_c_w
  void setPose( const Sophus::SE3d &T_c_w );

  // @brief 返回该帧的世界位姿T_c_w
  Sophus::SE3d getPose();

  // @brief 设置该帧的相对于参考关键帧的位姿T_c_k
  void setRelatitvePose( const Sophus::SE3d &T_c_k );

  // @brief 返回该帧的相对于参考关键帧的位姿T_c_k
  Sophus::SE3d getRelativePose();

  // @brief 设置该帧的id
  void setFrameId( const std::size_t &frame_id );

  // @brief 返回该帧的id
  std::size_t getFrameId();

  // @brief 设置该帧的时间戳
  void setTimeStamp( const double &time_stamp );

  // @brief 返回该帧的时间戳
  double getTimeStamp();

  // @brief 设置该帧的左图
  void setLeftImage( const cv::Mat &left_img );

  // @brief 返回该帧的左图
  cv::Mat getLeftImage();

  // @brief 设置该帧的右图
  void setRightImage( const cv::Mat &right_img );

  // @brief 返回该帧的右图
  cv::Mat getRightImage();

  // @brief 设置该帧的左图特征点
  void setLeftFeatures( const std::vector<std::shared_ptr<Feature>> &left_features );

  // @brief 返回该帧的左图特征点
  std::vector<std::shared_ptr<Feature>> getLeftFeatures();

  // @brief 设置该帧的右图特征点
  void setRightFeatures( const std::vector<std::shared_ptr<Feature>> &right_features );

  // @brief 返回该帧的右图特征点
  std::vector<std::shared_ptr<Feature>> getRightFeatures();

private:
  Sophus::SE3d T_c_w_;  // 该帧的世界位姿
  Sophus::SE3d T_c_k_;  // 该帧的相对于参考关键帧的位姿

  std::mutex updating_pose_mutex_;
  std::mutex updating_relative_pose_mutex_;

  std::size_t frame_id_;    // 该帧的id
  double      time_stamp_;  // 该帧的时间戳

  cv::Mat left_img_;   // 该帧的左图
  cv::Mat right_img_;  // 该帧的右图

  std::vector<std::shared_ptr<Feature>> left_features_;   // 该帧的左图特征点
  std::vector<std::shared_ptr<Feature>> right_features_;  // 该帧的右图特征点
};


}  // namespace lvio

#endif  // LVIO_FRAME_H