#ifndef LVIO_PANGOLIN_WINDOW_IMPL_HPP
#define LVIO_PANGOLIN_WINDOW_IMPL_HPP

#include <atomic>
#include <thread>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "keyFrame.hpp"
#include "map.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "pangolin/pangolin.h"
#include "ui/cloudUI.hpp"
#include "ui/trajectoryUI.hpp"
#include "unistd.h"

namespace ui
{
class PangolinWindowImpl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PangolinWindowImpl()                            = default;
  ~PangolinWindowImpl()                           = default;
  PangolinWindowImpl( const PangolinWindowImpl& ) = delete;
  PangolinWindowImpl& operator=( const PangolinWindowImpl& ) = delete;
  PangolinWindowImpl( PangolinWindowImpl&& )                 = delete;
  PangolinWindowImpl& operator=( PangolinWindowImpl&& ) = delete;

  // @brief Initialize the pangolin window.
  bool initPangolin();

  // @brief Set default image
  void setDefaultViewImage();

  // @brief render all data
  void renderAll();

  // @brief 创建图层
  void createDisplayLayout();

  // @brief 设置图像
  void setViewImage( const cv::Mat& left_image, const cv::Mat& right_image );
  // @brief 渲染图像
  void renderViewImage();

  // @brief 日志
  void renderPlotterDataLog();

  // @brief 设置角度
  void setEulerAngle( float yaw, float pitch, float roll );

  // @brief 更新轨迹
  void updateTrajectory( const Sophus::SE3d& pose );

  // @brief 更新点云
  void updatePointCloud( const Eigen::Vector3d& point );
};
//
}  // namespace ui

#endif  // LVIO_PANGOLIN_WINDOW_IMPL_HPP