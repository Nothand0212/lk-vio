#ifndef LVIO_PANGOLIN_WINDOW_HPP
#define LVIO_PANGOLIN_WINDOW_HPP

#include "frame.hpp"
#include "map.hpp"
#include "ui/pangolinWindowImpl.hpp"


namespace ui
{
class PangolinWindow
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PangolinWindow();
  ~PangolinWindow();

  bool init();
  bool needQuit();
  void quit();
  void viewImage( const cv::Mat &left_image, const cv::Mat &right_image );
  void plotAngleValue( float yaw, float pitch, float roll );
  void showVisualOdomResult( const Sophus::SE3d &pose );
  void addCurrentFrame( const std::shared_ptr<lvio::Frame> &frame );
  void addShowPointCloud( const Eigen::Vector3d &point );
  void setMap( const std::shared_ptr<lvio::Map> &map );

private:
  std::unique_ptr<ui::PangolinWindowImpl> pangolin_window_impl_ptr_;
};
}  // namespace ui

#endif  // LVIO_PANGOLIN_WINDOW_HPP