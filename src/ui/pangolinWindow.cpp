#include "ui/pangolinWindowImpl.hpp"


namespace ui
{
PangolinWindow::PangolinWindow()
{
  pangolin_window_impl_ptr_ = std::unique_ptr<PangolinWindowImpl>( new PangolinWindowImpl() );
}

bool PangolinWindow::init()
{
  bool is_init = pangolin_window_impl_ptr_->init();
  if ( is_init )
  {
    pangolin_window_impl_ptr_->render_thread_ = std::thread( [ this ]() { pangolin_window_impl_ptr_->render(); } );
  }
  return is_init;
}

bool PangolinWindow::needQuit()
{
  return pangolin::ShouldQuit();
}


void PangolinWindow::~PangolinWindow()
{
  quit();
}

void PangolinWindow::quit()
{
  if ( pangolin_window_impl_ptr_->render_thread_.joinable() )
  {
    pangolin_window_impl_ptr_->exit_flag_.store( true );
    pangolin_window_impl_ptr_->render_thread_.join();
  }
}

void PangolinWindow::viewImage( const cv::Mat &left_image, const cv::Mat &right_image )
{
  pangolin_window_impl_ptr_->setViewImage( left_image, right_image );
}

void PangolinWindow::addCurrentFrame( const std::shared_ptr<lvio::Frame> &frame )
{
  Sophus::SE3d current_pose = frame->getPose().inverse();
  showVisualOdomResult( current_pose );
  viewImage( frame->left_image_, frame->right_image_ );
  plotAngleValue( current_pose.angleY(), current_pose.angleX(), current_pose.angleZ() );
}

void PangolinWindow::addShowPointCloud( const Eigen::Vector3d &point )
{
  pangolin_window_impl_ptr_->updatePointCloud( point );
}
}  // namespace ui