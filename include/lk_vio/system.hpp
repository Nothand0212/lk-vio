#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "common/configuration.hpp"
#include "common/param_server.hpp"
#include "filesystem"
#include "lk_vio/backend.hpp"
#include "lk_vio/camera.hpp"
#include "lk_vio/frontend.hpp"
#include "lk_vio/imu_frame.hpp"
#include "lk_vio/loopclosing.hpp"
#include "lk_vio/orbextractor.hpp"
#include "lk_vio/ros_utilities.hpp"
// #include "ui/pangolin_window.hpp"
namespace lk_vio
{
  class System
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    System() = default;
    // explicit System( const common::Configuration &config );
    explicit System( const common::ParamServer &params );
    ~System() = default;
    // std::shared_ptr<ui::PangolinWindow> getViewUi() const { return view_ui_; };

    // void GenerateSteroCamera( const common::Configuration &config );
    // void GenerateORBextractor( const common::Configuration &config );
    void GenerateSteroCamera( const common::ParamServer &config );
    void GenerateORBextractor( const common::ParamServer &config );
    bool RunStep( const cv::Mat &left_img, const cv::Mat &right_img, const double &timestamp );
    bool RunStep( const cv::Mat &left_img, const cv::Mat &right_img, const double &timestamp, const std::vector<IMUFrame> &imu_measurements );

  private:
    // std::shared_ptr<ui::PangolinWindow> view_ui_ = nullptr;
    std::string                     sys_config_file_path_;
    std::shared_ptr<lk_vio::Camera> left_camera_   = nullptr;
    std::shared_ptr<lk_vio::Camera> right_camera_  = nullptr;
    std::shared_ptr<FrontEnd>       frontend_      = nullptr;
    std::shared_ptr<Map>            map_           = nullptr;
    std::shared_ptr<Backend>        backend_       = nullptr;
    std::shared_ptr<ORBextractor>   orb_extractor_ = nullptr, orb_init_extractor_ = nullptr;
    std::shared_ptr<LoopClosing>    loop_closing_ = nullptr;

    std::shared_ptr<RosUtilities> ros_utilities_ = nullptr;
  };
}  // namespace lk_vio
