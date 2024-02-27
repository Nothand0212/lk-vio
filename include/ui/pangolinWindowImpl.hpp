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
    void render();

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

    // @brief 设置地图
    void setMap( const std::shared_ptr<lvio::Map>& map );

    // @brief 渲染地图
    void renderMapFrameAndMapPoints();

    // @brief 渲染关键帧
    void renderFrame( const Sophus::SE3d& pose, const float* color );

  public:
    std::thread       render_thread_;
    std::atomic<bool> exit_flag_ = false;
    std::mutex        update_image_mutex_;
    std::mutex        update_euler_angle_mutex_;
    std::mutex        update_vo_state_;
    std::mutex        update_vo_cloud_;

  private:
    std::shared_ptr<lvio::Map>  map_ = nullptr;
    pangolin::OpenGlRenderState render_state_main_;

    // 图层
    int win_width_  = 1920;
    int win_height_ = 1080;

    static constexpr float cam_focus_  = 5000.0f;
    static constexpr float cam_z_near_ = 1.0f;
    static constexpr float cam_z_far_  = 1e9f;
    static constexpr int   menu_width_ = 200;

    const std::string win_name_         = "lvio";
    const std::string dis_main_name_    = "main";
    const std::string dis_3d_name_      = "Camera 3D";
    const std::string dis_3d_main_name_ = "Camera 3D Main";
    const std::string dis_plot_name_    = "Plot";
    const std::string dis_images_name_  = "Images";

    bool following_camera_ = true;  // 是否跟随相机

    float view_point_x_;
    float view_point_y_;
    float view_point_z_;
    float view_point_focus_;
    int   view_axis_direction_;

    pangolin::DataLog                  log_angle_;
    std::unique_ptr<pangolin::Plotter> plotter_angle_ = nullptr;

    // TODO 后续需要参数化
    int image_width_  = 640;
    int image_height_ = 480;

    std::string m_left_imageview_name_ = "left image view";
    std::string right_image_view_name_ = "right image view";
    cv::Mat     m_left_image;
    cv::Mat     right_image_;

    std::unique_ptr<pangolin::GlTexture> gl_texture_m_left_image = nullptr;
    std::unique_ptr<pangolin::GlTexture> gl_texture_right_image_ = nullptr;

    std::unique_ptr<ui::TrajectoryUI> no_backend_trajectory_ui_ = nullptr;
    std::unique_ptr<ui::TrajectoryUI> backend_trajectory_ui_    = nullptr;
    std::unique_ptr<ui::CloudUI>      cloud_ui_                 = nullptr;

    // yaw pitch roll
    std::tuple<float, float, float> euler_angle_ = std::make_tuple( 0, M_PI / 3., -M_PI / 3. );
  };
  //
}  // namespace ui

#endif  // LVIO_PANGOLIN_WINDOW_IMPL_HPP