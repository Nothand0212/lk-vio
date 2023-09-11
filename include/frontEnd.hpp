#ifndef LVIO_FRONT_END_HPP
#define LVIO_FRONT_END_HPP

#include <mutex>
#include <opencv2/core/core.hpp>

#include "camera.hpp"
#include "thirdparty/g2o/g2oTypes.hpp"
#include "ui/pangolin_window.hpp"
#include "utility.hpp"


namespace lvio
{
class Camera;
class Frame;
class KeyFrame;
class MapPoint;
class Map;
class ORBextractor;
// TODO back-end

enum class FrontEndStatus
{
  INITING,        // 初始化
  TRACKING_GOOD,  // 跟踪良好
  TRACKING_BAD,   // 跟踪失败
  LOST            // 丢失
};

class FrontEnd
{
public:
  FrontEnd();
  ~FrontEnd() = default;

  void setCamera( const Camera::Ptr &left_camera, const Camera::Ptr &right_camera );

  void setViewUI( const std::shared_ptr<ui::PangolinViewer> &viewerUI );

  void setORBextractor( const ORBextractor::Ptr &orb_extractor );

  void setInitORBextractor( const ORBextractor::Ptr &init_orb_extractor );

  void setMap( const Map::Ptr &map );

  // @brief 获取双目图像，并继续畸变矫正
  bool grabImageStereo( const cv::Mat &left_image, const cv::Mat &right_image, const double &time_stamp );

  // @brief 处理双目图像，提取特征点
  int detectFeatures();

  // @brief 双目初始化，构建最初的地图
  bool initStereoFrame();

  // @brief 在右图中找到对应的特征点
  int findFeaturesInRight();

  // @brief 构建初始地图
  bool buildInitMap();

  bool insertKeyFrame();

  // @brief 两帧之间的匹配track()调用了trackLastFrame()
  bool track();

  // @brief 用光流法对前后两帧的特征点进行跟踪匹配
  int trackLastFrame();

  // @brief 在丢失定位时，尝试重定位 --> 初始化时在buildInitMap里也有三角化
  int triangulateNewPoints();

  // @brief 用经过光流法匹配成功的特征点，用g2o进行优化，计算位姿
  int estimateCurrentPose();

  // TODO 后端优化
  // void setBackEnd();

  std::shared_ptr<KeyFrame> getReferenceKeyFrame()
  {
    std::unique_lock<std::mutex> lock( update_key_frame_mutex_ );
    return reference_key_frame_ptr_;
  }

  std::shared_ptr<Frame> getCurrentFrame()
  {
    return current_frame_ptr_;
  }

  std::shared_ptr<Frame> getLastFrame()
  {
    return last_frame_ptr_;
  }

private:
  std::mutex update_key_frame_map_point_mutex_;

  FrontEndStatus tracking_status_ = FrontEndStatus::INITING;

  std::shared_ptr<Camera> left_camera_  = nullptr;
  std::shared_ptr<Camera> right_camera_ = nullptr;

  std::shared_ptr<ui::PangolinWindow> viewer_ui_ = nullptr;

  std::shared_ptr<Frame>    current_frame_ptr_       = nullptr;
  std::shared_ptr<Frame>    last_frame_ptr_          = nullptr;
  std::shared_ptr<KeyFrame> reference_key_frame_ptr_ = nullptr;

  std::shared_ptr<ORBextractor> orb_extractor_ptr_      = nullptr;
  std::shared_ptr<ORBextractor> init_orb_extractor_ptr_ = nullptr;

  std::shared_ptr<MapPoint> map_point_ptr_ = nullptr;
  std::shared_ptr<Map>      map_ptr_       = nullptr;


  //TODO
  //   std::shared_ptr<BackEnd> back_end_ptr_ = nullptr;

  Sophus::SE3d relative_motion_;  // 帧i-1到帧i的相对位姿
  Sophus::SE3d relative_motion_to_reference_key_frame_;

  // 用于区分 tacking status 的参数
  int num_features_tracking_init_;
  int num_features_tracking_good_;
  int num_features_tracking_bad_;

  int min_init_map_point_;

  bool need_undistortion_         = false;
  bool show_orb_detect_result_    = false;
  bool show_lk_match_result_      = false;
  bool need_backend_optimization_ = false;
};


}  // namespace lvio
#endif  // LVIO_FRONT_END_HPP