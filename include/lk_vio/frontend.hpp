#pragma once
#include <atomic>
#include <mutex>
#include <thread>

#include "common/configuration.hpp"
#include "common/param_server.hpp"
#include "lk_vio/algorithm.hpp"
#include "lk_vio/camera.hpp"
#include "lk_vio/g2otypes.hpp"
#include "lk_vio/imu_frame.hpp"
#include "lk_vio/ros_utilities.hpp"
#include "mutex"
#include "opencv2/opencv.hpp"
// // #include "ui/pangolin_window.hpp"
namespace lk_vio
{
  class Frame;
  class ORBextractor;
  class Camera;
  class Map;
  class KeyFrame;
  class MapPoint;
  class Backend;
  class RosUtilities;

  enum class FrontendStatus
  {
    INITING,
    TRACKING_GOOD,
    TRACKING_BAD,
    LOST
  };

  class FrontEnd
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // EIGEN_DONT_ALIGN;
    // FrontEnd( const common::Configuration &config );
    FrontEnd( const common::ParamServer &config );
    ~FrontEnd() = default;
    void SetCamera( const Camera::Ptr &left, const Camera::Ptr &right );
    // void SetViewUI( const std::shared_ptr<ui::PangolinWindow> &ui );
    void SetOrbExtractor( const std::shared_ptr<lk_vio::ORBextractor> &orb );
    void SetOrbInitExtractor( const std::shared_ptr<lk_vio::ORBextractor> &orb );
    void SetBackend( const std::shared_ptr<Backend> &backend );
    void SetMap( const std::shared_ptr<Map> &map );
    void setRosUtilities( const std::shared_ptr<RosUtilities> &ros_utilities );

    int  DetectFeatures();
    bool SteroInit();
    int  FindFeaturesInRight();
    bool BuidInitMap();
    bool InsertKeyFrame();
    bool Track();
    int  TriangulateNewPoints();
    int  TrackLastFrame();
    int  EstimateCurrentPose();
    bool GrabSteroImage( const cv::Mat &left_img, const cv::Mat &right_img,
                         const double timestamp );
    void GrabIMUData( const std::vector<ImuFrame> &imu_measures );  // for imu_preintegration,

  public:
    std::vector<uchar>        lk_status_;
    std::mutex                getset_reference_kp_numtex_;
    std::shared_ptr<KeyFrame> getReferenceKF()
    {
      std::unique_lock<std::mutex> lck( getset_reference_kp_numtex_ );
      return reference_kf_;
    }

    std::shared_ptr<Frame> getCurrentFrame() { return current_frame_; }
    std::shared_ptr<Frame> getLastFrame() { return last_frame_; }

  private:
    FrontendStatus track_status_ = FrontendStatus::INITING;

    std::shared_ptr<Frame>                last_frame_         = nullptr;
    std::shared_ptr<KeyFrame>             reference_kf_       = nullptr;
    std::shared_ptr<Frame>                current_frame_      = nullptr;
    std::shared_ptr<Camera>               left_camera_        = nullptr;
    std::shared_ptr<Camera>               right_camera_       = nullptr;
    std::shared_ptr<lk_vio::ORBextractor> orb_extractor_      = nullptr;
    std::shared_ptr<lk_vio::ORBextractor> orb_extractor_init_ = nullptr;
    // std::shared_ptr<ui::PangolinWindow>   view_ui_            = nullptr;
    std::shared_ptr<Map>     map_     = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;

    std::shared_ptr<RosUtilities> ros_utilities_ = nullptr;


    /// the pose or motion variables of the current frame
    Sophus::SE3d relative_motion_;  /// T_{c_{i, i-1}}
    Sophus::SE3d relative_motion_to_reference_kf_;

    /// params for deciding the tracking status
    int num_features_tracking_good_;
    int num_features_tracking_bad_;
    int num_features_init_good_;
    int min_init_landmark_;

    bool is_need_undistortion_      = false;
    bool show_orb_detect_result_    = false;
    bool show_lk_result_            = false;
    bool open_backend_optimization_ = true;

    // related with imu preintegration
    // std::vector<IMUFrame>
  };
}  // namespace lk_vio
