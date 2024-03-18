#pragma once
#include "Eigen/Core"
#include "Eigen/Dense"
#include "atomic"
#include "filesystem"
// #include "glog/logging.h"
#include "common/configuration.hpp"
#include "list"
#include "lk_vio/orbextractor.hpp"
#include "lk_vio/orbvocabulary.hpp"
#include "logger/logger.h"
#include "memory"
#include "mutex"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include "sophus/se3.hpp"
#include "thread"
#include "unistd.h"
namespace lk_vio
{
  class KeyFrame;
  class Map;
  class Camera;
  class ORBextractor;
  class Backend;
  class FrontEnd;

  class LoopClosing
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<LoopClosing> Ptr;
    LoopClosing( const common::Configuration &config );
    ~LoopClosing() = default;
    void LoadParam( const common::Configuration &config );
    void GenerateORBextractor( const common::Configuration &config );

    void LoopClosingThread();
    void ProcessNewKeyframe();
    void StopLoopClosing();
    bool CheckNewKeyFrame();
    bool DetectLoop();
    int  OptimizeCurrentPose();
    bool MatchFeatures();
    bool ComputeCorrectPose();
    void LoopCorrect();

    void                 CorrectActivateKeyframeAndMappoint();
    void                 PoseGraphOptimization();
    void                 AddToKeyframeDatabase();
    void                 SetMap( std::shared_ptr<Map> map ) { map_ = map; }
    void                 SetBackend( const std::shared_ptr<Backend> backend ) { backend_ = backend; }
    void                 SetFrontend( const std::shared_ptr<FrontEnd> frontend ) { frontend_ = frontend; }
    std::vector<cv::Mat> ConvertToDescriptorVector( const cv::Mat &Descriptors );
    void                 SetSteroCamera( std::shared_ptr<Camera> left, std::shared_ptr<Camera> right );
    void                 InsertNewKeyFrame( const std::shared_ptr<KeyFrame> new_kf );

  public:
    std::atomic<bool> loop_thread_is_running_;
    std::mutex        all_new_keyframe_mutex_;
    std::thread       loop_closing_thread_;

  private:
    std::shared_ptr<Camera>                            left_camera_          = nullptr;
    std::shared_ptr<Camera>                            right_camera_         = nullptr;
    std::shared_ptr<KeyFrame>                          current_keyframe_     = nullptr;
    std::shared_ptr<KeyFrame>                          last_closed_keyframe_ = nullptr;
    std::shared_ptr<KeyFrame>                          loop_keyframe_        = nullptr;
    std::unique_ptr<ORBVocabulary>                     dbow2_vocabulary_     = nullptr;
    std::weak_ptr<Backend>                             backend_;
    std::shared_ptr<FrontEnd>                          frontend_ = nullptr;
    std::shared_ptr<Map>                               map_;
    std::list<std::shared_ptr<KeyFrame>>               all_new_keyframes_;
    std::shared_ptr<ORBextractor>                      orb_extractor_ = nullptr;
    std::map<unsigned long, std::shared_ptr<KeyFrame>> key_frame_database_;
    cv::Ptr<cv::DescriptorMatcher>                     cv_matcher_;
    std::set<std::pair<int, int>>                      set_valid_feature_matches_;

    Sophus::SE3d corrected_current_pose_;

    bool         need_correct_loop_pose_   = true;
    bool         open_loop_closing_        = true;
    bool         show_loop_closing_result_ = false;
    float        loop_threshold_heigher_;
    float        loop_threshold_lower_;
    int          pyramid_level_num_          = 8;
    unsigned int keyframe_database_min_size_ = 50;
  };

}  // namespace lk_vio
