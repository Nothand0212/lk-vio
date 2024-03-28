#include "common/utilities.hpp"
#include "lk_vio/backend.hpp"
#include "lk_vio/feature.hpp"
#include "lk_vio/frame.hpp"
#include "lk_vio/frontend.hpp"
#include "lk_vio/keyframe.hpp"
#include "lk_vio/map.hpp"
#include "lk_vio/mappoint.hpp"
#include "lk_vio/orbextractor.hpp"
#include "logger/logger.h"
namespace lk_vio
{
// FrontEnd::FrontEnd( const common::Configuration &config )
// {
//   num_features_init_good_     = config.tracking_status_params.init_good;
//   num_features_tracking_good_ = config.tracking_status_params.track_good;
//   num_features_tracking_bad_  = config.tracking_status_params.track_bad;
//   is_need_undistortion_       = config.camera_params.need_undistortion;
//   min_init_landmark_          = config.map_params.init_landmark_size;
//   open_backend_optimization_  = config.back_end_params.activate;
//   show_orb_detect_result_     = config.viewer_params.show_extractor_result_cv;
//   show_lk_result_             = config.viewer_params.show_lk_matcher_result_cv;

//   INFO( lk_vio::logger, "---- ---- FrontEnd Parameters ---- ----" );
//   INFO( lk_vio::logger, "Init Good Features: {0}", num_features_init_good_ );
//   INFO( lk_vio::logger, "Track Good Features: {0}", num_features_tracking_good_ );
//   INFO( lk_vio::logger, "Track Bad Features: {0}", num_features_tracking_bad_ );
//   INFO( lk_vio::logger, "Need Undistortion: {0}", is_need_undistortion_ );
//   INFO( lk_vio::logger, "Init Landmark Size: {0}", min_init_landmark_ );
//   INFO( lk_vio::logger, "Open Backend: {0}", open_backend_optimization_ );
//   INFO( lk_vio::logger, "Show ORB Detect Result: {0}", show_orb_detect_result_ );
//   INFO( lk_vio::logger, "Show LK Result: {0}", show_lk_result_ );
// }

FrontEnd::FrontEnd( const common::ParamServer &config )
{
  num_features_init_good_     = config.tracking_status_params.init_good;
  num_features_tracking_good_ = config.tracking_status_params.track_good;
  num_features_tracking_bad_  = config.tracking_status_params.track_bad;
  is_need_undistortion_       = config.camera_params.need_undistortion;
  min_init_landmark_          = config.map_params.init_landmark_size;
  open_backend_optimization_  = config.back_end_params.activate;
  show_orb_detect_result_     = config.viewer_params.show_extractor_result_cv;
  show_lk_result_             = config.viewer_params.show_lk_matcher_result_cv;

  INFO( lk_vio::logger, "---- ---- FrontEnd Parameters ---- ----" );
  INFO( lk_vio::logger, "Init Good Features: {0}", num_features_init_good_ );
  INFO( lk_vio::logger, "Track Good Features: {0}", num_features_tracking_good_ );
  INFO( lk_vio::logger, "Track Bad Features: {0}", num_features_tracking_bad_ );
  INFO( lk_vio::logger, "Need Undistortion: {0}", is_need_undistortion_ );
  INFO( lk_vio::logger, "Init Landmark Size: {0}", min_init_landmark_ );
  INFO( lk_vio::logger, "Open Backend: {0}", open_backend_optimization_ );
  INFO( lk_vio::logger, "Show ORB Detect Result: {0}", show_orb_detect_result_ );
  INFO( lk_vio::logger, "Show LK Result: {0}", show_lk_result_ );
}


void FrontEnd::SetCamera( const Camera::Ptr &left, const Camera::Ptr &right )
{
  assert( left != nullptr && right != nullptr );
  left_camera_  = left;
  right_camera_ = right;
}

bool FrontEnd::GrabSteroImage( const cv::Mat &left_img, const cv::Mat &right_img,
                               const double timestamp )
{
  current_frame_.reset( new Frame( left_img, right_img, timestamp ) );
  /// undistort the images, which is not required in KITTI
  if ( is_need_undistortion_ )
  {
    left_camera_->UndistortImage( current_frame_->left_image_, current_frame_->left_image_ );
    right_camera_->UndistortImage( current_frame_->right_image_, current_frame_->right_image_ );
  }

  {
    std::unique_lock<std::mutex> lck( map_->mmutex_map_update_ );
    switch ( track_status_ )
    {
      case FrontendStatus::INITING:
      {
        SteroInit();
        break;
      }
      case FrontendStatus::TRACKING_BAD:
      case FrontendStatus::TRACKING_GOOD:
      {
        Track();
        break;
      }
      case FrontendStatus::LOST:
      {
        /// TODO: search for keyframe from KeyFrameDatabase
        break;
      }
    }
  }


  if ( ros_utilities_ )
  {
    ros_utilities_->addCurrentFrame( current_frame_, this->lk_status_ );
  }

  last_frame_ = current_frame_;
  return true;
}

void FrontEnd::GrabIMUData( const std::vector<IMUFrame> &imu_measures )
{
  std::lock_guard<std::mutex> lck( imu_measures_deque_mutex_ );
  for ( const auto &imu_measure : imu_measures )
  {
    this->imu_measures_deque_.push_back( imu_measure );
  }
}

void FrontEnd::GrabIMUData( const IMUFrame &imu_measure )
{
  std::lock_guard<std::mutex> lck( imu_measures_deque_mutex_ );

  this->imu_measures_deque_.push_back( imu_measure );
}

void FrontEnd::PrepareIMUData()
{
  // TODO: implement preintegration
  // 1. When the first StereoFrame(f_1) is grabbed, just return
  // 2. When the second StereoFrame is grabbed, preintegrate the IMU data and store it in the current_frame_(f_2)
  //    How? The first StereoFrame's Pose is used as the initial pose for the preintegration.
  //    The IMU data between f_1 and f_2 is used to preintegrate the pose of f_2.


  // step 1:
  if ( this->current_frame_->getLastFramePtr() == nullptr )
  {
    // todo
    INFO( lk_vio::logger, "No previous frame available for preintegration" );
    this->current_frame_->setIntegratedFlag();
    return;
  }

  if ( this->imu_measures_deque_.size() == 0 )
  {
    ERROR( lk_vio::logger, "No IMU data available for preintegration" );
    return;
  }

// step 2: did I need to consider the IMU data's timestamp? or suppose that the IMU data is between f_1 and f_2?
#define TIME_DIFF 0.01
  bool is_enough{ false };

  while ( true )
  {
    {
      std::lock_guard<std::mutex> lock( imu_measures_deque_mutex_ );
      if ( !this->imu_measures_deque_.empty() )
      {
        auto &imu_frame = this->imu_measures_deque_.front();
        if ( imu_frame.t < this->current_frame_->getLastFramePtr()->timestamp_ - TIME_DIFF )
        {
          this->imu_measures_deque_.pop_front();
        }
        else if ( imu_frame.t > this->current_frame_->getLastFramePtr()->timestamp_ - TIME_DIFF )
        {
          this->imu_measures_vector_from_lf_.push_back( imu_frame );
          this->imu_measures_deque_.pop_front();
        }
        else
        {
          this->imu_measures_vector_from_lf_.push_back( imu_frame );
          is_enough = true;
          break;
        }
      }
      else
      {
        is_enough = false;
      }
    }

    if ( !is_enough )
    {
      std::this_thread::sleep_for( std::chrono::microseconds( 500 ) );
    }
  }

  // 3. preintegrate the IMU data and store it in the current_frame_(f_2)
  this->PreintegrateIMU();
}


void FrontEnd::PreintegrateIMU()
{
  // TODO: implement preintegration

  if ( this->imu_measures_vector_from_lf_.empty() )
  {
    WARN( lk_vio::logger, "No IMU data available for preintegration" );
    return;
  }

  const int num = this->imu_measures_vector_from_lf_.size();

  // Create an IMUPreintegrator object with the IMU bias from the last frame and the IMU calibration from the current frame
  IMUPreintegrator *preintegrator_from_lf = new IMUPreintegrator( this->last_frame_->imu_bias_, this->current_frame_->imu_calib_ );

  for ( int i = 0; i < num; i++ )
  {
    double          time_step{ 0.0f };
    Eigen::Vector3f acc, ang_vel;
    // the first IMU data and the last IMU data need treated differently
    if ( ( i == 0 ) && ( i < ( num - 1 ) ) )  // the first
    {
      float imu_imu_time_diff   = this->imu_measures_vector_from_lf_[ i + 1 ].t - this->imu_measures_vector_from_lf_[ i ].t;
      float imu_image_time_diff = this->imu_measures_vector_from_lf_[ i ].t - this->current_frame_->last_frame_sptr_->timestamp_;

      acc       = ( ( this->imu_measures_vector_from_lf_[ i ].a + this->imu_measures_vector_from_lf_[ i + 1 ].a ) - ( this->imu_measures_vector_from_lf_[ i + 1 ].a - this->imu_measures_vector_from_lf_[ i ].a ) * ( imu_image_time_diff / imu_imu_time_diff ) ) * 0.5f;
      ang_vel   = ( ( this->imu_measures_vector_from_lf_[ i ].w + this->imu_measures_vector_from_lf_[ i + 1 ].w ) - ( this->imu_measures_vector_from_lf_[ i + 1 ].w - this->imu_measures_vector_from_lf_[ i ].w ) * ( imu_image_time_diff / imu_imu_time_diff ) ) * 0.5f;
      time_step = this->imu_measures_vector_from_lf_[ i + 1 ].t - this->current_frame_->last_frame_sptr_->timestamp_;
    }
    else if ( i < ( num - 1 ) )  // normal situation
    {
      acc       = ( this->imu_measures_vector_from_lf_[ i ].a + this->imu_measures_vector_from_lf_[ i + 1 ].a ) * 0.5f;
      ang_vel   = ( this->imu_measures_vector_from_lf_[ i ].w + this->imu_measures_vector_from_lf_[ i + 1 ].w ) * 0.5f;
      time_step = this->imu_measures_vector_from_lf_[ i + 1 ].t - this->imu_measures_vector_from_lf_[ i ].t;
    }
    else if ( ( i > 0 ) && ( i == ( num - 1 ) ) )  // the last one
    {
      float imu_imu_time_diff   = this->imu_measures_vector_from_lf_[ i + 1 ].t - this->imu_measures_vector_from_lf_[ i ].t;
      float imu_image_time_diff = this->imu_measures_vector_from_lf_[ i + 1 ].t - this->current_frame_->timestamp_;

      acc       = ( ( this->imu_measures_vector_from_lf_[ i ].a + this->imu_measures_vector_from_lf_[ i + 1 ].a ) - ( this->imu_measures_vector_from_lf_[ i + 1 ].a - this->imu_measures_vector_from_lf_[ i ].a ) * ( imu_image_time_diff / imu_imu_time_diff ) ) * 0.5f;
      ang_vel   = ( ( this->imu_measures_vector_from_lf_[ i ].w + this->imu_measures_vector_from_lf_[ i + 1 ].w ) - ( this->imu_measures_vector_from_lf_[ i + 1 ].w - this->imu_measures_vector_from_lf_[ i ].w ) * ( imu_image_time_diff / imu_imu_time_diff ) ) * 0.5f;
      time_step = this->current_frame_->timestamp_ - this->imu_measures_vector_from_lf_[ i ].t;
    }
    else if ( ( i == 0 ) && ( i == ( num - 1 ) ) )  // only one IMU data
    {
      acc       = this->imu_measures_vector_from_lf_[ i ].a;
      ang_vel   = this->imu_measures_vector_from_lf_[ i ].w;
      time_step = this->current_frame_->timestamp_ - this->current_frame_->last_frame_sptr_->timestamp_;
    }

    // Integrate the IMU data
    if ( this->preintegrator_from_kf_ == nullptr )
    {
      WARN( lk_vio::logger, "No preintegrator from last keyframe available, using identity preintegration" );
    }

    this->preintegrator_from_kf_->IntegrateNewMeasurement( acc, ang_vel, time_step );
    preintegrator_from_lf->IntegrateNewMeasurement( acc, ang_vel, time_step );
  }

  this->current_frame_->imu_preintegration_lf_ = preintegrator_from_lf;
  this->current_frame_->imu_preintegration_kf_ = this->preintegrator_from_kf_;
  this->current_frame_->key_frame_sptr_        = this->reference_kf_;  // todo: change to last key frame, because reference kf may be too old (long time ago)

  this->current_frame_->setIntegratedFlag();
}


bool FrontEnd::Track()
{
  /// use constant velocity model to preliminarily estimiate the current frame's pose
  if ( last_frame_ )
  {
    /// T{i k} = T{i-1_i-2} * T{i-1_k}
    /// A more general point of view is that relative_motion_ represents the motion between two adjacent frames,
    /// We have the pose estimation of the previous frame, then use the pose of the previous frame to multiply
    /// the motion according to the constant velocity model to represent the initial estimate of the pose of the current frame
    current_frame_->SetRelativePose( relative_motion_ * last_frame_->getRelativePose() );
  }

  TrackLastFrame();
  int inline_pts = EstimateCurrentPose();

  if ( inline_pts > num_features_tracking_good_ )
  {
    /// tracking good
    track_status_ = FrontendStatus::TRACKING_GOOD;
  }
  else if ( inline_pts > num_features_tracking_bad_ )
  {
    /// tracking bad
    track_status_ = FrontendStatus::TRACKING_BAD;
    // WARN( lk_vio::logger, "---------------------" );
    // WARN( lk_vio::logger, "--inline_pts:-- {0}", inline_pts );
    // WARN( lk_vio::logger, "----Tracking BAD!----" );
    // WARN( lk_vio::logger, "---------------------" );
  }
  else
  {
    /// lost
    track_status_ = FrontendStatus::LOST;
    WARN( lk_vio::logger, "----------------------" );
    WARN( lk_vio::logger, "--inline_pts:-- {0}", inline_pts );
    WARN( lk_vio::logger, "----Tracking LOST!----" );
    WARN( lk_vio::logger, "----------------------" );
  }

  relative_motion_ = current_frame_->getRelativePose() * last_frame_->getRelativePose().inverse();

  /// detect new features; create new mappoints; create new KF
  if ( track_status_ == FrontendStatus::TRACKING_BAD )
  {
    DetectFeatures();
    FindFeaturesInRight();
    TriangulateNewPoints();
    InsertKeyFrame();
  }
  return true;
}

int FrontEnd::TrackLastFrame()
{
  std::vector<cv::Point2f> kps_last, kps_current;
  kps_last.reserve( current_frame_->features_left_.size() );
  kps_current.reserve( last_frame_->features_left_.size() );
  for ( size_t i = 0; i < last_frame_->features_left_.size(); i++ )
  {
    Feature::Ptr  feat     = last_frame_->features_left_[ i ];
    MapPoint::Ptr mappoint = feat->map_point_.lock();
    kps_last.emplace_back( feat->kp_position_.pt );
    if ( mappoint )
    {
      std::unique_lock<std::mutex> lck( getset_reference_kp_numtex_ );
      Eigen::Vector2d              p = left_camera_->world2pixel( mappoint->getPosition(),
                                                     current_frame_->getRelativePose() *
                                                         reference_kf_->getPose() );
      kps_current.emplace_back( cv::Point2f( p.x(), p.y() ) );
    }
    else
    {
      kps_current.emplace_back( feat->kp_position_.pt );
    }
  }
  assert( kps_last.size() > 1 && kps_current.size() > 1 );
  std::vector<uchar> status;
  cv::Mat            error;
  cv::calcOpticalFlowPyrLK(
      last_frame_->left_image_,
      current_frame_->left_image_,
      kps_last,
      kps_current,
      status,
      error,
      cv::Size( 11, 11 ),
      5,
      cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 60, 0.005 ),
      cv::OPTFLOW_USE_INITIAL_FLOW );
  int num_good_track = 0;
  for ( size_t i = 0; i < status.size(); i++ )
  {
    /// 只会跟踪已经三角化后的点，因为没有三角化的点是无法形成BA约束的
    if ( status[ i ] && !last_frame_->features_left_[ i ]->map_point_.expired() )
    {
      cv::KeyPoint kp( kps_current[ i ], 7 );
      Feature::Ptr feature( new Feature( kp ) );
      feature->map_point_ = last_frame_->features_left_[ i ]->map_point_;
      current_frame_->features_left_.emplace_back( feature );
      num_good_track++;
    }
  }
  return num_good_track;
}

int FrontEnd::EstimateCurrentPose()
{
  EIGEN_ALIGN_TO_BOUNDARY( 16 )
  Eigen::Matrix3d K = left_camera_->getK();

  typedef g2o::BlockSolver_6_3                                    BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

  g2o::SparseOptimizer optimizer;

  auto solver = new g2o::OptimizationAlgorithmLevenberg( std::make_unique<BlockSolverType>( std::make_unique<LinearSolverType>() ) );
  optimizer.setAlgorithm( solver );

  /// vertes
  VertexPose *vertex_pose = new VertexPose();
  vertex_pose->setId( 0 );
  /// T{i_k} * T{k_w} = T{i_w}
  {
    std::unique_lock<std::mutex> lck( getset_reference_kp_numtex_ );
    vertex_pose->setEstimate( current_frame_->getRelativePose() * reference_kf_->getPose() );
  }
  optimizer.addVertex( vertex_pose );


  /// edges
  int                                   index = 1;
  std::vector<EdgeProjectionPoseOnly *> edges;
  std::vector<Feature::Ptr>             features;
  features.reserve( current_frame_->features_left_.size() );
  edges.reserve( current_frame_->features_left_.size() );
  for ( size_t i = 0; i < current_frame_->features_left_.size(); i++ )
  {
    MapPoint::Ptr mappoint = current_frame_->features_left_[ i ]->map_point_.lock();
    cv::Point2f   pt       = current_frame_->features_left_[ i ]->kp_position_.pt;
    if ( mappoint && !mappoint->is_outlier_ )
    {
      features.emplace_back( current_frame_->features_left_[ i ] );
      EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly( mappoint->getPosition(), K );
      edge->setId( index );
      edge->setVertex( 0, vertex_pose );
      edge->setMeasurement( Eigen::Vector2d( pt.x, pt.y ) );
      edge->setInformation( Eigen::Matrix2d::Identity() );
      edge->setRobustKernel( new g2o::RobustKernelHuber );

      edges.emplace_back( edge );
      optimizer.addEdge( edge );
      index++;
    }
  }

  // estimate the Pose and determine the outliers
  // start optimization
  const double chi2_th        = 5.991;
  int          cnt_outliers   = 0;
  int          num_iterations = 4;
  for ( int iteration = 0; iteration < num_iterations; iteration++ )
  {
    optimizer.initializeOptimization();
    optimizer.optimize( 10 );
    cnt_outliers = 0;

    // count the outliers, outlier is not included in estimation until it is regarded as a inlier
    for ( size_t i = 0, N = edges.size(); i < N; i++ )
    {
      auto e = edges[ i ];
      if ( features[ i ]->is_outlier_ )
      {
        e->computeError();
      }
      if ( e->chi2() > chi2_th )
      {
        features[ i ]->is_outlier_ = true;
        e->setLevel( 1 );
        cnt_outliers++;
      }
      else
      {
        features[ i ]->is_outlier_ = false;
        e->setLevel( 0 );
      }

      // remove the robust kernel to see if it's outlier
      if ( iteration == num_iterations - 2 )
      {
        e->setRobustKernel( nullptr );
      }
    }
  }

  /// set pose
  current_frame_->SetPose( vertex_pose->estimate() );
  /// T{i_k} = T{i_w} * T{k_w}.inverse()
  {
    std::unique_lock<std::mutex> lck( getset_reference_kp_numtex_ );

    current_frame_->SetRelativePose( vertex_pose->estimate() * reference_kf_->getPose().inverse() );
  }

  for ( auto &feat : features )
  {
    if ( feat->is_outlier_ )
    {
      std::unique_lock<std::mutex> lck( getset_reference_kp_numtex_ );
      MapPoint::Ptr                mp = feat->map_point_.lock();
      if ( mp && current_frame_->frame_id_ - reference_kf_->frame_id_ <= 2 )
      {
        mp->is_outlier_ = true;
        map_->AddOutlierMapPoint( mp->id_ );
      }
      feat->map_point_.reset();
      feat->is_outlier_ = false;
    }
  }

  // INFO( lk_vio::logger, "Frontend: Outliers/Inliers in frontend current pose estimating: {0}/{1}", cnt_outliers, features.size() - cnt_outliers );

  return features.size() - cnt_outliers;
}

int FrontEnd::DetectFeatures()
{
  cv::Mat mask( current_frame_->left_image_.size(), CV_8UC1, cv::Scalar::all( 255 ) );
  for ( const auto &feat : current_frame_->features_left_ )
  {
    cv::rectangle( mask,
                   feat->kp_position_.pt - cv::Point2f( 10, 10 ),
                   feat->kp_position_.pt + cv::Point2f( 10, 10 ),
                   0,
                   cv::FILLED );
  }

  std::vector<cv::KeyPoint> kps;
  if ( track_status_ == FrontendStatus::INITING )
  {
    orb_extractor_init_->Detect( current_frame_->left_image_, mask, kps );
  }
  else
  {
    orb_extractor_->Detect( current_frame_->left_image_, mask, kps );
  }

  int cnt_detected = 0;
  for ( const auto &kp : kps )
  {
    Feature::Ptr feature( new Feature( kp ) );
    current_frame_->features_left_.emplace_back( feature );
    cnt_detected++;
  }
  if ( show_orb_detect_result_ && cnt_detected > 0 )
  {
    cv::Mat show_img( current_frame_->left_image_.size(), CV_8UC1 );
    cv::drawKeypoints( current_frame_->left_image_, kps, show_img, cv::Scalar( 0, 255, 0 ) );
    cv::imshow( "ORB Detect Result", show_img );
    cv::waitKey( 1 );
  }

  if ( track_status_ == FrontendStatus::TRACKING_BAD )
  {
    // WARN( lk_vio::logger, "Detect New Features: {0} left image features size: {1}", cnt_detected, current_frame_->features_left_.size() );
  }
  return cnt_detected;
}

int FrontEnd::FindFeaturesInRight()
{
  std::vector<cv::Point2f> left_cam_kps, right_cam_kps;
  left_cam_kps.reserve( current_frame_->features_left_.size() );
  right_cam_kps.reserve( current_frame_->features_left_.size() );
  for ( const auto &feat : current_frame_->features_left_ )
  {
    left_cam_kps.push_back( feat->kp_position_.pt );
    auto pt_in_map = feat->map_point_.lock();
    if ( pt_in_map )
    {
      std::unique_lock<std::mutex> lck( getset_reference_kp_numtex_ );
      /// If it is a point that has been triangulated, it is projected onto the plane of the second
      /// camera through the pose of the first camera and the external parameters between the camera
      Eigen::Vector2d p = right_camera_->world2pixel( pt_in_map->getPosition(),
                                                      current_frame_->getRelativePose() *
                                                          reference_kf_->getPose() );
      right_cam_kps.push_back( cv::Point2f( p.x(), p.y() ) );
    }
    else
    {
      right_cam_kps.push_back( feat->kp_position_.pt );
    }
  }

  // LK flow: calculate the keypoints' positions in right frame
  // std::vector<uchar> status;
  cv::Mat error;
  cv::calcOpticalFlowPyrLK(
      current_frame_->left_image_,
      current_frame_->right_image_,
      left_cam_kps,
      right_cam_kps,
      this->lk_status_,
      error,
      cv::Size( 11, 11 ),
      5,
      cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 60, 0.005 ),
      cv::OPTFLOW_USE_INITIAL_FLOW );

  /// create new feature objects in right frame
  int num_good_points = 0;
  current_frame_->features_right_.reserve( this->lk_status_.size() );
  for ( size_t i = 0; i < this->lk_status_.size(); ++i )
  {
    if ( this->lk_status_[ i ] )
    {
      /// only the position of keypoint is needed, so size 7 is just for creation with no meaning
      cv::KeyPoint kp( right_cam_kps[ i ], 7 );
      Feature::Ptr feat( new Feature( kp ) );
      feat->is_on_left_frame_ = false;
      current_frame_->features_right_.push_back( feat );
      num_good_points++;
    }
    else
    {
      current_frame_->features_right_.push_back( nullptr );
    }
  }

  if ( show_lk_result_ )
  {
    cv::Mat show;
    cv::cvtColor( current_frame_->left_image_, show, cv::COLOR_GRAY2BGR );
    for ( size_t i = 0; i < current_frame_->features_left_.size(); i++ )
    {
      if ( this->lk_status_[ i ] )
      {
        cv::Point2i pt1 = current_frame_->features_left_[ i ]->kp_position_.pt;
        cv::Point2i pt2 = current_frame_->features_right_[ i ]->kp_position_.pt;
        cv::circle( show, pt1, 2, cv::Scalar( 0, 250, 0 ), 2 );
        cv::line( show, pt1, pt2, cv::Scalar( 0, 0, 255 ), 1.5 );
      }
    }
    cv::imshow( "LK Matches Result", show );
    cv::waitKey( 1 );
  }

  if ( track_status_ == FrontendStatus::TRACKING_BAD )
  {
    // WARN( lk_vio::logger, "LK Track New Features: {0}", num_good_points );
  }
  return num_good_points;
}

bool FrontEnd::SteroInit()
{
  int cnt_detected_features = DetectFeatures();
  int cnt_track_features    = FindFeaturesInRight();

  if ( cnt_track_features < num_features_init_good_ )
  {
    WARN( lk_vio::logger, "Too few feature points" );
    return false;
  }

  bool build_init_map_success = BuidInitMap();
  if ( build_init_map_success )
  {
    track_status_ = FrontendStatus::TRACKING_GOOD;
    return true;
  }

  return false;
}

bool FrontEnd::BuidInitMap()
{
  std::vector<Sophus::SE3d> poses{ left_camera_->getPose(), right_camera_->getPose() };
  size_t                    cnt_init_landmarks = 0;
  for ( size_t i = 0, N = current_frame_->features_left_.size(); i < N; i++ )
  {
    if ( current_frame_->features_right_[ i ] == nullptr )
    {
      continue;
    }
    /// create mappoints by triangulation
    std::vector<Eigen::Vector3d> points{
        left_camera_->pixel2camera(
            Eigen::Vector2d( current_frame_->features_left_[ i ]->kp_position_.pt.x,
                             current_frame_->features_left_[ i ]->kp_position_.pt.y ) ),
        right_camera_->pixel2camera(
            Eigen::Vector2d( current_frame_->features_right_[ i ]->kp_position_.pt.x,
                             current_frame_->features_right_[ i ]->kp_position_.pt.y ) ) };
    Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

    if ( triangulation( poses, points, pworld ) && pworld[ 2 ] > 0 )
    {
      /// if successfully triangulate, then create new mappoint and insert it to the map
      MapPoint::Ptr new_map_point( new MapPoint );
      new_map_point->SetPosition( pworld );
      current_frame_->features_left_[ i ]->map_point_  = new_map_point;
      current_frame_->features_right_[ i ]->map_point_ = new_map_point;

      if ( map_ )
      {
        map_->InsertMapPoint( new_map_point );
      }

      cnt_init_landmarks++;
    }
  }

  if ( cnt_init_landmarks < min_init_landmark_ )
  {
    WARN( lk_vio::logger, "Build init map Failed, have {0} points, min is: {1}", cnt_init_landmarks,
          min_init_landmark_ );
    return false;
  }

  InsertKeyFrame();
  INFO( lk_vio::logger, "Map initialized successfully with {0} points", cnt_init_landmarks );
  cv::destroyAllWindows();
  return true;
}

int FrontEnd::TriangulateNewPoints()
{
  auto                      cv_point2f_to_vec2 = []( cv::Point2f &pt ) { return Eigen::Vector2d( pt.x, pt.y ); };
  std::vector<Sophus::SE3d> poses{ left_camera_->getPose(), right_camera_->getPose() };
  Sophus::SE3d              current_pose_Twc;
  {
    std::unique_lock<std::mutex> lck( getset_reference_kp_numtex_ );
    current_pose_Twc = ( current_frame_->getRelativePose() * reference_kf_->getPose() ).inverse();
  }

  std::vector<PointXYZ> point_cloud;
  size_t                cnt_trangulat_pts = 0, cnt_previous_mappoint = 0;
  for ( size_t i = 0; i < current_frame_->features_left_.size(); i++ )
  {
    Feature::Ptr  feat_left  = current_frame_->features_left_[ i ];
    Feature::Ptr  feat_right = current_frame_->features_right_[ i ];
    MapPoint::Ptr mp         = feat_left->map_point_.lock();
    if ( !current_frame_->features_left_[ i ]->map_point_.expired() )
    {
      /// no need to triangulate
      cnt_previous_mappoint++;
      continue;
    }
    /// LK track failed
    if ( feat_right == nullptr )
      continue;


    std::vector<Eigen::Vector3d> points;
    points.emplace_back(
        left_camera_->pixel2camera( cv_point2f_to_vec2( feat_left->kp_position_.pt ) ) );
    points.emplace_back(
        right_camera_->pixel2camera( cv_point2f_to_vec2( feat_right->kp_position_.pt ) ) );
    Eigen::Vector3d pt_camera1;
    if ( triangulation( poses, points, pt_camera1 ) && pt_camera1[ 2 ] > 0 )
    {
      MapPoint::Ptr new_mappoint( new MapPoint );
      new_mappoint->SetPosition( current_pose_Twc * pt_camera1 );
      current_frame_->features_left_[ i ]->map_point_  = new_mappoint;
      current_frame_->features_right_[ i ]->map_point_ = new_mappoint;
      if ( map_ )
      {
        map_->InsertMapPoint( new_mappoint );
      }


      if ( ros_utilities_ )
      {
        point_cloud.emplace_back( PointXYZ{ new_mappoint->getPosition().x(),
                                            new_mappoint->getPosition().y(),
                                            new_mappoint->getPosition().z() } );
      }


      cnt_trangulat_pts++;
    }
  }

  if ( ros_utilities_ )
  {
    ros_utilities_->publishPointCloud( point_cloud );
  }
  // INFO( lk_vio::logger, "TriangulateNewPoints: {0} points, {1} previous mappoints", cnt_trangulat_pts, cnt_previous_mappoint );
  return cnt_trangulat_pts;
}

bool FrontEnd::InsertKeyFrame()
{
  Eigen::Matrix<double, 6, 1> se3_zero     = Eigen::Matrix<double, 6, 1>::Zero();
  KeyFrame::Ptr               new_keyframe = KeyFrame::CreateKF( current_frame_ );
  if ( track_status_ == FrontendStatus::INITING )
  {
    new_keyframe->SetPose( Sophus::SE3d::exp( se3_zero ) );
  }
  else
  {
    std::unique_lock<std::mutex> lck( getset_reference_kp_numtex_ );
    /// current_frame_->pose(); T_ik * T_kw = T_iw
    new_keyframe->SetPose( current_frame_->getRelativePose() * reference_kf_->getPose() );
    new_keyframe->last_key_frame_           = reference_kf_;
    new_keyframe->relative_pose_to_last_KF_ = current_frame_->getRelativePose();  /// T_ik
  }

  //////////////////////////////////////////////////////////////
  if ( backend_ )
  {
    backend_->InsertKeyFrame( new_keyframe, open_backend_optimization_ );
  }
  //////////////////////////////////////////////////////////////
  {
    std::unique_lock<std::mutex> lck( getset_reference_kp_numtex_ );
    reference_kf_ = new_keyframe;
  }
  current_frame_->SetRelativePose( Sophus::SE3d::exp( se3_zero ) );

  return true;
}

void FrontEnd::SetOrbExtractor( const std::shared_ptr<lk_vio::ORBextractor> &orb )
{
  assert( orb != nullptr );
  orb_extractor_ = orb;
}


void FrontEnd::SetOrbInitExtractor( const std::shared_ptr<lk_vio::ORBextractor> &orb )
{
  assert( orb != nullptr );
  orb_extractor_init_ = orb;
}

void FrontEnd::SetMap( const shared_ptr<Map> &map )
{
  assert( map != nullptr );
  map_ = map;
}

void FrontEnd::SetBackend( const shared_ptr<Backend> &backend )
{
  assert( backend != nullptr );
  backend_ = backend;
}

void FrontEnd::setRosUtilities( const std::shared_ptr<RosUtilities> &ros_utilities )
{
  assert( ros_utilities != nullptr );
  ros_utilities_ = ros_utilities;
}


}  // namespace lk_vio
