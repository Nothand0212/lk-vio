#include "frame.hpp"
#include "frontEnd.hpp"
#include "keyFrame.hpp"
#include "map.hpp"
#include "mapPoint.hpp"
#include "orbExtractor.hpp"
#include "utility.hpp"
// TODO
// #include "backEnd.hpp"

namespace lvio
{
FrontEnd::FrontEnd()
{
  num_features_tracking_init_ = Setting::Get<int>( "numFeatures.initGood" );
  num_features_tracking_good_ = Setting::Get<int>( "numFeatures.trackingGood" );
  num_features_tracking_bad_  = Setting::Get<int>( "numFeatures.trackingBad" );

  need_undistortion_ = Setting::Get<bool>( "Camera.NeedUndistortion" );

  show_orb_detect_result_ = Setting::Get<bool>( "Viewer.ORB.Extractor.Result" );
  show_lk_match_result_   = Setting::Get<bool>( "Viewer.LK.Flow" );

  min_init_map_point_ = Setting::Get<int>( "MapPoint.InitMin" );

  need_backend_optimization_ = Setting::Get<bool>( "BackEnd.Optimization" );
  LOG( INFO ) << "Open BackEnd Optimization: " << need_backend_optimization_;
}

void FrontEnd::setCamera( const Camera::Ptr &left_camera, const Camera::Ptr &right )
{
  assert( left_camera != nullptr && right_camera_ != nullptr );
  left_camera_  = left_camera;
  right_camera_ = right_camera_;
}

void FrontEnd::setViewUI( const std::shared_ptr<ui::PangolinWindow> &viewer_ui )
{
  assert( viewer_ui != nullptr );
  viewer_ui_ = viewer_ui;
}

void setORBextractor( const std::shared_ptr<ORBextractor> &orb_extractor )
{
  assert( orb_extractor != nullptr );
  orb_extractor_ptr_ = orb_extractor;
}

void setInitORBextractor( const std::shared_ptr<ORBextractor> &init_orb_extractor )
{
  assert( init_orb_extractor != nullptr );
  init_orb_extractor_ptr_ = init_orb_extractor;
}

void setMap( const std::shared_ptr<Map> &map )
{
  assert( map != nullptr );
  map_ptr_ = map;
}

// TODO
// void setBackEnd()
// {
//   // TODO
// }
//

bool FrontEnd::grabImageStereo( const cv::Mat &left_image, const cv::Mat &right_image, const double &time_stamp )
{
  current_frame_ptr_.reset( new Frame( left_image, right_image, timestamp ) );

  if ( need_undistortion_ )
  {
    left_camera_ptr_->unDistortImage( current_frame_ptr_->left_image_, current_frame_ptr_->left_image_ );
    right_camera_ptr_->unDistortImage( current_frame_ptr_->right_image_, current_frame_ptr_->right_image_ );
  }

  // 更新地图
  {
    std::unique_lock<std::mutex> lock( map_ptr_->map_update_mutex_ );

    switch ( tracking_status_ )
    {
      case FrontEndStatus::INITING:
      {
        initStereoFrame();
        break;
      }

      case FrontEndStatus::TRACKING_BAD:
      case FrontEndStatus::TRACKING_GOOD:
      {
        track();
        break;
      }
      case FrontEndStatus::LOST:
      {
        LOG( WARNING ) << "Tracking lost";
        // TODO
        break;
      }
    }
  }  // unlock map_update_mutex_

  if ( view_ui_ != nullptr )
  {
    // TODO
    // view_ui_->updateImageView( current_frame_ptr_->left_image_, current_frame_ptr_->right_image_ );
  }

  last_frame_ptr_ = current_frame_ptr_;
  return true;
}


bool FrontEnd::track()
{
  if ( last_frame_ )
  {
    // T_i_k 约等于 T_{i-1}_{i-2} * T_{i-1}_k
    // 这里时恒速模型假设，即上一帧的相对移动，和当前帧的相对移动应该是一样的。以此来计算一个大概的相对位姿，后面再继续优化。
    // TODO --> 用IMU预积分的结果来计算相对位姿，应该会好一点
    current_frame_->setRelativePose( relative_motion_ * last_frame_->getRelativePose() );
  }

  trackLastFrame();

  int num_inline_points = estimateCurrentPose();

  if ( num_inline_points > num_features_tracking_good_ )
  {
    tracking_status_ = FrontEndStatus::TRACKING_GOOD;
    LOG( INFO ) << "TRACKING_GOOD";
  }
  else if ( num_inline_points > num_features_tracking_bad_ )
  {
    tracking_status_ = FrontEndStatus::TRACKING_BAD;
    LOG( INFO ) << "TRACKING_BAD";
  }
  else
  {
    tracking_status_ = FrontEndStatus::LOST;
    LOG( WARNNING ) << "*************";
    LOG( WARNNING ) << "*TRACKING_LOST*";
    LOG( WARNNING ) << "*Inline Points Num: %ld*" << num_inline_points;
    LOG( WARNNING ) << "*************";
  }

  // T_{i}_{i-1} = T_i_k * T_k_{i-1}
  relative_motion_ = current_frame_->getRelativePose() * last_frame_->getRelativePose().inverse();

  if ( tracking_status_ == FrontEndStatus::BAD )
  {
    detectFeatures();
    findFeaturesInRight();
    triangulateNewPoints();
    insertKeyFrame();
  }

  return true;
}

int FrontEnd::trackLastFrame()
{
  std::vector<cv::Point2f> current_key_points_vec, last_key_points_vec;
  current_key_points_vec.reserve( current_frame_->features_left_.size() );
  last_key_points_vec.reserve( last_frame_->features_left_.size() );

  for ( std::size_t i = 0; i < last_frame_->features_left_.size(); ++i )
  {
    Feature::Ptr  feature_ptr   = last_frame_->features_left_[ i ];
    MapPoint::Ptr map_point_ptr = feature_ptr->map_point_.lock();

    last_key_points_vec.emplace_back( feature_ptr->position_.pt );

    if ( map_point_ptr )
    {
      std::unique_lock<std::mutex> lock( update_key_frame_map_point_mutex_ );

      Eigen::Vector2d pixel_point = left_camera_->world2pixel( map_point_ptr->getPosition(), current_frame_->getRelativePose() * reference_kf_->getPose() );

      current_key_points_vec.emplace_back( cv::Point2f( pixel_point.x(), pixel_point.y() ) );
    }
    else
    {
      current_key_points_vec.emplace_back( feature_ptr->position_.pt );
    }
  }

  assert( last_key_points_vec.size() > 1 && current_key_points_vec.size() > 1 );

  std::vector<uchar> status;
  cv::Mat            error;

  cv::calcOpticalFlowPyrLK( last_frame_->left_image_, current_frame_->left_image_,
                            last_key_points_vec, current_key_points_vec,
                            status,                                                                         // 用于标记跟踪成功的特征点
                            error,                                                                          // 记录每个特征点的误差
                            cv::Size( 11, 11 ),                                                             // 光流窗口，用于计算光流向量
                            3,                                                                              // 金字塔层
                            cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01 ),  // 停止条件：最大迭代次数30，最小误差0.01
                            cv::OPTFLOW_USE_INITIAL_FLOW                                                    // 使用初始光流向量计算，提高计算速度和精度
  );

  int num_good_tracking_points = 0;
  for ( std::size_t i = 0; i < status.size(); i++ )
  {
    // expired返回true表示该智能指针已经被释放，只有当该特征点对应的地图点存在时，才进行跟踪
    if ( status[ i ] && !last_frame_->features_left_[ i ]->map_point_.expired() )
    {
      cv::KeyPoint key_point( current_key_points_vec[ i ], 7 );  // 7为ORB特征点所在圆的半径
      Feature::Ptr feature_ptr = Feature::Ptr( new Feature( key_point ) );
      feature_ptr->map_point_  = last_frame_->features_left_[ i ]->map_point_;
      current_frame_->features_left_.emplace_back( feature_ptr );
      num_good_tracking_points++;
    }
  }

  return num_good_tracking_points;
}

int FrontEnd::estimateCurrentPose()
{
  Eigen::Matrix3d K = left_camera_->getK();

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3> >           BlockSolverType;
  typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

  auto solver = new g2o::OptimizationAlgorithmLevenberg( g2o::make_unique<BlockSolverType>( g2o::make_unique<LinearSolverType>() ) );

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm( solver );

  // 添加顶点
  VertexPose *vertex_pose = new VertexPose();
  vertex_pose->setId( 0 );

  // 顶点的估计值为当前帧的位姿
  {
    std::unique_lock<std::mutex> lock( update_key_frame_map_point_mutex_ );
    vertex_pose->setEstimate( current_frame_->getRelativePose() * reference_kf_->getPose() );
  }
  optimizer.addVertex( vertex_pose );

  // 添加边
  int index = 1;

  std::vector<EdgeProjectionPoseOnly *> edges;

  std::vector<Feature::Ptr> features;

  features.reserve( current_frame_->features_left_.size() );
  edges.reserve( current_frame_->features_left_.size() );

  for ( std::size_t i = 0; i < current_frame_->features_left_.size(); ++i )
  {
    MapPoint::Ptr map_point_ptr = current_frame_->features_left_[ i ]->map_point_.lock();
    cv::Point2f   pt            = current_frame_->features_left_[ i ]->position_.pt;
    if ( map_point_ptr && !map_point_ptr->is_outlier_ )
    {
      features.emplace_back( current_frame_->features_left_[ i ] );
      EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly( map_point_ptr->getPosition(), K );
      edge->setId( index );
      edge->setVertex( 0, vertex_pose );
      edge->setMeasurement( Eigen::Vector2d( pt.x, pt.y ) );
      edge->setInformation( Eigen::Matrix2d::Identity() );
      edge->setRobustKernel( new g2o::RobustKernelHuber() );
      edges.emplace_back( edge );
      optimizer.addEdge( edge );
      index++;
    }
  }

  const double chi2_th           = 5.991;  // 误差平方和的阈值
  int          count_of_outliers = 0;
  int          max_iteration     = 4;
  int          num_edges         = edges.size();
  for ( int iteration = 0; iteration < max_iteration; iteration++ )
  {
    optimizer.initializeOptimization();
    optimizer.optimize( 10 );
    count_of_outliers = 0;

    // 统计内外点
    for ( std::size_t i = 0; i < num_edges; i++ )
    {
      auto temp_edge = edges[ i ];

      if ( features[ i ]->is_outlier_ )
      {
        temp_edge->computeError();
      }

      if ( temp_edge->chi2() > chi2_th )
      {
        features[ i ]->is_outlier_ = true;
        temp_edge->setLevel( 1 );  // 将边的等级设为1，下次优化时不考虑这条边
        count_of_outliers++;
      }
      else
      {
        features[ i ]->is_outlier_ = false;
        temp_edge->setLevel( 0 );
      }

      // 优化两次之后，关闭鲁棒核，如果某些点的误差还是很大，那么就认为这些点是outlier
      if ( iteration == max_iteration - 2 )
      {
        temp_edge->setRobustKernel( nullptr );
      }
    }
  }

  // 设置位姿
  current_frame_->setPose( vertex_pose->estimate() );
  {  // T_i_k = T_i_w * T_k_w.inverse()
    std::unique_lock<std::mutex> lock( update_key_frame_map_point_mutex_ );
    current_frame_->setRelativePose( vertex_pose->estimate() * reference_kf_->getPose().inverse() );
  }

  // 删除outlier，如果有对应的地图点，修改地图点的内点flag
  for ( auto &feature : features )
  {
    if ( feature->is_outlier_ )
    {
      std::unique_lock<std::mutex> lock( update_key_frame_map_point_mutex_ );
      MapPoint::Ptr                map_point_ptr = feature->map_point_.lock();
      if ( map_point_ptr && current_frame_->frame_id_ - reference_kf->frame_id_ <= 2 )
      {
        map_point_ptr->is_outlier_ = true;
        map_ptr_->addOutlierMapPoint( map_point_ptr->getId() );
      }

      feature->map_point_.reset();
      feature->is_outlier_ = false;
    }
  }

  // 返回内点数量
  return features.size() - count_of_outliers;
}

int FrontEnd::detectFeatures()
{
  cv::Mat mask( current_frame_->left_image_.size(), CV_8UC1, cv::Scalar::all( 255 ) );

  // 用矩形框标记出特征点所在位置
  for ( const auto &feature : current_frame_->features_left_ )
  {
    cv::rectangle( mask,
                   feature->position_.pt - cv::Point2f( 10, 10 ),
                   feature->position_.pt + cv::Point2f( 10, 10 ),
                   0,
                   CV_FILLED );
  }

  std::vector<cv::KeyPoint> key_points_vec;

  if ( tracking_status == FrontEndStatus::INITING )
  {
    init_orb_extractor_ptr_->detect( current_frame_->left_image_, mask, key_points_vec );
  }
  else
  {
    orb_extractor_ptr_->detect( current_frame_->left_image_, mask, key_points_vec );
  }

  int count_of_detected_features = 0;

  for ( const auto &key_point : key_points_vec )
  {
    Feature::Ptr feature_ptr = Feature::Ptr( new Feature( key_point ) );
    current_frame_->features_left_.emplace_back( feature_ptr );
    count_of_detected_features++;
  }

  if ( show_orb_detect_result_ && count_of_detected_features > 0 )
  {
    cv::Mat image_show( current_frame_->left_image_.size(), CV_8UC1 );
    cv::drawKeypoints( current_frame_->left_image_, key_points_vec, image_show, cv::Scalar( 0, 255, 0 ) );
    cv::imshow( "ORB Features", image_show );
    cv::waitKey( 1 );
  }

  if ( tracking_status_ == FrontEndStatus::TRACKING_BAD )
  {
    // TODO
    LOG( WARNNING ) << "TRACKING_BAD";
    LOG( WARNNING ) << "Detect New Features: " << count_of_detected_features;
  }

  return count_of_detected_features;
}

int FrontEnd::findFeaturesInRight()
{
  std::vector<cv::Point2f> left_cam_key_points_vec, right_cam_key_points_vec;
  left_cam_key_points_vec.reserve( current_frame_->features_left_.size() );
  right_cam_key_points_vec.reserve( current_frame_->features_left_.size() );

  for ( const auto &feature : current_frame_->features_left_ )
  {
    left_cam_key_points_vec.emplace_back( feature->position_.pt );

    auto map_point_ptr = feature->map_point_.lock();

    if ( map_point_ptr )
    {
      std::unique_lock<std::mutex> lock( update_key_frame_map_point_mutex_ );

      Eigen::Vector2d pixel_point = left_camera_->world2pixel( map_point_ptr->getPosition(), current_frame_->getRelativePose() * reference_kf_->getPose() );
      right_cam_key_points_vec.emplace_back( cv::Point2f( pixel_point.x(), pixel_point.y() ) );
    }
    else
    {
      right_cam_key_points_vec.emplace_back( feature->position_.pt );
    }
  }

  // 用光流法再右图里找匹配点
  std::vector<uchar> status;
  cv::Mat            error;
  cv::calcOpticalFlowPyrLK(
      current_frame_->left_image_, current_frame_->right_image_,
      left_cam_key_points_vec, right_cam_key_points_vec,
      status,                                                                         // 用于标记跟踪成功的特征点
      error,                                                                          // 记录每个特征点的误差
      cv::Size( 11, 11 ),                                                             // 光流窗口，用于计算光流向量
      3,                                                                              // 金字塔层
      cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01 ),  // 停止条件：最大迭代次数30，最小误差0.01
      cv::OPTFLOW_USE_INITIAL_FLOW                                                    // 使用初始光流向量计算，提高计算速度和精度

  );

  int num_good_tracking_points = 0;
  current_frame_->features_right_.reserve( status.size() );

  for ( std::size_t i = 0; i < status.size(); ++i )
  {
    if ( status[ i ] )
    {
      cv::KeyPoint key_point( right_cam_key_points_vec[ i ], 7 );
      Feature::Ptr feature_ptr       = Feature::Ptr( new Feature( key_point ) );
      feature_ptr->is_on_left_image_ = false;
      current_frame_->features_right_.emplace_back( feature_ptr );
      num_good_tracking_points++;
    }
    else
    {
      current_frame_->features_right_.emplace_back( nullptr );
    }
  }

  if ( show_lk_result_ )
  {
    cv::Mat image_show;
    cv::cvtColor( current_frame_->left_image_, image_show, CV_GRAY2RGB );
    for ( std::size_t i = 0; i < current_frame_->features_left_.size(); i++ )
    {
      if ( status[ i ] )
      {
        cv::Point2i pt_1 = current_frame_->features_left_[ i ]->position_.pt;
        cv::Point2i pt_2 = current_frame_->features_right_[ i ]->position_.pt;
        cv::circle( image_show, pt_1, 2, cv::Scalar( 0, 250, 0 ), 2 );
        cv::line( image_show, pt_1, pt_2, cv::Scalar( 0, 250, 0 ), 1.5 );
      }
    }

    cv::imshow( "LK Flow", image_show );
    cv::waitKey( 1 );
  }

  if ( track_status == FrontEndStatus::TRACKING_BAD )
  {
    LOG( WARNING ) << "TRACKING_BAD";
    LOG( WARNING ) << "Find Features in Right: " << num_good_tracking_points;
  }

  return num_good_tracking_points;
}

bool FrontEnd::initStereoFrame()
{
  int count_of_detected_features = detectFeatures();
  int count_of_tracking_features = findFeaturesInRight();

  if ( count_of_detected_features < num_features_tracking_init_ )
  {
    LOG( WARNING ) << "Detect Features: " << count_of_detected_features << ", less than " << num_features_tracking_init_;
    return false;
  }

  bool build_init_map_success = buildInitMap();
  if ( build_init_map_success )
  {
    tracking_status_ = FrontEndStatus::TRACKING_GOOD;
    LOG( INFO ) << "TRACKING_GOOD";
    return true;
  }
  else
  {
    LOG( WARNING ) << "Build Init Map Failed";
    return false;
  }
}

bool FrontEnd::buildInitMap()
{
  std::vector<Sophus::SE3d> poses{ left_camera_->getPose(), right_camera_->getPose() };
  LOG( INFO ) << "Pose Size: " << poses.size() << " for Init Map";

  std::size_t count_of_inti_map_points = 0;
  std::size_t num_left_features        = current_frame_->features_left_.size();
  for ( std::size_t i = 0; i < num_left_features; i++ )
  {
    if ( current_frame_->features_right_[ i ] == nullptr )
    {
      continue;
    }

    std::vector<Eigen::Vector3d> points_vec{ left_camera_->pixel2camera( Eigen::Vector2d( current_frame_->features_left_[ i ]->position_.pt.x, current_frame_->features_left_[ i ]->position_.pt.y ) ),
                                             right_camera_->pixel2camera( Eigen::Vector2d( current_frame_->features_right_[ i ]->position_.pt.x, current_frame_->features_right_[ i ]->position_.pt.y ) ) };

    Eigen::Vector3d point_in_world = Eigen::Vector3d::Zero();

    if ( traiangulation( poses, points_vec, point_in_world ) )
    {
      MapPoint::Ptr map_point_ptr = MapPoint::Ptr( new MapPoint );
      map_point_ptr->setPosition( point_in_world );

      current_frame_->features_left_[ i ]->map_point_  = map_point_ptr;
      current_frame_->features_right_[ i ]->map_point_ = map_point_ptr;
      if ( map_ )
      {
        map_->insertMapPoint( map_point_ptr );
      }

      if ( view_ui_ )
      {
        // TODO
        // view_ui_->addShowMapPoint( map_point_ptr );
      }

      count_of_inti_map_points++;
    }
  }

  if ( count_of_inti_map_points < min_init_map_point_ )
  {
    LOG( WARNING ) << "Init Map Failed, There are " << count_of_inti_map_points << " Map Points, Min Threshold is " << min_init_map_point_;
    return false;
  }

  insertKeyFrame();
  LOG( INFO ) << "Build Init Map Success, There are " << count_of_inti_map_points << " Map Points";
  cv::destroyAllWindows();
  return true;
}

inline Eigen::Vector2d cvPoint2fToEigenVector2d( const cv::Point2f &pt )
{
  return Eigen::Vector2d( pt.x, pt.y );
}

int FrontEnd::triangluateNewPoints()
{
  std::vector<Sophus::SE3d> poses{ left_camera_->getPose(), right_camera_->getPose() };
  Sophus::SE3d              current_pose_Twc;

  {
    std::unique_lock<std::mutex> lock( update_key_frame_map_point_mutex_ );
    current_pose_Twc = ( current_frame_->getRelativePose() * reference_kf_->getPose() ).inverse();
  }

  std::size_t count_of_new_map_points = 0;
  std::size_t num_previous_map_points = 0;
  std::size_t num_features            = current_frame_->features_left_.size();

  for ( std::size_t i = 0; i < num_features; i++ )
  {
    Feature::Ptr  feature_left_ptr  = current_frame_->features_left_[ i ];
    Feature::Ptr  feature_right_ptr = current_frame_->features_right_[ i ];
    MapPoint::Ptr map_point_ptr     = feature_left_ptr->map_point_.lock();

    if ( !current_frame_->features_left_[ i ]->map_point_.expired() )
    {
      num_previous_map_points++;
      continue;
    }

    if ( feature_right_ptr == nullptr )
    {
      continue;
    }

    std::vector<Eigen::Vector3d> points;
    points.emplace_back( left_camera_->pixel2camera( cvPoint2fToEigenVector2d( feature_left_ptr->position_.pt ) ) );
    points.emplace_back( right_camera_->pixel2camera( cvPoint2fToEigenVector2d( feature_right_ptr->position_.pt ) ) );

    Eigen::Vector3d point_in_world = Eigen::Vector3d::Zero();
    if ( triangleation( poses, points, point_in_world ) && point_in_world[ 2 ] > 0 )
    {
      MapPoint::Ptr new_map_point_ptr = MapPoint::Ptr( new MapPoint );
      new_map_point_ptr->setPosition( current_pose_Twc * point_in_world );
      current_frame_->features_left_[ i ]->map_point_  = new_map_point_ptr;
      current_frame_->features_right_[ i ]->map_point_ = new_map_point_ptr;
      if ( map_ )
      {
        map_->insertMapPoint( new_map_point_ptr );
      }

      if ( view_ui_ )
      {
        // TODO
        // view_ui_->addShowMapPoint( new_map_point_ptr );
      }

      count_of_new_map_points++;
    }
  }
  LOG( INFO ) << "count_of_new_map_points: " << count_of_new_map_points << " num_previous_map_points: " << num_previous_map_points << " num_features: " << num_features;
  return count_of_new_map_points;
}

bool FrontEnd::insertKeyFrame()
{
  Eigen::Matrix<double, 6, 1> se3_zero = Eigen::Matrix<double, 6, 1>::Zero();

  KeyFrame::Ptr new_key_frame_ptr = KeyFrame::CreateKeyFrame( current_frame_ );

  if ( tracking_status_ == FrontEndStatus::INITING )
  {
    new_key_frame_ptr->setPose( Sophus::SE3d::exp( se3_zero ) );
  }
  else
  {
    std::unique_lock<std::mutex> lock( update_key_frame_map_point_mutex_ );
    // T_i_w = T_i_k * T_k_w
    new_key_frame_ptr->setPose( current_frame_->getRelativePose() * reference_key_frame_->getPose() );
    new_key_frame_ptr->last_key_frame_                  = reference_key_frame_;
    new_key_frame_ptr->relative_pose_to_last_key_frame_ = current_frame_->getRelativePose();
  }

  // TODO
  //   if ( back_end_ != nullptr )
  //   {
  //     back_end_->insertKeyFrame( new_key_frame_ptr, open_backend_optimization_ );
  //   }

  {
    std::unique_lock<std::mutex> lock( update_key_frame_map_point_mutex_ );
    reference_key_frame_ = new_key_frame_ptr;
  }

  current_frame_->setRelativePose( Sophus::SE3d::exp( se3_zero ) );

  return true;
}

}  // namespace lvio