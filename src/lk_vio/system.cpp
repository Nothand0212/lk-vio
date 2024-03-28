#include "lk_vio/map.hpp"
#include "lk_vio/system.hpp"
// #include "ui/pangolin_window.hpp"
namespace lk_vio
{
  // System::System( const common::Configuration &config )
  // {
  //   GenerateSteroCamera( config );
  //   GenerateORBextractor( config );

  //   // Map
  //   map_ = std::shared_ptr<Map>( new Map( config ) );

  //   // LoopClosing
  //   loop_closing_ = std::shared_ptr<LoopClosing>( new LoopClosing( config ) );
  //   loop_closing_->SetSteroCamera( left_camera_, right_camera_ );
  //   loop_closing_->SetMap( map_ );

  //   // ROS
  //   ros_utilities_ = std::shared_ptr<RosUtilities>( new RosUtilities( config ) );
  //   ros_utilities_->setMap( map_ );

  //   // Backend
  //   backend_ = std::make_shared<Backend>();
  //   backend_->SetMap( map_ );
  //   backend_->SetCameras( left_camera_, right_camera_ );
  //   // backend_->SetViewer( view_ui_ );
  //   backend_->SetLoopClosing( loop_closing_ );

  //   // Frontend
  //   frontend_ = std::shared_ptr<FrontEnd>( new FrontEnd( config ) );
  //   frontend_->SetCamera( left_camera_, right_camera_ );
  //   frontend_->SetOrbExtractor( orb_extractor_ );
  //   frontend_->SetOrbInitExtractor( orb_init_extractor_ );
  //   // frontend_->SetViewUI( view_ui_ );
  //   frontend_->SetMap( map_ );
  //   frontend_->SetBackend( backend_ );
  //   frontend_->setRosUtilities( ros_utilities_ );

  //   // LoopClosing
  //   loop_closing_->SetBackend( backend_ );
  //   loop_closing_->SetFrontend( frontend_ );
  // }

  System::System( const common::ParamServer &config )
  {
    GenerateSteroCamera( config );
    GenerateORBextractor( config );

    // Map
    map_ = std::shared_ptr<Map>( new Map( config ) );

    // LoopClosing
    loop_closing_ = std::shared_ptr<LoopClosing>( new LoopClosing( config ) );
    loop_closing_->SetSteroCamera( left_camera_, right_camera_ );
    loop_closing_->SetMap( map_ );

    // ROS
    ros_utilities_ = std::shared_ptr<RosUtilities>( new RosUtilities( config ) );
    ros_utilities_->setMap( map_ );

    // Backend
    backend_ = std::make_shared<Backend>();
    backend_->SetMap( map_ );
    backend_->SetCameras( left_camera_, right_camera_ );
    // backend_->SetViewer( view_ui_ );
    backend_->SetLoopClosing( loop_closing_ );

    // Frontend
    frontend_ = std::shared_ptr<FrontEnd>( new FrontEnd( config ) );
    frontend_->SetCamera( left_camera_, right_camera_ );
    frontend_->SetOrbExtractor( orb_extractor_ );
    frontend_->SetOrbInitExtractor( orb_init_extractor_ );
    // frontend_->SetViewUI( view_ui_ );
    frontend_->SetMap( map_ );
    frontend_->SetBackend( backend_ );
    frontend_->setRosUtilities( ros_utilities_ );

    // LoopClosing
    loop_closing_->SetBackend( backend_ );
    loop_closing_->SetFrontend( frontend_ );
  }


  bool System::RunStep( const cv::Mat &left_img, const cv::Mat &right_img, const double &timestamp )
  {
    if ( left_img.empty() || right_img.empty() || timestamp < 0 )
    {
      SPDLOG_LOGGER_ERROR( lk_vio::logger, "Input image is empty or timestamp is negative." );

      return false;
    }


    bool track_success = frontend_->GrabSteroImage( left_img, right_img, timestamp );
    return track_success;
  }
  bool System::RunStep( const cv::Mat &left_img, const cv::Mat &right_img, const double &timestamp, const std::vector<IMUFrame> &imu_measures )
  {
    if ( left_img.empty() || right_img.empty() || timestamp < 0 || imu_measures.empty() )
    {
      SPDLOG_LOGGER_ERROR( lk_vio::logger, "Input image is empty || timestamp is negative || imu_measures is empty. Please Check the Data..." );

      return false;
    }


    bool track_success = frontend_->GrabSteroImage( left_img, right_img, timestamp );
    return track_success;
  }
  // void System::GenerateSteroCamera( const common::Configuration &config )
  // {
  //   bool camera_need_undistortion = config.camera_params.need_undistortion;

  //   float           fx_left = config.camera_params.left.fx;
  //   float           fy_left = config.camera_params.left.fy;
  //   float           cx_left = config.camera_params.left.cx;
  //   float           cy_left = config.camera_params.left.cy;
  //   Eigen::Vector3d t_left  = Eigen::Vector3d::Zero();

  //   float           fx_right = config.camera_params.right.fx;
  //   float           fy_right = config.camera_params.right.fy;
  //   float           cx_right = config.camera_params.right.cx;
  //   float           cy_right = config.camera_params.right.cy;
  //   float           bf       = config.camera_params.baseline;
  //   float           baseline = bf / fx_right;
  //   Eigen::Vector3d t_right  = Eigen::Vector3d( -baseline, 0, 0 );


  //   cv::Mat dist_coef_left  = cv::Mat::zeros( 4, 1, CV_32F );
  //   cv::Mat dist_coef_right = cv::Mat::zeros( 4, 1, CV_32F );
  //   if ( camera_need_undistortion )
  //   {
  //     dist_coef_left.at<float>( 0 ) = config.camera_params.left.k1;
  //     dist_coef_left.at<float>( 1 ) = config.camera_params.left.k2;
  //     dist_coef_left.at<float>( 2 ) = config.camera_params.left.p1;
  //     dist_coef_left.at<float>( 3 ) = config.camera_params.left.p2;

  //     dist_coef_right.at<float>( 0 ) = config.camera_params.right.k1;
  //     dist_coef_right.at<float>( 1 ) = config.camera_params.right.k2;
  //     dist_coef_right.at<float>( 2 ) = config.camera_params.right.p1;
  //     dist_coef_right.at<float>( 3 ) = config.camera_params.right.p2;
  //   }
  //   // set the pose of left camera as identity isometry matrix by default
  //   left_camera_ = std::shared_ptr<lk_vio::Camera>( new Camera( fx_left,
  //                                                               fy_left,
  //                                                               cx_left,
  //                                                               cy_left,
  //                                                               0,
  //                                                               Sophus::SE3d( Sophus::SO3d(), t_left ),
  //                                                               dist_coef_left ) );

  //   right_camera_ = std::shared_ptr<lk_vio::Camera>( new Camera( fx_right,
  //                                                                fy_right,
  //                                                                cx_right,
  //                                                                cy_right,
  //                                                                baseline,
  //                                                                Sophus::SE3d( Sophus::SO3d(), t_right ),
  //                                                                dist_coef_right ) );
  // }

  // void System::GenerateORBextractor( const common::Configuration &config )
  // {
  //   int   num_orb_bew_features = config.extractor_params.init_features_num;
  //   float scale_factor         = config.extractor_params.scale_factor;
  //   int   n_levels             = config.extractor_params.pyramid_level;
  //   int   fIniThFAST           = config.extractor_params.init_fast_threshold;
  //   int   fMinThFAST           = config.extractor_params.min_fast_threshold;
  //   orb_extractor_             = ORBextractor::Ptr( new ORBextractor( num_orb_bew_features, scale_factor, n_levels, fIniThFAST, fMinThFAST ) );

  //   int num_features_init = config.extractor_params.init_features_num;
  //   orb_init_extractor_   = ORBextractor::Ptr( new ORBextractor( num_features_init, scale_factor, n_levels, fIniThFAST, fMinThFAST ) );
  // }
  void System::GenerateSteroCamera( const common::ParamServer &config )
  {
    bool camera_need_undistortion = config.camera_params.need_undistortion;

    float           fx_left = config.camera_params.left.fx;
    float           fy_left = config.camera_params.left.fy;
    float           cx_left = config.camera_params.left.cx;
    float           cy_left = config.camera_params.left.cy;
    Eigen::Vector3d t_left  = Eigen::Vector3d::Zero();

    float           fx_right = config.camera_params.right.fx;
    float           fy_right = config.camera_params.right.fy;
    float           cx_right = config.camera_params.right.cx;
    float           cy_right = config.camera_params.right.cy;
    float           bf       = config.camera_params.baseline;
    float           baseline = bf / fx_right;
    Eigen::Vector3d t_right  = Eigen::Vector3d( -baseline, 0, 0 );


    cv::Mat dist_coef_left  = cv::Mat::zeros( 4, 1, CV_32F );
    cv::Mat dist_coef_right = cv::Mat::zeros( 4, 1, CV_32F );
    if ( camera_need_undistortion )
    {
      dist_coef_left.at<float>( 0 ) = config.camera_params.left.k1;
      dist_coef_left.at<float>( 1 ) = config.camera_params.left.k2;
      dist_coef_left.at<float>( 2 ) = config.camera_params.left.p1;
      dist_coef_left.at<float>( 3 ) = config.camera_params.left.p2;

      dist_coef_right.at<float>( 0 ) = config.camera_params.right.k1;
      dist_coef_right.at<float>( 1 ) = config.camera_params.right.k2;
      dist_coef_right.at<float>( 2 ) = config.camera_params.right.p1;
      dist_coef_right.at<float>( 3 ) = config.camera_params.right.p2;
    }
    // set the pose of left camera as identity isometry matrix by default
    left_camera_ = std::shared_ptr<lk_vio::Camera>( new Camera( fx_left,
                                                                fy_left,
                                                                cx_left,
                                                                cy_left,
                                                                0,
                                                                Sophus::SE3d( Sophus::SO3d(), t_left ),
                                                                dist_coef_left ) );

    right_camera_ = std::shared_ptr<lk_vio::Camera>( new Camera( fx_right,
                                                                 fy_right,
                                                                 cx_right,
                                                                 cy_right,
                                                                 baseline,
                                                                 Sophus::SE3d( Sophus::SO3d(), t_right ),
                                                                 dist_coef_right ) );
  }

  void System::GenerateORBextractor( const common::ParamServer &config )
  {
    int   num_orb_bew_features = config.extractor_params.init_features_num;
    float scale_factor         = config.extractor_params.scale_factor;
    int   n_levels             = config.extractor_params.pyramid_level;
    int   fIniThFAST           = config.extractor_params.init_fast_threshold;
    int   fMinThFAST           = config.extractor_params.min_fast_threshold;
    orb_extractor_             = ORBextractor::Ptr( new ORBextractor( num_orb_bew_features, scale_factor, n_levels, fIniThFAST, fMinThFAST ) );

    int num_features_init = config.extractor_params.init_features_num;
    orb_init_extractor_   = ORBextractor::Ptr( new ORBextractor( num_features_init, scale_factor, n_levels, fIniThFAST, fMinThFAST ) );
  }
}  // namespace lk_vio