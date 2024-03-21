#include <filesystem>

#include "common/color.hpp"
#include "lk_vio/backend.hpp"
#include "lk_vio/camera.hpp"
#include "lk_vio/feature.hpp"
#include "lk_vio/frontend.hpp"
#include "lk_vio/g2otypes.hpp"
#include "lk_vio/keyframe.hpp"
#include "lk_vio/loopclosing.hpp"
#include "lk_vio/mappoint.hpp"
#include "logger/logger.h"
namespace lk_vio
{
  LoopClosing::LoopClosing( const common::Configuration &config )
  {
    LoadParam( config );
    GenerateORBextractor( config );
    loop_thread_is_running_.store( true );

    cv_matcher_ = cv::DescriptorMatcher::create( "BruteForce-Hamming" );

    std::string orb_voc_path = config.dbow2_voc_path;


    if ( orb_voc_path.empty() )

    {
      ERROR( lk_vio::logger, "ORB Vocabulary path is empty. Loop Closing is disabled." );
      return;
    }

    if ( !std::filesystem::exists( orb_voc_path ) )
    {
      ERROR( lk_vio::logger, "ORB Vocabulary path does not exist. Loop Closing is disabled." );
      return;
    }

    INFO( lk_vio::logger, "orb_voc_path: {0}", orb_voc_path );


    if ( open_loop_closing_ )
    {
      dbow2_vocabulary_ = std::make_unique<ORBVocabulary>();
      WARN( lk_vio::logger, "Reading ORB Vocabulary txt File, Wait a Second....." );
      // dbow2_vocabulary_->loadFromTextFile( orb_voc_path );
      dbow2_vocabulary_->loadFromBinaryFile( orb_voc_path );
      loop_closing_thread_ = std::thread( &LoopClosing::LoopClosingThread, this );
    }
  }

  void LoopClosing::LoopClosingThread()
  {
    while ( loop_thread_is_running_.load() )
    {
      if ( CheckNewKeyFrame() )
      {
        ProcessNewKeyframe();

        bool confirm_loop_closing = false;
        if ( key_frame_database_.size() > keyframe_database_min_size_ )
        {
          if ( DetectLoop() )
          {
            if ( MatchFeatures() )
            {
              confirm_loop_closing = ComputeCorrectPose();
              if ( confirm_loop_closing )
              {
                LoopCorrect();
              }
            }
          }
        }
        if ( !confirm_loop_closing )
        {
          AddToKeyframeDatabase();
        }
      }

      usleep( 1000 );
    }
  }

  bool LoopClosing::DetectLoop()
  {
    float         max_score     = 0.0;
    unsigned long best_match_id = 0;

    for ( const auto &db : key_frame_database_ )
    {
      if ( current_keyframe_->key_frame_id_ - db.second->key_frame_id_ < 20 )
        break;
      if ( !current_keyframe_->bow2_vec_.empty() && !db.second->bow2_vec_.empty() )
      {
        float similarity_score =
            dbow2_vocabulary_->score( current_keyframe_->bow2_vec_, db.second->bow2_vec_ );
        if ( similarity_score > max_score )
        {
          max_score     = similarity_score;
          best_match_id = db.first;
        }
      }
    }

    if ( max_score < loop_threshold_heigher_ )
    {
      return false;
    }

    loop_keyframe_ = key_frame_database_.at( best_match_id );

    INFO( lk_vio::logger, "----------------------------------------------------" );
    INFO( lk_vio::logger, "LoopClosing: find potential Candidate KF. Score is {0} Loop Keyframe ID: {1}", max_score, loop_keyframe_->key_frame_id_ );


    return true;
  }

  bool LoopClosing::MatchFeatures()
  {
    std::vector<cv::DMatch> matches;
    cv_matcher_->match( loop_keyframe_->ORBDescriptors_, current_keyframe_->ORBDescriptors_, matches );

    auto   min_max      = std::minmax_element( matches.begin(), matches.end(), []( const cv::DMatch &m1, const cv::DMatch &m2 ) { return m1.distance < m2.distance; } );
    double min_distance = min_max.first->distance;

    set_valid_feature_matches_.clear();
    for ( const auto &mt : matches )
    {
      if ( mt.distance <= std::max( 2 * min_distance, 30.0 ) )
      {
        /// class_id indicates the number of feature points, the pyramid is removed, and the class_id of the same pyramid level is the same
        int loop_feature_id = loop_keyframe_->pyramid_key_points_[ mt.queryIdx ].class_id;
        int current_feature_id =
            current_keyframe_->pyramid_key_points_[ mt.trainIdx ].class_id;

        /// the matches of keypoints belonging to the same feature pair shouldn't be inserted into the valid matches twice
        if ( set_valid_feature_matches_.find( { current_feature_id, loop_feature_id } ) != set_valid_feature_matches_.end() )
        {
          continue;
        }
        set_valid_feature_matches_.insert( { current_feature_id, loop_feature_id } );
      }
    }

    INFO( lk_vio::logger, "LoopClosing: number of valid feature matches: {0}", set_valid_feature_matches_.size() );

    if ( set_valid_feature_matches_.size() < 10 )
    {
      return false;
    }

    return true;
  }

  bool LoopClosing::ComputeCorrectPose()
  {
    std::vector<cv::Point3f> loop_point3f;
    std::vector<cv::Point2f> current_point2f;
    std::vector<cv::DMatch>  matches_with_mappoint;

    for ( auto iter = set_valid_feature_matches_.begin(); iter != set_valid_feature_matches_.end(); )
    {
      int           current_feature_id = iter->first;
      int           loop_feature_id    = iter->second;
      MapPoint::Ptr mp                 = loop_keyframe_->features_left_[ loop_feature_id ]->map_point_.lock();
      if ( mp )
      {
        Eigen::Vector3d p = mp->getPosition();
        current_point2f.push_back( current_keyframe_->features_left_[ current_feature_id ]->kp_position_.pt );
        loop_point3f.push_back( cv::Point3f( p( 0 ), p( 1 ), p( 2 ) ) );
        // useful if needs to draw the matches
        cv::DMatch valid_match( current_feature_id, loop_feature_id, 10.0 );
        matches_with_mappoint.push_back( valid_match );
        iter++;
      }
      else
      {
        iter = set_valid_feature_matches_.erase( iter );
      }
    }

    INFO( lk_vio::logger, "LoopClosing: number of valid matches with mappoints: {0}", loop_point3f.size() );


    if ( show_loop_closing_result_ )
    {
      //  show the match result
      // cv::Mat img_goodmatch;
      // cv::drawMatches( current_keyframe_->image_left_,
      //                  current_keyframe_->GetKeyPoints(),
      //                  loop_keyframe_->image_left_,
      //                  loop_keyframe_->GetKeyPoints(),
      //                  matches_with_mappoint,
      //                  img_goodmatch );
      // cv::resize( img_goodmatch, img_goodmatch, cv::Size(), 0.5, 0.5 );
      // cv::imshow( "valid matches with mappoints", img_goodmatch );
      // cv::waitKey( 1 );
    }

    if ( loop_point3f.size() < 10 )
    {
      return false;
    }

    cv::Mat rvec, tvec, R, K;
    cv::eigen2cv( left_camera_->getK(), K );
    Eigen::Matrix3d Reigen;
    Eigen::Vector3d teigen;
    
    // TODO: change to MLPnP like ORB-SLAM3
    try
    {
      cv::solvePnPRansac( loop_point3f, current_point2f, K, cv::Mat(), rvec, tvec, false, 100, 5.991, 0.99 );
    }
    catch ( const std::exception &e )
    {
      ERROR( lk_vio::logger, "LoopClosing: cv::solvePnPRansac() failed with exception: {0}", e.what() );
      return false;
    }
    catch ( ... )
    {
      return false;
    }

    cv::Rodrigues( rvec, R );
    cv::cv2eigen( R, Reigen );
    cv::cv2eigen( tvec, teigen );

    corrected_current_pose_ = Sophus::SE3d( Reigen, teigen );

    int cnt_inliner = OptimizeCurrentPose();
    if ( cnt_inliner < 10 )
    {
      return false;
    }

    INFO( lk_vio::logger, "LoopClosing: number of match inliers (after optimization): {0}", cnt_inliner );

    double error = ( current_keyframe_->getPose() * corrected_current_pose_.inverse() ).log().norm();

    if ( error > 1.0f && error < 15.0f )
    {
      WARN( lk_vio::logger, "Loop Error: {0} --> Need To Correct Loop Pose", error );
      need_correct_loop_pose_ = true;
    }
    else
    {
      INFO( lk_vio::logger, "Loop Error: {0} --> No Need to Correct Loop Pose.", error );
      need_correct_loop_pose_ = false;
    }

    current_keyframe_->loop_key_frame_           = loop_keyframe_;
    current_keyframe_->relative_pose_to_loop_KF_ = corrected_current_pose_ * loop_keyframe_->getPose().inverse();

    last_closed_keyframe_ = current_keyframe_;

    return true;
  }

  int LoopClosing::OptimizeCurrentPose()
  {
    auto to_vec2 = []( cv::Point2f &pt ) { return Eigen::Vector2d( pt.x, pt.y ); };

    typedef g2o::BlockSolver_6_3                                      BlockerSolverType;
    typedef g2o::LinearSolverDense<BlockerSolverType::PoseMatrixType> LinerrSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg( std::make_unique<BlockerSolverType>( std::make_unique<LinerrSolverType>() ) );

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    VertexPose *vertex_pose = new VertexPose();
    vertex_pose->setId( 0 );
    vertex_pose->setEstimate( corrected_current_pose_ );
    optimizer.addVertex( vertex_pose );

    int             index       = 1;
    Eigen::Matrix3d left_cam_K  = left_camera_->getK();
    Eigen::Matrix3d right_cam_K = right_camera_->getK();

    std::vector<EdgeProjectionPoseOnly *> edges;
    edges.reserve( set_valid_feature_matches_.size() );
    std::vector<bool> edge_is_outlier;
    edge_is_outlier.reserve( set_valid_feature_matches_.size() );
    std::vector<std::pair<int, int>> matches;
    matches.reserve( set_valid_feature_matches_.size() );

    for ( auto &match : set_valid_feature_matches_ )
    {
      int current_feature_id = match.first;
      int loop_feature_id    = match.second;

      Feature::Ptr    feat    = current_keyframe_->features_left_[ current_feature_id ];
      Eigen::Vector2d observe = to_vec2( feat->kp_position_.pt );
      Eigen::Vector3d pt3d    = loop_keyframe_->features_left_[ loop_feature_id ]->map_point_.lock()->getPosition();

      EdgeProjectionPoseOnly *edge = nullptr;
      if ( feat->is_on_left_frame_ )
      {
        edge = new EdgeProjectionPoseOnly( pt3d, left_cam_K );
      }
      else
      {
        edge = new EdgeProjectionPoseOnly( pt3d, right_cam_K );
      }

      edge->setId( index );
      edge->setVertex( 0, vertex_pose );
      edge->setInformation( Eigen::Matrix2d::Identity() );
      edge->setMeasurement( observe );
      edge->setRobustKernel( new g2o::RobustKernelHuber );
      optimizer.addEdge( edge );

      edges.emplace_back( edge );
      edge_is_outlier.emplace_back( false );
      matches.emplace_back( match );
      index++;
    }

    // estimate the Pose and determine the outliers
    // start optimization
    const double chi2_th        = 5.991;
    int          cnt_outliers   = 0;
    int          num_iterations = 4;

    optimizer.initializeOptimization();
    optimizer.optimize( 10 );

    for ( int iter = 0; iter < num_iterations; iter++ )
    {
      optimizer.initializeOptimization();
      optimizer.optimize( 10 );
      cnt_outliers = 0;
      for ( size_t i = 0; i < edges.size(); i++ )
      {
        auto edge = edges[ i ];
        if ( edge_is_outlier[ i ] )
        {
          edge->computeError();
        }
        if ( edge->chi2() > chi2_th )
        {
          edge_is_outlier[ i ] = true;
          edge->setLevel( 1 );
          cnt_outliers++;
        }
        else
        {
          edge->setLevel( 0 );
          edge_is_outlier[ i ] = false;
        }
        if ( iter == num_iterations - 2 )
        {
          edge->setRobustKernel( nullptr );
        }
      }
    }

    for ( size_t i = 0; i < edge_is_outlier.size(); i++ )
    {
      if ( edge_is_outlier[ i ] )
      {
        set_valid_feature_matches_.erase( matches[ i ] );
      }
    }


    std::stringstream ss;
    ss << "before g2o corrected_current_pose_: " << corrected_current_pose_.translation().transpose();
    INFO( lk_vio::logger, ss.str() );

    corrected_current_pose_ = vertex_pose->estimate();

    ss.str( "" );
    ss << "after g2o corrected_current_pose_: " << corrected_current_pose_.translation().transpose();
    INFO( lk_vio::logger, ss.str() );


    return set_valid_feature_matches_.size();
  }

  void LoopClosing::LoopCorrect()
  {
    if ( !need_correct_loop_pose_ )
    {
      WARN( lk_vio::logger, "Loop closing, No need correct poses" );
      return;
    }

    auto backend = backend_.lock();
    backend->RequestPause();
    while ( backend->IfHasPaused() )
    {
      usleep( 1e3 );
    }

    CorrectActivateKeyframeAndMappoint();
    INFO( lk_vio::logger, "Start Loop Cloeing Process Has Finished" );


    PoseGraphOptimization();

    backend->Resume();

    INFO( lk_vio::logger, "End Loop Cloeing Process Has Finished" );
    INFO( lk_vio::logger, "-------------------------------------------" );
    return;
  }

  void LoopClosing::CorrectActivateKeyframeAndMappoint()
  {
    std::unique_lock<std::mutex> lck( map_->mmutex_map_update_ );

    std::unordered_map<unsigned long, Sophus::SE3d> correct_activate_pose;

    correct_activate_pose.insert(
        { current_keyframe_->key_frame_id_, corrected_current_pose_ } );

    for ( auto &kf : map_->GetActiveKeyFrames() )
    {
      unsigned long kf_id = kf.first;
      if ( kf_id == current_keyframe_->key_frame_id_ )
      {
        continue;
      }
      /// T_{{k-n}_k} = T_{{k-n}_w} * T_{w_k}
      /// Get the relative pose between two keyframes, the relative pose is constant
      Sophus::SE3d T_kn_k = kf.second->getPose() * current_keyframe_->getPose().inverse();
      /// T_a = T_{{k-n}_k} * T_{kt_w}
      /// Calculate the correct pose of the key frame based on the correct pose of the current frame
      Sophus::SE3d T_kn_true = T_kn_k * corrected_current_pose_;
      correct_activate_pose.insert( { kf_id, T_kn_true } );
    }

    /// correct the active mappoints' positions
    for ( auto &mappoint : map_->GetActiveMapPoints() )
    {
      MapPoint::Ptr mp = mappoint.second;
      assert( mp->GetActiveObservations().empty() == false );
      /// correct the mappoint's position according to the corrected pose of active KF which first observes it
      /// and the relative position between them
      auto feat       = mp->GetActiveObservations().front().lock();
      auto observe_kf = feat->keyframe_.lock();

      // assert(correct_activate_pose.find(observe_kf->key_frame_id_) !=
      //        correct_activate_pose.end());
      if ( correct_activate_pose.find( observe_kf->key_frame_id_ ) == correct_activate_pose.end() )
      {
        continue;
      }
      /// P_cam = T_cw * P_w
      Eigen::Vector3d pos_cam      = observe_kf->getPose() * mp->getPosition();
      Sophus::SE3d    correct_pose = correct_activate_pose.at( observe_kf->key_frame_id_ );
      mp->SetPosition( correct_pose.inverse() * pos_cam );
    }

    for ( auto &keyframe : map_->GetActiveKeyFrames() )
    {
      keyframe.second->SetPose( correct_activate_pose.at( keyframe.first ) );
    }

    /// replace the current KF's mappoints with loop KF's matched mappoints
    for ( auto iter = set_valid_feature_matches_.begin();
          iter != set_valid_feature_matches_.end();
          iter++ )
    {
      int  current_feature_id = iter->first;
      int  loop_feature_id    = iter->second;
      auto loop_mp            = loop_keyframe_->features_left_[ loop_feature_id ]->map_point_.lock();
      // assert(loop_mp != nullptr);
      auto current_mp =
          current_keyframe_->features_left_[ current_feature_id ]->map_point_.lock();

      if ( current_mp && loop_mp )
      {
        for ( auto &obs : current_mp->GetObservations() )
        {
          auto feat = obs.lock();
          loop_mp->AddObservation( feat );
          feat->map_point_ = loop_mp;
        }
        map_->RemoveMapPoint( current_mp );
      }
      else
      {
        current_keyframe_->features_left_[ current_feature_id ]->map_point_ = loop_mp;
      }
    }

    return;
  }

  void LoopClosing::PoseGraphOptimization()
  {
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>          BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto                 solver = new g2o::OptimizationAlgorithmLevenberg( std::make_unique<BlockSolverType>( std::make_unique<LinearSolverType>() ) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    Map::KeyFramesType allKFs = map_->GetAllKeyFrames();

    // vertices
    std::map<unsigned long, VertexPose *> vertices_kf;
    for ( auto &keyframe : allKFs )
    {
      unsigned long kfId        = keyframe.first;
      auto          kf          = keyframe.second;
      VertexPose *  vertex_pose = new VertexPose();
      vertex_pose->setId( kf->key_frame_id_ );
      vertex_pose->setEstimate( kf->getPose() );
      vertex_pose->setMarginalized( false );

      auto mapActiveKFs = map_->GetActiveKeyFrames();
      // active KFs, loop KF, initial KF are fixed
      if ( mapActiveKFs.find( kfId ) != mapActiveKFs.end()  //
           || ( kfId == loop_keyframe_->key_frame_id_ ) || kfId == 0 )
      {
        vertex_pose->setFixed( true );
      }

      optimizer.addVertex( vertex_pose );
      vertices_kf.insert( { kf->key_frame_id_, vertex_pose } );
    }

    // edges
    int                            index = 0;
    std::map<int, EdgePoseGraph *> vEdges;
    for ( auto &keyframe : allKFs )
    {
      unsigned long kfId = keyframe.first;
      assert( vertices_kf.find( kfId ) != vertices_kf.end() );
      auto kf = keyframe.second;

      // edge type 1: edge between two KFs adjacent in time
      auto lastKF = kf->last_key_frame_.lock();
      if ( lastKF )
      {
        EdgePoseGraph *edge = new EdgePoseGraph();
        edge->setId( index );
        edge->setVertex( 0, vertices_kf.at( kfId ) );
        edge->setVertex( 1, vertices_kf.at( lastKF->key_frame_id_ ) );
        edge->setMeasurement( kf->relative_pose_to_last_KF_ );
        edge->setInformation( Eigen::Matrix<double, 6, 6>::Identity() );
        optimizer.addEdge( edge );
        vEdges.insert( { index, edge } );
        index++;
      }
      // edge type 2: loop edge
      auto loopKF = kf->loop_key_frame_.lock();
      if ( loopKF )
      {
        EdgePoseGraph *edge = new EdgePoseGraph();
        edge->setId( index );
        edge->setVertex( 0, vertices_kf.at( kfId ) );
        edge->setVertex( 1, vertices_kf.at( loopKF->key_frame_id_ ) );
        edge->setMeasurement( kf->relative_pose_to_loop_KF_ );
        edge->setInformation( Eigen::Matrix<double, 6, 6>::Identity() );
        optimizer.addEdge( edge );
        vEdges.insert( { index, edge } );
        index++;
      }
    }

    // do the optimization
    optimizer.initializeOptimization();
    optimizer.optimize( 20 );

    // correct the KFs' poses
    // correct all mappoints positions according to the KF which first observes it
    {  // mutex
      // avoid the conflict between frontend tracking and loopclosing correction
      std::unique_lock<std::mutex> lck( map_->mmutex_map_update_ );

      // set the mappoints' positions according to its first observing KF's optimized pose
      auto allMapPoints    = map_->GetAllMapPoints();
      auto activeMapPoints = map_->GetActiveMapPoints();
      for ( auto iter = activeMapPoints.begin(); iter != activeMapPoints.end(); iter++ )
      {
        allMapPoints.erase( ( *iter ).first );
      }
      for ( auto &mappoint : allMapPoints )
      {
        MapPoint::Ptr mp = mappoint.second;

        assert( !mp->GetObservations().empty() );

        auto feat        = mp->GetObservations().front().lock();
        auto observingKF = feat->keyframe_.lock();
        if ( vertices_kf.find( observingKF->key_frame_id_ ) == vertices_kf.end() )
        {
          // NOTICE: this is for the case that one mappoint is inserted into map in frontend thread
          // but the KF which first observes it hasn't been inserted into map in backend thread
          continue;
        }
        Eigen::Vector3d posCamera = observingKF->getPose() * mp->getPosition();

        Sophus::SE3d T_optimized = vertices_kf.at( observingKF->key_frame_id_ )->estimate();
        mp->SetPosition( T_optimized.inverse() * posCamera );
      }

      unsigned long frontend_ref_kf_id = frontend_->getReferenceKF()->key_frame_id_;
      // set the KFs' optimized poses
      for ( auto &v : vertices_kf )
      {
        if ( v.first == frontend_ref_kf_id )
        {
          //   Frame::Ptr front_curr_frame = frontend_->getCurrentFrame();
          //   Frame::Ptr front_last_frame = frontend_->getLastFrame();

          //   Sophus::SE3d ref_kf_new_pose = v.second->estimate();
          //   Sophus::SE3d ref_kf_old_pose = frontend_->getReferenceKF()->getPose();
          //   Sophus::SE3d T_k_old_new = ref_kf_old_pose.inverse() * ref_kf_new_pose;

          //   Sophus::SE3d new_current_frame_rel_pose = front_curr_frame->getRelativePose() * T_k_old_new;
          //   front_curr_frame->SetRelativePose(new_current_frame_rel_pose);
          //   Sophus::SE3d new_last_frame_rel_pose = front_last_frame->getRelativePose() * T_k_old_new;
          //   front_last_frame->SetRelativePose(new_last_frame_rel_pose);
          continue;
        }
        allKFs.at( v.first )->SetPose( v.second->estimate() );
      }

    }  // mutex

    return;
  }

  void LoopClosing::ProcessNewKeyframe()
  {
    {
      std::unique_lock<std::mutex> lck( all_new_keyframe_mutex_ );
      current_keyframe_ = all_new_keyframes_.front();

      all_new_keyframes_.pop_front();
    }

    std::vector<cv::KeyPoint> pyramid_points;
    pyramid_points.reserve( pyramid_level_num_ * current_keyframe_->features_left_.size() );
    for ( size_t i = 0; i < current_keyframe_->features_left_.size(); i++ )
    {
      /// The class id marks the serial number
      current_keyframe_->features_left_[ i ]->kp_position_.class_id = i;
      for ( int level = 0; level < pyramid_level_num_; level++ )
      {
        cv::KeyPoint kp( current_keyframe_->features_left_[ i ]->kp_position_ );
        kp.octave   = level;
        kp.response = -1;
        kp.class_id = i;
        pyramid_points.emplace_back( kp );
      }
    }
    // remove the pyramid keypoints which are not FAST corner or beyond borders
    // compute their orientations and sizes
    orb_extractor_->ScreenAndComputeKPsParams( current_keyframe_->image_left_,
                                               pyramid_points,
                                               current_keyframe_->pyramid_key_points_ );

    // calculate the orb descriptors of all valid pyramid keypoints
    orb_extractor_->CalcDescriptors( current_keyframe_->image_left_,
                                     current_keyframe_->pyramid_key_points_,
                                     current_keyframe_->ORBDescriptors_ );
    std::vector<cv::Mat> desc =
        ConvertToDescriptorVector( current_keyframe_->ORBDescriptors_ );

    dbow2_vocabulary_->transform( desc, current_keyframe_->bow2_vec_ );
  }

  void LoopClosing::StopLoopClosing()
  {
    while ( CheckNewKeyFrame() )
    {
      usleep( 1e5 );
    }
    loop_thread_is_running_.store( false );
    loop_closing_thread_.join();
  }

  void LoopClosing::AddToKeyframeDatabase()
  {
    key_frame_database_.insert( { current_keyframe_->key_frame_id_, current_keyframe_ } );
  }

  bool LoopClosing::CheckNewKeyFrame()
  {
    std::unique_lock<std::mutex> lck( all_new_keyframe_mutex_ );
    return ( !all_new_keyframes_.empty() );
  }

  void LoopClosing::InsertNewKeyFrame( const std::shared_ptr<KeyFrame> new_kf )
  {
    std::unique_lock<std::mutex> lck( all_new_keyframe_mutex_ );
    if ( last_closed_keyframe_ == nullptr ||
         new_kf->key_frame_id_ - last_closed_keyframe_->key_frame_id_ > 5 )
    {
      all_new_keyframes_.push_back( new_kf );
    }
    else
    {
      // TODO
    }
  }

  std::vector<cv::Mat> LoopClosing::ConvertToDescriptorVector( const cv::Mat &descriptors )
  {
    assert( descriptors.rows > 0 );
    std::vector<cv::Mat> desc;
    desc.reserve( descriptors.rows );
    for ( int j = 0; j < descriptors.rows; j++ )
    {
      desc.push_back( descriptors.row( j ) );
    }
    return desc;
  }

  void LoopClosing::SetSteroCamera( std::shared_ptr<Camera> left,
                                    std::shared_ptr<Camera> right )
  {
    assert( left != nullptr && right != nullptr );
    left_camera_  = left;
    right_camera_ = right;
  }

  void LoopClosing::GenerateORBextractor( const common::Configuration &config )
  {
    int   num_orb_bew_features = config.extractor_params.track_features_num;
    float scale_factor         = config.extractor_params.scale_factor;
    int   n_levels             = config.extractor_params.pyramid_level;
    int   fIniThFAST           = config.extractor_params.init_fast_threshold;
    int   fMinThFAST           = config.extractor_params.min_fast_threshold;

    INFO( lk_vio::logger, "---- ---- LoopClosing: ORBextractor Parameters ---- ----" );
    INFO( lk_vio::logger, "Number of ORB features: {0}", num_orb_bew_features );
    INFO( lk_vio::logger, "Scale factor: {0}", scale_factor );
    INFO( lk_vio::logger, "Number of pyramid levels: {0}", n_levels );
    INFO( lk_vio::logger, "Initial FAST threshold: {0}", fIniThFAST );
    INFO( lk_vio::logger, "Minimum FAST threshold: {0}", fMinThFAST );
    orb_extractor_ = ORBextractor::Ptr( new ORBextractor( num_orb_bew_features, scale_factor, n_levels, fIniThFAST, fMinThFAST ) );
  }

  void LoopClosing::LoadParam( const common::Configuration &config )
  {
    open_loop_closing_        = config.loop_closure_params.activate;
    show_loop_closing_result_ = config.loop_closure_params.show_result;
    loop_threshold_heigher_   = config.loop_closure_params.max_threshold;
    loop_threshold_lower_     = config.loop_closure_params.min_threshold;
    pyramid_level_num_        = config.loop_closure_params.pyramid_level;

    INFO( lk_vio::logger, "---- ---- LoopClosing Parameters ---- ----" );
    INFO( lk_vio::logger, "Loop.Closing.Open: {0}", open_loop_closing_ );
    INFO( lk_vio::logger, "Loop.Show.Closing.Result: {0}", show_loop_closing_result_ );
    INFO( lk_vio::logger, "Loop.Threshold.Heigher: {0}", loop_threshold_heigher_ );
    INFO( lk_vio::logger, "Loop.Threshold.Lower: {0}", loop_threshold_lower_ );
    INFO( lk_vio::logger, "Pyramid.Level: {0}", pyramid_level_num_ );
    INFO( lk_vio::logger, "Loop.Closig.Keyframe.Database.Min.Size: {0}", keyframe_database_min_size_ );
  }


}  // namespace lk_vio
