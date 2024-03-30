#include <ros/ros.h>

#include <thread>

#include "chrono"
#include "common/configuration.hpp"
#include "common/param_server.hpp"
#include "common/read_kitii_dataset.hpp"
#include "common/statisticsOnline.h"
#include "common/timer.hpp"
#include "lk_vio/system.hpp"
#include "logger/logger.h"
using namespace spdlog;
int main( int argc, char** argv )
{
  if ( argc < 2 )
  {
    printf( "Usage: %s <config_file.json>\n", argv[ 0 ] );
    return -1;
  }

  ros::init( argc, argv, "lk_vio", ros::init_options::NoSigintHandler );


  // common::Configuration config;
  // config.readConfigFile( argv[ 1 ] );
  lk_vio::common::ParamServer& config = lk_vio::common::ParamServer::getInstance();
  config.readConfigFile( argv[ 1 ] );

  lk_vio::initLogger( config.log_data_path );

  common::Timer               timer;
  common::RunningStat<double> running_stat;

  /// load sequence frames
  std::vector<std::string> image_left_vec_path, image_right_vec_path;
  std::vector<double>      vec_timestamp;

  INFO( lk_vio::logger, "Loading KITTI dataset from: {0}", config.data_set_path );

  common::LoadKittiImagesTimestamps( config.data_set_path, image_left_vec_path, image_right_vec_path, vec_timestamp );

  const size_t num_images = image_left_vec_path.size();
  INFO( lk_vio::logger, "Num Images: {0}", num_images );

  if ( num_images != image_right_vec_path.size() || num_images != vec_timestamp.size() )
  {
    ERROR( lk_vio::logger, "The number of right images is {0}, the number of timestamps is {1}.", image_right_vec_path.size(), vec_timestamp.size() );
    return -1;
  }


  /// Init SLAM System
  lk_vio::System system( config );

  for ( int ni = 0; ni < num_images; ni++ )
  {
    // INFO( lk_vio::logger, "Processing frame {0}", ni + 1 );
    timer.tic();
    if ( ni % 100 == 99 )
    {
      INFO( lk_vio::logger, "Has processed {0} frames.", ni + 1 );
      INFO( lk_vio::logger, "Time Comsumed: {0} ms, Average Time Consumed: {1} ms", timer.time_consumed_ms_double, running_stat.getMean() );
    }

    INFO( lk_vio::logger, "left iamge {0}", image_left_vec_path[ ni ] );
    INFO( lk_vio::logger, "right iamge {0}", image_right_vec_path[ ni ] );

    cv::Mat img_left  = cv::imread( image_left_vec_path[ ni ], cv::IMREAD_GRAYSCALE );
    cv::Mat img_right = cv::imread( image_right_vec_path[ ni ], cv::IMREAD_GRAYSCALE );
    double  timestamp = vec_timestamp[ ni ];
    if ( img_left.empty() )
    {
      SPDLOG_LOGGER_ERROR( lk_vio::logger, "Failed to load image at: {0}", image_left_vec_path[ ni ] );
    }

    system.RunStep( img_left, img_right, timestamp );
    // usleep(1e4);
    timer.toc();
    running_stat.addValue( timer.time_consumed_ms_double );
    // std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }

  while ( true )
  {
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }


  // system.getViewUi()->SaveTrajectoryAsTUM();
  // while ( !system.getViewUi()->ShouldQuit() )
  // {
  //   // std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  //   ;
  // }
  return 0;
}