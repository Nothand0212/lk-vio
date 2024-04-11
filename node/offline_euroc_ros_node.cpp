#include <ros/ros.h>

#include <Eigen/Dense>
#include <thread>

#include "chrono"
#include "common/configuration.hpp"
#include "common/param_server.hpp"
#include "common/read_euroc_dataset.hpp"
#include "common/read_kitii_dataset.hpp"
#include "common/statisticsOnline.h"
#include "common/timer.hpp"
#include "lk_vio/system.hpp"
#include "logger/logger.h"


void printEscapedString( const std::string& input )
{
  for ( char c : input )
  {
    switch ( c )
    {
      case '\t':
        std::cout << "\\t";  // 打印制表符的转义形式
        break;
      case '\n':
        std::cout << "\\n";  // 打印换行符的转义形式
        break;
      // 其他转义字符的处理方式类似
      default:
        std::cout << c;  // 打印普通字符
    }
  }
}

using namespace spdlog;
int main( int argc, char** argv )
{
  if ( argc < 2 )
  {
    printf( "Usage: %s <config_file.json>\n", argv[ 0 ] );
    return -1;
  }

  ros::init( argc, argv, "offline_euroc_ros_node", ros::init_options::NoSigintHandler );
  ros::NodeHandle nh;

  lk_vio::common::ParamServer& config = lk_vio::common::ParamServer::getInstance();
  config.readConfigFile( argv[ 1 ] );

  lk_vio::initLogger( config.log_data_path );


  // ---- ---- Time Consumed Statistics ---- ---- //
  common::Timer               timer;
  common::RunningStat<double> running_stat;
  // ---- ---- Time Consumed Statistics ---- ---- //


  // Read Euroc dataset
  // std::string                  dataset_path = "/home/lin/Datasets/mav0";
  std::vector<std::string>     str_image_left_vec_path;
  std::vector<std::string>     str_image_right_vec_path;
  std::vector<Eigen::Vector3d> acc_vec;
  std::vector<Eigen::Vector3d> gyro_vec;
  std::vector<double>          image_time_vec;
  std::vector<double>          imu_time_vec;

  common::read_euroc_dataset( config.data_set_path, str_image_left_vec_path, str_image_right_vec_path, acc_vec, gyro_vec, image_time_vec, imu_time_vec );


  const size_t num_images = str_image_left_vec_path.size();
  INFO( lk_vio::logger, "Number of images: {0}", num_images );
  INFO( lk_vio::logger, "Number of image timestamps: {0}", image_time_vec.size() );
  INFO( lk_vio::logger, "Number of IMU timestamps: {0}", imu_time_vec.size() );
  INFO( lk_vio::logger, "Number of accelerometer measurements: {0}", acc_vec.size() );
  INFO( lk_vio::logger, "Number of gyroscope measurements: {0}", gyro_vec.size() );

  if ( ( str_image_left_vec_path.size() != str_image_right_vec_path.size() ) || ( str_image_left_vec_path.size() != image_time_vec.size() ) )
  {
    ERROR( lk_vio::logger, "The number of left and right images are not equal!" );
    ERROR( lk_vio::logger, "Number of left/right images: {0}/{1}", str_image_left_vec_path.size(), str_image_right_vec_path.size() );
    ERROR( lk_vio::logger, "Number of image timestamps: {0}", image_time_vec.size() );
    return -1;
  }

  /// Init SLAM System
  lk_vio::System system( config );

  for ( std::size_t ni = 0; ni < num_images; ni++ )
  {
    timer.tic();

    if ( ni % 100 == 99 )
    {
      INFO( lk_vio::logger, "Has processed {0} frames.", ni + 1 );
      INFO( lk_vio::logger, "Time Comsumed: {0} ms, Average Time Consumed: {1} ms", timer.time_consumed_ms_double, running_stat.getMean() );
    }

    INFO( lk_vio::logger, "left iamge:  {0}", str_image_left_vec_path[ ni ] );
    INFO( lk_vio::logger, "right iamge: {0}", str_image_right_vec_path[ ni ] );

    // printEscapedString( str_image_left_vec_path[ ni ] );
    // printEscapedString( str_image_right_vec_path[ ni ] );

    cv::Mat img_left  = cv::imread( str_image_left_vec_path[ ni ], cv::IMREAD_GRAYSCALE );
    cv::Mat img_right = cv::imread( str_image_right_vec_path[ ni ], cv::IMREAD_GRAYSCALE );
    // cv::Mat img_left        = cv::imread( "/home/lin/Datasets/mav0/cam0/data/1403636579763555584.png", cv::IMREAD_GRAYSCALE );
    // cv::Mat img_right       = cv::imread( "/home/lin/Datasets/mav0/cam1/data/1403636579763555584.png", cv::IMREAD_GRAYSCALE );
    double timestamp_image = image_time_vec[ ni ];

    if ( img_left.empty() || img_right.empty() )
    {
      ERROR( lk_vio::logger, "Failed to read left image: {0} - {1}", str_image_left_vec_path[ ni ], img_left.empty() );
      ERROR( lk_vio::logger, "Failed to read right image: {0} - {1}", str_image_right_vec_path[ ni ], img_right.empty() );
      std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
      return -1;
    }

    system.RunStep( img_left, img_right, timestamp_image );
    // usleep(1e4);
    timer.toc();
    running_stat.addValue( timer.time_consumed_ms_double );
  }


  while ( true )
  {
    std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ) );
  }
}