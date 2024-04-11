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

class CameraDriver
{
public:
  CameraDriver( const int device_id, const int frame_rate, const int width, const int height )
      : device_id( device_id ), frame_rate( frame_rate ), width( width ), height( height )
  {
    this->initialize();
  }
  CameraDriver( const std::string& device_name, const int frame_rate, const int width, const int height )
      : device_name( device_name ), frame_rate( frame_rate ), width( width ), height( height )
  {
  }

  ~CameraDriver()
  {
    if ( this->cap.isOpened() )
    {
      this->cap.release();
    }
  }

  void initialize()
  {
    // std::cout << "Initialize Camera Driver" << std::endl;
    // std::cout << "Device ID: " << this->device_id << std::endl;
    // std::cout << "Frame Rate: " << this->frame_rate << std::endl;
    // std::cout << "Width: " << this->width << std::endl;
    // std::cout << "Height: " << this->height << std::endl;

    if ( this->device_name.empty() )
    {
      std::cout << "Initialize Camera Driver with Device ID: " << this->device_id << std::endl;
      this->cap.open( this->device_id );
    }
    else
    {
      std::cout << "Initialize Camera Driver with Device Name: " << this->device_name << std::endl;
      this->cap.open( this->device_name );
    }

    if ( !this->cap.isOpened() )
    {
      std::cout << "Error opening video stream or file" << std::endl;
      exit( -1 );
    }

    this->cap.set( cv::CAP_PROP_FPS, this->frame_rate );
    this->cap.set( cv::CAP_PROP_FRAME_WIDTH, this->width * 2 );
    this->cap.set( cv::CAP_PROP_FRAME_HEIGHT, this->height );

    this->cap.set( cv::CAP_PROP_BUFFERSIZE, 1 );
    this->cap.set( cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc( 'M', 'J', 'P', 'G' ) );
  }

  void getStereoFrame( cv::Mat& frame_left, cv::Mat& frame_right )
  {
    cv::Mat frame;
    this->cap >> frame;
    cv::cvtColor( frame, frame, cv::COLOR_BGR2GRAY );


    frame_left  = frame( cv::Rect( 0, 0, this->width, this->height ) );
    frame_right = frame( cv::Rect( this->width, 0, this->width, this->height ) );
  }

private:
  int         device_id;
  std::string device_name;
  int         frame_rate;
  int         width;
  int         height;

  cv::VideoCapture cap;
};

int main( int argc, char** argv )
{
  if ( argc < 2 )
  {
    printf( "Usage: %s <config_file.json>\n", argv[ 0 ] );
    return -1;
  }

  ros::init( argc, argv, "lk_vio", ros::init_options::NoSigintHandler );

  lk_vio::common::ParamServer& config = lk_vio::common::ParamServer::getInstance();
  config.readConfigFile( argv[ 1 ] );

  lk_vio::initLogger( config.log_data_path );

  common::Timer               timer;
  common::RunningStat<double> running_stat;

  CameraDriver camera_driver( 2, 30, config.camera_params.width, config.camera_params.height );
  /// Init SLAM System
  lk_vio::System system( config );

  std::size_t count{ 0 };
  while ( true )
  {
    timer.tic();

    auto   now       = std::chrono::system_clock::now();
    double timestamp = std::chrono::duration<double>( now.time_since_epoch() ).count();

    if ( count % 100 == 99 )
    {
      INFO( lk_vio::logger, "Has processed {0} frames.", count + 1 );
      INFO( lk_vio::logger, "Time Comsumed: {0} ms, Average Time Consumed: {1} ms", timer.time_consumed_ms_double, running_stat.getMean() );
    }


    cv::Mat frame_left, frame_right;
    camera_driver.getStereoFrame( frame_left, frame_right );

    if ( frame_left.empty() || frame_right.empty() )
    {
      WARN( lk_vio::logger, "No frame received, skip this frame." );
      continue;
    }

    system.RunStep( frame_left, frame_right, timestamp );
    timer.toc();


    running_stat.addValue( timer.time_consumed_ms_double );

    count++;
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }

  return 0;
}