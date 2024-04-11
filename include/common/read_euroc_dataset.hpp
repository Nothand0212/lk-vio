#pragma once

#include <Eigen/Core>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
namespace common
{
  std::vector<std::string> splitString( const std::string &input, char delimiter )
  {
    std::vector<std::string> result;
    std::stringstream        ss( input );
    std::string              item;
    while ( std::getline( ss, item, delimiter ) )
    {
      result.push_back( item );
    }
    return result;
  }

  void read_euroc_dataset( const std::string &           str_path_to_sequence,
                           std::vector<std::string> &    str_image_left_vec_path,
                           std::vector<std::string> &    str_image_right_vec_path,
                           std::vector<Eigen::Vector3d> &acc_vec,
                           std::vector<Eigen::Vector3d> &gyro_vec,
                           std::vector<double> &         image_time_vec,
                           std::vector<double> &         imu_time_vec )
  {
    // TODO: Implement this function

    std::string str_path_to_imu_file  = str_path_to_sequence + "/imu0/data.csv";
    std::string str_path_to_cam0_file = str_path_to_sequence + "/cam0/data.csv";
    std::string str_path_to_cam1_file = str_path_to_sequence + "/cam1/data.csv";


    // Read camera data
    std::ifstream cam_0_file( str_path_to_cam0_file );
    std::string   temp_line;
    std::getline( cam_0_file, temp_line );  // Skip header line
    while ( std::getline( cam_0_file, temp_line ) )
    {
      // std::cout << " ---- ---- ---- ---- ----\n";
      // std::cout << "temp_line: " << temp_line << std::endl;

      auto        reuslt        = splitString( temp_line, ',' );
      std::string timestamp_str = reuslt[ 0 ];
      std::string image_path    = reuslt[ 1 ];
      if ( image_path[ image_path.size() - 1 ] == '\r' || image_path[ image_path.size() - 1 ] == '\n' )
      {
        image_path = image_path.substr( 0, image_path.size() - 1 );
      }

      // std::cout << "time str: " << timestamp_str << std::endl;
      // std::cout << "image path: " << image_path << std::endl;

      // std::cout << "time second: " << timestamp_str.substr( 0, 10 ) << std::endl;
      // std::cout << "time nano: " << timestamp_str.substr( 10, timestamp_str.size() ) << std::endl;
      // timestamps_vec.push_back( std::stoll( timestamp_str.substr( 0, 10 ) ) + std::stoll( timestamp_str.substr( 11, timestamp_str.size() ) ) / 1e8 );
      double time_1 = std::stold( timestamp_str ) * 1e-9;
      double time_2 = std::stold( timestamp_str.substr( 0, 10 ) ) + std::stold( timestamp_str.substr( 10, timestamp_str.size() ) ) * 1e-9;
      // std::cout << "time_1: " << time_1 << std::endl;
      // std::cout << "time_2: " << time_2 << std::endl;
      image_time_vec.push_back( time_1 );

      str_image_left_vec_path.push_back( str_path_to_sequence + "/cam0/data/" + image_path );
      str_image_right_vec_path.push_back( str_path_to_sequence + "/cam1/data/" + image_path );


      std::cout << "Timestamp: " << std::fixed << std::setprecision( 12 ) << image_time_vec.back() << std::endl;
      std::cout << "Image path: " << str_image_left_vec_path.back() << std::endl;
      std::cout << "Image path: " << str_image_right_vec_path.back() << std::endl;
    }

    // Read IMU data
    std::ifstream imu_file( str_path_to_imu_file );
    std::getline( imu_file, temp_line );  // Skip header line
    while ( std::getline( imu_file, temp_line ) )
    {
      // std::cout << " ---- ---- ----\n";
      // std::cout << "temp_line: " << temp_line << std::endl;
      auto        reuslt        = splitString( temp_line, ',' );
      std::string timestamp_str = reuslt[ 0 ];
      std::string angle_vel_x   = reuslt[ 1 ];
      std::string angle_vel_y   = reuslt[ 2 ];
      std::string angle_vel_z   = reuslt[ 3 ];
      std::string lin_acc_x     = reuslt[ 4 ];
      std::string lin_acc_y     = reuslt[ 5 ];
      std::string lin_acc_z     = reuslt[ 6 ];

      imu_time_vec.push_back( std::stold( timestamp_str ) * 1e-9 );
      acc_vec.push_back( Eigen::Vector3d( std::stod( lin_acc_x ), std::stod( lin_acc_y ), std::stod( lin_acc_z ) ) );
      gyro_vec.push_back( Eigen::Vector3d( std::stod( angle_vel_x ), std::stod( angle_vel_y ), std::stod( angle_vel_z ) ) );

      // std::cout << "Timestamp: " << std::fixed << std::setprecision( 12 ) << image_time_vec.back() << std::endl;
      // std::cout << "Acc: " << acc_vec.back().transpose() << std::endl;
      // std::cout << "Gyro: " << gyro_vec.back().transpose() << std::endl;
    }
  }
}  // namespace common