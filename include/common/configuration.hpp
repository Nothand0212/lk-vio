#pragma once

#include <fstream>
#include <iostream>
#include <string>

#include "nlohmann/json.hpp"

namespace common
{
struct CameraIntrinsics
{
  double fx{ 0.0f };
  double fy{ 0.0f };
  double cx{ 0.0f };
  double cy{ 0.0f };
  double k1{ 0.0f };
  double k2{ 0.0f };
  double p1{ 0.0f };
  double p2{ 0.0f };
  double k3{ 0.0f };
};

struct CameraParams
{
  CameraIntrinsics left;
  CameraIntrinsics right;

  float baseline;
  int   width;
  int   height;
  bool  need_undistortion;
};


struct MapParams
{
  int active_map_size;
  int init_landmark_size;
};

struct TrackingStatusParams
{
  int init_good;
  int track_good;
  int track_bad;
};

struct ExtractorParams
{
  int   init_features_num;
  int   track_features_num;
  float scale_factor;
  int   pyramid_level;
  int   init_fast_threshold;
  int   min_fast_threshold;
};

struct BackEndParams
{
  bool activate;
};

struct LoopClosureParams
{
  bool  activate;
  bool  show_result;
  float max_threshold;
  float min_threshold;
  int   min_database_size;
  int   pyramid_level;
};

struct ViewerParams
{
  bool show_extractor_result_cv;
  bool show_lk_matcher_result_cv;
};


class Configuration
{
public:
  std::string data_set_path;
  std::string trajectory_path;
  std::string dbow2_voc_path;
  // std::string setting_param_path;
  std::string log_data_path;

  CameraParams         camera_params;
  CameraIntrinsics     left_camera_intrinsics;
  CameraIntrinsics     right_camera_intrinsics;
  MapParams            map_params;
  TrackingStatusParams tracking_status_params;
  ExtractorParams      extractor_params;
  BackEndParams        back_end_params;
  LoopClosureParams    loop_closure_params;
  ViewerParams         viewer_params;

public:
  Configuration()  = default;
  ~Configuration() = default;
  void readConfigFile( const std::string &config_file_path );
};
}  // namespace common