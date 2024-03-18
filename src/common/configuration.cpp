#include "common/configuration.hpp"

namespace common
{
void Configuration::readConfigFile( const std::string &config_file_path )
{
  std::ifstream  config_file( config_file_path );
  nlohmann::json config;
  config_file >> config;
  config_file.close();

  // Path related parameters
  data_set_path   = config[ "path" ][ "data_set_path" ];
  log_data_path   = config[ "path" ][ "log_data_path" ];
  trajectory_path = config[ "path" ][ "trajectory_path" ];
  dbow2_voc_path  = config[ "path" ][ "dbow2_voc_path" ];
  std::cout << "---- ---- path: ---- ---- \n";
  std::cout << "data_set_path: " << data_set_path << "\n";
  std::cout << "log_data_path: " << log_data_path << "\n";
  std::cout << "trajectory_path: " << trajectory_path << "\n";
  std::cout << "dbow2_voc_path: " << dbow2_voc_path << "\n\n";

  // Camera parameters
  left_camera_intrinsics.fx = config[ "camera" ][ "left" ][ "fx" ];
  left_camera_intrinsics.fy = config[ "camera" ][ "left" ][ "fy" ];
  left_camera_intrinsics.cx = config[ "camera" ][ "left" ][ "cx" ];
  left_camera_intrinsics.cy = config[ "camera" ][ "left" ][ "cy" ];
  left_camera_intrinsics.k1 = config[ "camera" ][ "left" ][ "k1" ];
  left_camera_intrinsics.k2 = config[ "camera" ][ "left" ][ "k2" ];
  left_camera_intrinsics.p1 = config[ "camera" ][ "left" ][ "p1" ];
  left_camera_intrinsics.p2 = config[ "camera" ][ "left" ][ "p2" ];
  left_camera_intrinsics.k3 = config[ "camera" ][ "left" ][ "k3" ];
  std::cout << "---- ---- left_camera_intrinsics: ---- ---- \n";
  std::cout << "fx: " << left_camera_intrinsics.fx << "\n";
  std::cout << "fy: " << left_camera_intrinsics.fy << "\n";
  std::cout << "cx: " << left_camera_intrinsics.cx << "\n";
  std::cout << "cy: " << left_camera_intrinsics.cy << "\n";
  std::cout << "k1: " << left_camera_intrinsics.k1 << "\n";
  std::cout << "k2: " << left_camera_intrinsics.k2 << "\n";
  std::cout << "p1: " << left_camera_intrinsics.p1 << "\n";
  std::cout << "p2: " << left_camera_intrinsics.p2 << "\n";
  std::cout << "k3: " << left_camera_intrinsics.k3 << "\n";

  right_camera_intrinsics.fx = config[ "camera" ][ "right" ][ "fx" ];
  right_camera_intrinsics.fy = config[ "camera" ][ "right" ][ "fy" ];
  right_camera_intrinsics.cx = config[ "camera" ][ "right" ][ "cx" ];
  right_camera_intrinsics.cy = config[ "camera" ][ "right" ][ "cy" ];
  right_camera_intrinsics.k1 = config[ "camera" ][ "right" ][ "k1" ];
  right_camera_intrinsics.k2 = config[ "camera" ][ "right" ][ "k2" ];
  right_camera_intrinsics.p1 = config[ "camera" ][ "right" ][ "p1" ];
  right_camera_intrinsics.p2 = config[ "camera" ][ "right" ][ "p2" ];
  right_camera_intrinsics.k3 = config[ "camera" ][ "right" ][ "k3" ];
  std::cout << "---- ---- right_camera_intrinsics: ---- ---- \n";
  std::cout << "fx: " << right_camera_intrinsics.fx << "\n";
  std::cout << "fy: " << right_camera_intrinsics.fy << "\n";
  std::cout << "cx: " << right_camera_intrinsics.cx << "\n";
  std::cout << "cy: " << right_camera_intrinsics.cy << "\n";
  std::cout << "k1: " << right_camera_intrinsics.k1 << "\n";
  std::cout << "k2: " << right_camera_intrinsics.k2 << "\n";
  std::cout << "p1: " << right_camera_intrinsics.p1 << "\n";
  std::cout << "p2: " << right_camera_intrinsics.p2 << "\n";
  std::cout << "k3: " << right_camera_intrinsics.k3 << "\n";

  camera_params.left              = left_camera_intrinsics;
  camera_params.right             = right_camera_intrinsics;
  camera_params.baseline          = config[ "camera" ][ "baseline" ];
  camera_params.width             = config[ "camera" ][ "width" ];
  camera_params.height            = config[ "camera" ][ "height" ];
  camera_params.need_undistortion = config[ "camera" ][ "need_undistortion" ];
  std::cout << "---- ---- camera_params: ---- ---- \n";
  std::cout << "baseline: " << camera_params.baseline << "\n";
  std::cout << "width: " << camera_params.width << "\n";
  std::cout << "height: " << camera_params.height << "\n";
  std::cout << "need_undistortion: " << camera_params.need_undistortion << "\n\n";

  // // Map parameters
  map_params.active_map_size    = config[ "map" ][ "active_map_size" ];
  map_params.init_landmark_size = config[ "map" ][ "init_landmark_size" ];
  std::cout << "---- ---- map_params: ---- ---- \n";
  std::cout << "active_map_size: " << map_params.active_map_size << "\n";
  std::cout << "init_landmark_size: " << map_params.init_landmark_size << "\n\n";

  // Tracking status parameters
  tracking_status_params.init_good  = config[ "tracking_status" ][ "init_good" ];
  tracking_status_params.track_good = config[ "tracking_status" ][ "track_good" ];
  tracking_status_params.track_bad  = config[ "tracking_status" ][ "track_bad" ];
  std::cout << "---- ---- tracking_status_params: ---- ---- \n";
  std::cout << "init_good: " << tracking_status_params.init_good << "\n";
  std::cout << "track_good: " << tracking_status_params.track_good << "\n";
  std::cout << "track_bad: " << tracking_status_params.track_bad << "\n\n";

  // Extractor parameters
  extractor_params.init_features_num   = config[ "extractor" ][ "init_features_num" ];
  extractor_params.track_features_num  = config[ "extractor" ][ "track_features_num" ];
  extractor_params.scale_factor        = config[ "extractor" ][ "scale_factor" ];
  extractor_params.pyramid_level       = config[ "extractor" ][ "pyramid_level" ];
  extractor_params.init_fast_threshold = config[ "extractor" ][ "init_fast_threshold" ];
  extractor_params.min_fast_threshold  = config[ "extractor" ][ "min_fast_threshold" ];
  std::cout << "---- ---- extractor_params: ---- ---- \n";
  std::cout << "init_features_num: " << extractor_params.init_features_num << "\n";
  std::cout << "track_features_num: " << extractor_params.track_features_num << "\n";
  std::cout << "scale_factor: " << extractor_params.scale_factor << "\n";
  std::cout << "pyramid_level: " << extractor_params.pyramid_level << "\n";
  std::cout << "init_fast_threshold: " << extractor_params.init_fast_threshold << "\n";
  std::cout << "min_fast_threshold: " << extractor_params.min_fast_threshold << "\n\n";

  // Back-end parameters
  back_end_params.activate = config[ "back_end" ][ "activate" ];
  std::cout << "---- ---- back_end_params: ---- ---- \n";
  std::cout << "activate: " << back_end_params.activate << "\n\n";

  // Loop closure parameters
  loop_closure_params.activate          = config[ "loop_closure" ][ "activate" ];
  loop_closure_params.show_result       = config[ "loop_closure" ][ "show_result" ];
  loop_closure_params.max_threshold     = config[ "loop_closure" ][ "max_threshold" ];
  loop_closure_params.min_threshold     = config[ "loop_closure" ][ "min_threshold" ];
  loop_closure_params.min_database_size = config[ "loop_closure" ][ "min_database_size" ];
  loop_closure_params.pyramid_level     = config[ "loop_closure" ][ "pyramid_level" ];
  std::cout << "---- ---- loop_closure_params: ---- ---- \n";
  std::cout << "activate: " << loop_closure_params.activate << "\n";
  std::cout << "show_result: " << loop_closure_params.show_result << "\n";
  std::cout << "max_threshold: " << loop_closure_params.max_threshold << "\n";
  std::cout << "min_threshold: " << loop_closure_params.min_threshold << "\n";
  std::cout << "min_database_size: " << loop_closure_params.min_database_size << "\n";
  std::cout << "pyramid_level: " << loop_closure_params.pyramid_level << "\n\n";

  // Viewer parameters
  viewer_params.show_extractor_result_cv  = config[ "viewer" ][ "show_extractor_result_cv" ];
  viewer_params.show_lk_matcher_result_cv = config[ "viewer" ][ "show_lk_matcher_result_cv" ];

  std::cout << "---- ---- viewer_params: ---- ---- \n";
  std::cout << "show_extractor_result_cv: " << viewer_params.show_extractor_result_cv << "\n";
  std::cout << "show_lk_matcher_result_cv: " << viewer_params.show_lk_matcher_result_cv << "\n\n";
}
}  // namespace common
