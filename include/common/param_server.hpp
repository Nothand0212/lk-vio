#pragma once

#include <fstream>
#include <iostream>
#include <string>

#include "common/params.hpp"
#include "common/singleton.hpp"
#include "nlohmann/json.hpp"
namespace lk_vio
{
  namespace common
  {
    class ParamServer : public Singleton<ParamServer>
    {
    public:
      bool initialized{ false };

      std::string data_set_path;
      std::string trajectory_path;
      std::string dbow2_voc_path;
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

    private:
      ParamServer();
      friend class Singleton<ParamServer>;


    public:
      ParamServer( const ParamServer & ) = delete;
      ParamServer &operator=( const ParamServer & ) = delete;

      ~ParamServer() = default;
      void readConfigFile( const std::string &config_file_path );
    };
  }  // namespace common
}  // namespace lk_vio