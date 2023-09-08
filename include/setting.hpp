#ifndef LVIO_SETTING_HPP
#define LVIO_SETTING_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <filesystem>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>

#include "glog/logging.h"
#include "sophus/se3.hpp"

namespace lvio
{
static std::once_flag singleton_flag;

class Setting
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static std::shared_ptr<Setting> getSingleton()
  {
    std::call_once( singleton_flag, [ & ] { singleton_ = std::shared_ptr<Setting>( new Setting() ); } );
    return singleton_;
  }

  bool initParamSetting( const std::string config_file_path );

  template <typename T>
  static T getParam( const std::string &key )
  {
    return T( Setting::singleton_->file_[ key ] );
  }

  ~Setting() = default;

private:
  Setting() = default;

private:
  static std::shared_ptr<Setting> singleton_;

  cv::FileStorage file_;
};

inline bool Setting::initParamSetting( const std::string config_file_path )
{
  LOG_ASSERT( std::filesystem::exists( config_file_path ) ) << "Config file not exist! Please Check!";
  singleton_->file_ = cv::FileStorage( config_file_path.c_str(), cv::FileStorage::READ );

  if ( !singleton_->file_.isOpened() )
  {
    LOG( FATAL ) << "Parameter File: \"" << config_file_path << " \" Does not Exist.";
    singleton_->file_.release();
    return false;
  }

  return true;
}

}  // namespace lvio

#endif  // LVIO_SETTING_HPP