#pragma once
#include <atomic>

#include "Eigen/Core"
#include "lk_vio/feature.hpp"
#include "lk_vio/imu_frame.hpp"
#include "lk_vio/imu_preintegration.hpp"
#include "memory"
#include "mutex"
#include "opencv2/opencv.hpp"
#include "sophus/se3.hpp"
namespace lk_vio
{
  class Feature;
  class IMUBias;
  class IMUCalibration;
  class IntegratedRotation;
  class Preintegrated;

  class Frame
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;
    Frame() = default;

    ~Frame();
    // withou imu
    Frame( const cv::Mat &leftImg, const cv::Mat &rightImg, const double &dTimeStamp );

    // with imu
    Frame( const cv::Mat &leftImg, const cv::Mat &rightImg, const double &dTimeStamp, const IMUCalibration &imuCalib );
    void SetPose( const Sophus::SE3d &pose );
    /// the relative pose to the reference KF
    void         SetRelativePose( const Sophus::SE3d &relativePose );
    Sophus::SE3d getPose();
    Sophus::SE3d getRelativePose();

    // related with IMU
    void                   setLastFramePtr( std::shared_ptr<Frame> last_frame_sptr );
    std::shared_ptr<Frame> getLastFramePtr();

    void setIntegratedFlag();
    bool getIntegratedFlag();

    void setVelocity( const Eigen::Vector3d &velocity );

  public:
    unsigned long frame_id_;
    double        timestamp_;

    cv::Mat left_image_, right_image_;

    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<std::shared_ptr<Feature>> features_right_;

  private:
    Sophus::SE3d pose_;
    /// for tracking, What is stored is the pose relative to the previous keyframe T_{c_i {i-t}}
    /// Here we regard the moment i-t as a key frame, and i is the current moment
    Sophus::SE3d relative_pose_to_kf_;

    std::mutex update_pose_;
    std::mutex update_realteive_pose_;

    // related with IMU
    // Frame *last_frame_{ nullptr };
    std::shared_ptr<Frame> last_frame_sptr_;
    std::atomic_bool       is_intigrated_{ false };

    IMUCalibration imu_calib_;
    IMUBias        imu_bias_;
    IMUBias        pre_imu_bias_;
    Preintegrated *imu_preintegration_kf_{ nullptr };
    Preintegrated *imu_preintegration_lf_{ nullptr };
  };

}  // namespace lk_vio
