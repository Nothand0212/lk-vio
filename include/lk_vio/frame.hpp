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
  class IMUPreintegrator;

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

    // related with IM
    void            setVelocity( const Eigen::Vector3d &velocity );
    Eigen::Vector3d getVelocity() const;

    void setIMUPoseAndVelocity( const Sophus::SE3d &pose, const Eigen::Vector3d &velocity );
    void updatePoseMatrices();

    IMUBias                     getIMUBias() const;
    Eigen::Matrix<double, 3, 1> getTranslationWorldToCamera() const;
    Eigen::Matrix<double, 3, 3> getRotationWorldToCamera() const;
    Sophus::SE3d                getPoseWorldToCamera() const;


  public:
    unsigned long frame_id_;
    double        timestamp_;

    cv::Mat left_image_, right_image_;

    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<std::shared_ptr<Feature>> features_right_;

    // related with IMU
    // Frame *last_frame_{ nullptr };
    std::shared_ptr<Frame>    last_frame_sptr_;
    std::shared_ptr<KeyFrame> key_frame_sptr_;
    std::atomic_bool          is_intigrated_{ false };

    IMUCalibration    imu_calib_;
    IMUBias           imu_bias_;
    IMUBias           pre_imu_bias_;
    IMUPreintegrator *imu_preintegration_kf_{ nullptr };
    IMUPreintegrator *imu_preintegration_lf_{ nullptr };

  private:
    Sophus::SE3d  pose_;
    Sophus::SE3d &T_cam_w_ = pose_;
    // used for computing relative pose
    Eigen::Matrix<double, 3, 3> r_w_cam_;
    Eigen::Matrix<double, 3, 3> r_cam_w_;
    Eigen::Matrix<double, 3, 1> t_w_cam_;
    Eigen::Matrix<double, 3, 1> t_cam_w_;

    /// for tracking, What is stored is the pose relative to the previous keyframe T_{c_i {i-t}}
    /// Here we regard the moment i-t as a key frame, and i is the current moment
    Sophus::SE3d relative_pose_to_kf_;

    std::mutex update_pose_;
    std::mutex update_realteive_pose_;

    // related with IMU
    Eigen::Vector3d velocity_;
    bool            has_velocity_{ false };
  };

}  // namespace lk_vio
