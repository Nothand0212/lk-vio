#ifndef LVIO_KEYFRAME_HPP
#define LVIO_KEYFRAME_HPP

#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <vector>

#include "frame.hpp"
#include "sophus/se3.hpp"
#include "thirdparty/orb/orbVocabulary.hpp"

namespace lvio
{
  class MapPoint;

  class KeyFrame
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<KeyFrame> Ptr;

    KeyFrame() = default;

    KeyFrame( std::shared_ptr<Frame> frame );

    ~KeyFrame() = default;

    static KeyFrame::Ptr createKeyFramePtrFromFramPtr( std::shared_ptr<Frame> frame );

    void        setKeyFrameId( const std::size_t& key_frame_id );
    std::size_t getKeyFrameId();

    void        setFrameId( const std::size_t& frame_id );
    std::size_t getFrameId();

    void   setTimeStamp( const double& time_stamp );
    double getTimeStamp();

    void         setPose( const Sophus::SE3d& pose );
    Sophus::SE3d getPose();

    void         setPoseToLastKeyFrame( const Sophus::SE3d& pose_to_last_key_frame );
    Sophus::SE3d getPoseToLastKeyFrame();

    void         setPoseToLoopKeyFrame( const Sophus::SE3d& pose_to_loop_key_frame );
    Sophus::SE3d getPoseToLoopKeyFrame();

    void                      setKeyPoints( const std::vector<cv::KeyPoint>& key_points );
    std::vector<cv::KeyPoint> getKeyPoints();

    std::vector<cv::KeyPoint> getPyramidKeyPoints();

    void                                  setLeftFeatures( const std::vector<std::shared_ptr<Feature>>& features_in_left_image );
    std::vector<std::shared_ptr<Feature>> getLeftFeatures();

    void    setORBDescriptors( const cv::Mat& ORB_descriptors );
    cv::Mat getORBDescriptors();

    void    setLeftImage( const cv::Mat& left_image );
    cv::Mat getLeftImage();

    void             setBowVec( const DBoW2::BowVector& bow_vec );
    DBoW2::BowVector getBowVec();

    void                      setLastKeyFramePtr( std::shared_ptr<KeyFrame> last_key_frame );
    std::shared_ptr<KeyFrame> getLastKeyFramePtr();

    void                      setLoopKeyFramePtr( std::shared_ptr<KeyFrame> loop_key_frame );
    std::shared_ptr<KeyFrame> getLoopKeyFramePtr();

  private:
    std::size_t frame_id_;
    std::size_t key_frame_id_;

    double time_stamp_;

    // 三种位姿，分别是相机到世界坐标系的位姿，相机到上一关键帧的位姿，相机到回环帧的位姿
    Sophus::SE3d pose_;                    // T_c_w
    Sophus::SE3d pose_to_last_key_frame_;  // T_c_kf_last
    Sophus::SE3d pose_to_loop_key_frame_;  // T_c_kf_loop

    std::weak_ptr<KeyFrame> last_key_frame_wptr_;
    std::weak_ptr<KeyFrame> loop_key_frame_wptr_;

    std::vector<cv::KeyPoint>             pyramid_key_points_;  // 计算ORB特征点时，使用的金字塔特征点
    std::vector<std::shared_ptr<Feature>> features_in_left_image_;

    cv::Mat ORBDescriptors_;
    cv::Mat left_image_;

    DBoW2::BowVector bow_vec_;

    std::mutex update_pose_mutex_;
  };
  //
}  // namespace lvio

#endif  // LVIO_KEYFRAME_HPP