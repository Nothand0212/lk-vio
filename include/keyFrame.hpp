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

    void    setDescriptors( const cv::Mat& descriptors );
    cv::Mat getDescriptors();

    void    setLeftImage( const cv::Mat& left_image );
    cv::Mat getLeftImage();

    void             setBowVec( const DBoW2::BowVector& bow_vec );
    DBoW2::BowVector getBowVec();

    void                      setLastKeyFramePtr( std::shared_ptr<KeyFrame> last_key_frame );
    std::shared_ptr<KeyFrame> getLastKeyFramePtr();

    void                      setLoopKeyFramePtr( std::shared_ptr<KeyFrame> loop_key_frame );
    std::shared_ptr<KeyFrame> getLoopKeyFramePtr();

  private:
    std::size_t m_frame_id;
    std::size_t m_key_frame_id;

    double m_time_stamp;

    // 三种位姿，分别是相机到世界坐标系的位姿，相机到上一关键帧的位姿，相机到回环帧的位姿
    Sophus::SE3d m_pose;                    // T_c_w
    Sophus::SE3d m_pose_to_last_key_frame;  // T_c_kf_last
    Sophus::SE3d m_pose_to_loop_key_frame;  // T_c_kf_loop

    std::weak_ptr<KeyFrame> m_wptr_last_key_frame;
    std::weak_ptr<KeyFrame> m_wptr_loop_key_frame;

    std::vector<cv::KeyPoint>             m_key_points;
    std::vector<std::shared_ptr<Feature>> m_features_in_m_left_image;

    cv::Mat m_descriptors;
    cv::Mat m_left_image;

    DBoW2::BowVector m_bow_vec;

    std::mutex update_pose_mutex_;
  };
  //
}  // namespace lvio

#endif  // LVIO_KEYFRAME_HPP