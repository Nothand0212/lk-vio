#ifndef LVIO_ORB_EXTRACTOR_HPP
#define LVIO_ORB_EXTRACTOR_HPP

#include <opencv4/opencv2/core/hal/interface.h>

#include <iostream>
#include <list>
#include <memory>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/features2d/features2d.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>

#include "glog/logging.h"
#include "thirdparty/orb/orbPattern.hpp"

namespace lvio
{
class ExtractorNode
{
public:
  ExtractorNode() : no_more_( false ) {}

  void divideNode( ExtractorNode& n1, ExtractorNode& n2, ExtractorNode& n3, ExtractorNode& n4 );

  std::vector<cv::KeyPoint>          key_points_vec_;
  cv::Point2i                        up_left_, up_right_, below_left_, below_right_;
  std::list<ExtractorNode>::iterator list_iterator_;
  bool                               no_more_;
};

class ORBextractor
{
public:
  typedef std::shared_ptr<ORBextractor> Ptr;

  ORBextractor( int num_features, float scale_factor, int num_levels, int ini_fast_threshold, int min_fast_threshold );

  void calculateDescriptors( const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors );

  void detectFeatures( const cv::Mat& image, const cv::Mat& mask, std::vector<cv::KeyPoint>& keypoints );

  void detectFeaturesOnPyramid( const cv::Mat& image, const cv::Mat& mask, std::vector<cv::KeyPoint>& keypoints );

  void filterKeyPoints( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints_in, std::vector<cv::KeyPoint>& keypoints_out );

private:
  std::vector<cv::Mat> image_pyramid_vec_;
  std::vector<cv::Mat> mask_pyramid_vec_;

protected:
  void computePyramid( const cv::Mat& image, const cv::Mat& mask );
  void computePyramid( const cv::Mat& image );

  void computeKeyPointsOctTree( std::vector<std::vector<cv::KeyPoint>>& all_keypoints );

  std::vector<cv::KeyPoint> distributeOctTree( const std::vector<cv::KeyPoint>& key_points_to_distribute, const int& min_x, const int& max_x,
                                               const int& min_y, const int& max_y, const int& features_num, const int& level );

  std::vector<cv::Point> pattern_;

  int              num_features_;
  double           scale_factor_;
  int              num_levels_;
  int              ini_fast_threshold_;
  int              min_fast_threshold_;
  std::vector<int> umax_;
  std::vector<int> features_per_level_vec_;

  std::vector<float> scale_factors_vec_;
  std::vector<float> inv_scale_factors_vec_;
  std::vector<float> level_sigma_sq_vec_;
  std::vector<float> inv_level_sigma_sq_vec_;
  //
};


}  // namespace lvio

#endif  // LVIO_ORB_EXTRACTOR_HPP