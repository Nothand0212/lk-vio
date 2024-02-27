#pragma once


#include <opencv2/opencv.hpp>

class Region
{
public:
  cv::Rect2i      rectangle;
  std::deque<int> indexs;
};

class KeyPoints
{
private:
  std::vector<cv::Point2f> m_v_key_points;
  std::vector<float>       m_v_scores;
  cv::Mat                  m_mat_descriptor;

public:
  KeyPoints()  = default;
  ~KeyPoints() = default;

  void operator=( const KeyPoints& key_points )
  {
    m_v_key_points   = key_points.m_v_key_points;
    m_v_scores       = key_points.m_v_scores;
    m_mat_descriptor = key_points.m_mat_descriptor.clone();
  }

  void setKeyPoints( const std::vector<cv::Point2f>& key_points )
  {
    m_v_key_points = key_points;
  }


  void setScores( const std::vector<float>& scores )
  {
    m_v_scores = scores;
  }

  void setDescriptor( const cv::Mat& descriptor )
  {
    m_mat_descriptor = descriptor.clone();
  }

  std::vector<cv::Point2f> getKeyPoints() const
  {
    return m_v_key_points;
  }

  std::vector<float> getScores() const
  {
    return m_v_scores;
  }

  cv::Mat getDescriptor() const
  {
    return m_mat_descriptor;
  }
};