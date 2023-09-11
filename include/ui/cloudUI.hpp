#ifndef LVIO_CLOUD_UI_HPP
#define LVIO_CLOUD_UI_HPP

#include "Eigen/Core"
#include "glog/logging.h"
#include "pangolin/pangolin.h"
#include "sophus/se3.hpp"

namespace ui
{
class CloudUI
{
public:
  enum UseColor
  {
    SELF_DEFINE_COLOR,
    HEIGHT_COLOR,
  };

  CloudUI = default;
  CloudUI( const Eigen::Vector3d color, const CloudUI::UseColor use_color_sytle );

  void renderCloud;
  void buildIntensityTable();
  void addCloudPoint( const Eigen::Vector3d &point );

  Eigen::Vector4f intensityToRGB( const float &intensity ) const;

  void setUsedColor( CloudUI::UseColor color )
  {
    use_color_ = color;
  }

  void setSelfColor( const Eigen::Vector3d &color )
  {
    color_ = color.cast<float>();
  }

private:
  const int64_t                       max_point_num_ = 10000000;
  UseColor                            use_color_     = UseColor::HEIGHT_COLOR;
  Eigen::Vector3f                     color_         = Eigen::Vector3f( 1.0, 1.0, 1.0 );  // R G B --> 红色
  std::vector<Eigen::Vector3f>        points_vec_;                                        // 存三维点
  std::vector<Eigen::Vector4f>        points_color_vec_;                                  //存点的颜色
  static std::vector<Eigen::Vector4f> intensity_color_table_pcl_;                         /// PCL中intensity table
};

}  // namespace ui

#endif  // LVIO_CLOUD_UI_HPP