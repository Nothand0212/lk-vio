#include "ui/cloudUI.hpp"

namespace ui
{
CloudUI::CloudUI( const Eigen::Vector3d color, const CloudUI::UseColor use_color_sytle )
    : color_( color.cast<float>() ), use_color_( use_color_sytle )
{
  buildIntensityTable();
}

void CloudUI::buildIntensityTable()
{
  intensity_color_table_pcl_.resize( 256 * 6 );
  auto make_color = []( int r, int g, int b ) -> Eigen::Vector4f { return Eigen::Vector4f( r / 255.0, g / 255.0, b / 255.0, 1.0 ); };

  for ( int i = 0; i < 256; i++ )
  {
    intensity_color_table_pcl_.emplace_back( make_color( 255, i, 0 ) );
  }

  for ( int i = 0; i < 256; i++ )
  {
    intensity_color_table_pcl_.emplace_back( make_color( 255 - i, 0, 255 ) );
  }

  for ( int i = 0; i < 256; i++ )
  {
    intensity_color_table_pcl_.emplace_back( make_color( 0, 255, i ) );
  }

  for ( int i = 0; i < 256; i++ )
  {
    intensity_color_table_pcl_.emplace_back( make_color( 255, 255 - i, 0 ) );
  }

  for ( int i = 0; i < 256; i++ )
  {
    intensity_color_table_pcl_.emplace_back( make_color( i, 0, 255 ) );
  }

  for ( int i = 0; i < 256; i++ )
  {
    intensity_color_table_pcl_.emplace_back( make_color( 0, 255, 255 - i ) );
  }
}

void CloudUI::addCloudPoint( const Eigen::Vector3d &point )
{
  if ( points_vec_.size() > max_point_num_ )
  {
    // TODO 设置一个参数，控制删除多少点
    LOG( WARNING ) << "Point Cloud is too Large with " << points_vec_.size() << " Points, Delete Half of them";
    points_vec_.erase( points_vec_.begin(), points_vec_.begin() + points_vec_.size() / 2 );
  }
  points_vec_.emplace_back( point.cast<float>() );
  points_color_vec_.emplace_back( color_ );
}

void CloudUI::renderCloud()
{
  glPointSize( 2 );
  glBegin( GL_POINTS );
  for ( size_t i = 0; i < points_vec_.size() && i < points_color_vec_.size(); i++ )
  {
    auto color = points_color_vec_[ i ];
    glColor3f( color[ 0 ], color[ 1 ], color[ 2 ] );

    auto position = points_vec_[ i ];
    glVertex3d( position[ 0 ], position[ 1 ], position[ 2 ] );
  }
  glEnd();
}

Eigen::Vector4f CloudUI::intensityToRGB( const float &intensity ) const
{
  int index = static_cast<int>( intensity * 6 );
  index     = index % intensity_color_table_pcl_.size();
  return intensity_color_table_pcl_[ index ];
}

//
}  // namespace ui
