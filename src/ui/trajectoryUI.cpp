#include "ui/trajectoryUI.hpp"

namespace ui
{
void TrajectoryUI::addTrajectoryPose( const Sophus::SE3d &pose )
{
  pose_vec_.emplace_back( pose.cast<float>() );
  if ( pose_vec_.size() > max_size_ )
  {
    pose_vec_.erase( pose_vec_.begin(), pose_vec_.begin() + pose_vec_.size() / 2 );
  }

  positions.emplace_back( pose.translation().cast<float>() );
  if ( positions_vec.size() > max_size_ )
  {
    positions_vec.erase( positions_vec.begin(), positions_vec.begin() + positions_vec.size() / 2 );
  }
}

void TrajectoryUI::render()
{
  if ( positions_vec.empty() )
  {
    return;
  }
  glPointSize( 5 );
  glBegin( GL_POINTS );
  for ( auto &position : positions_vec_ )
  {
    glColor3f( color_[ 0 ], color_[ 1 ], color_[ 2 ] );
    glVertex3f( position[ 0 ], position[ 1 ], position[ 2 ] );
  }
  glEnd();
}

void TrajectoryUI::clear()
{
  positions_vec_.clear();
  positions_vec_.reserve( max_size_ );
}
//
}  // namespace ui