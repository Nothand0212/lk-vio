#include "mapPoint.hpp"
#include "setting.hpp"
#include "ui/pangolinWindowImpl.hpp"

namespace ui
{
  bool PangolinWindowImpl::initPangolin()
  {
    // Create a Window and Bind its Context to the Main Thread
    pangolin::CreateWindowAndBind( win_name_, win_width_, win_height_ );

    // Enable depth
    glEnable( GL_DEPTH_TEST );

    pangolin::GetBoundWindow()->RemoveCurrent();

    // Create TrajectoryUI
    // TODO 修改颜色
    no_backend_trajectory_ui_ = std::make_unique<ui::TrajectoryUI>( Eigen::Vector3d( 0.0, 1.0, 0.0 ) );
    backend_trajectory_ui_    = std::make_unique<ui::TrajectoryUI>( Eigen::Vector3d( 1.0, 0.0, 0.0 ) );

    // Create CloudUI
    cloud_ui_ = std::make_unique<ui::CloudUI>( Eigen::Vector3d( 0.0, 0.0, 1.0 ) );

    log_yaw_angle_.SetLabels( std::vector<std::string>{ "yaw_angle", "pitch_angle", "roll_angle" } );

    setDefaultViewImage();
    return true;
  }

  void PangolinWindowImpl::createDisplayLayout()
  {
    view_point_x_        = lvio::Setting::getParam<float>( "Viewer.ViewPoint.X" );
    view_point_y_        = lvio::Setting::getParam<float>( "Viewer.ViewPoint.Y" );
    view_point_z_        = lvio::Setting::getParam<float>( "Viewer.ViewPoint.Z" );
    view_point_focus_    = lvio::Setting::getParam<float>( "Viewer.ViewPoint.Focus" );  // TODO 后续需要参数化
    view_axis_direction_ = lvio::Setting::getParam<int>( "Viewer.ViewAxis.Direction" );

    auto projection_matrix_main = pangolin::ProjectionMatrix( win_width_, win_height_, view_point_focus_, view_point_focus_,
                                                              win_width_ / 2, win_height_ / 2, cam_z_near_, cam_z_far_ );
    auto model_view_main        = pangolin::ModelViewLookAt( view_point_x_, view_point_y_, view_point_z_, 0, 0, 0, static_cast<pangolin::AxisDirection>( view_axis_direction_ ) );

    render_state_main_ = pangolin::OpenGlRenderState( projection_matrix_main, model_view_main );

    pangolin::View &display_main = pangolin::Display( dis_3d_main_name_ ).SetBounds( 0.0, 1.0, 0.0, 1.0 ).SetHandler( new pangolin::Handler3D( render_state_main_ ) );
    pangolin::View &display_cam  = pangolin::Display( dis_3d_name_ ).SetBounds( 0.0, 1.0, 0.0, 1.0 ).SetLayout( pangolin::LayoutOverlay() ).AddDisplay( display_main );

    // &log_yaw_angle_：一个指向std::vector<float>类型对象的指针，用于存储偏航角的变化数据。
    // -10：曲线图的X轴范围的最小值。
    // 100：曲线图的X轴范围的最大值。
    // -M_PI：曲线图的Y轴范围的最小值，即偏航角的最小值。
    // M_PI：曲线图的Y轴范围的最大值，即偏航角的最大值。
    // 75：曲线图的宽度，即曲线图的像素宽度。
    // 2：曲线图的高度，即曲线图的像素高度。
    plotter_angle_ = std::make_unique<pangolin::Plotter>( &log_angle_, -10, 100, -M_PI, M_PI, 75, 2 );
    // 0.：曲线图的X轴范围的最小值。
    // 1 / 3.0f：曲线图的X轴范围的最大值。
    // 0.0f：曲线图的Y轴范围的最小值。
    // 0.98f：曲线图的Y轴范围的最大值。
    // 752 / 480.：曲线图的宽高比，即曲线图的宽度与高度之比。
    plotter_angle_->SetBounds( 0., 1 / 3.0f, 0.0f, 0.98f, 752 / 480. );
    // "$i"表示使用整数作为横坐标。
    plotter_angle_->Track( "$i" );
    plotter_angle_->SetBackgroundColour( pangolin::Colour( 248. / 255., 248. / 255., 255. / 255. ) );
    // 1 / 3.0f：视图的X轴范围的最小值。
    // 2 / 3.0f：视图的X轴范围的最大值。
    // 0.：视图的Y轴范围的最小值。
    // 0.98f：视图的Y轴范围的最大值。
    // 800 / 480.：视图的宽高比，即视图的宽度与高度之比。
    pangolin::View &m_left_imageshow = pangolin::Display( m_left_imageview_name_ ).SetBounds( 1 / 3.0f, 2 / 3.0f, 0., 0.98f, 800 / 480. );
    pangolin::View &right_image_show = pangolin::Display( right_image_view_name_ ).SetBounds( 2 / 3.0f, 1.0f, 0.0f, 0.98f, 800 / 480. );
    pangolin::View &plot_show        = pangolin::Display( dis_plot_name_ ).SetBounds( 0.0f, 1.0, 0.70, 1.0 ).AddDisplay( *plotter_angle_ ).AddDisplay( m_left_imageshow ).AddDisplay( right_image_show );
    pangolin::Display( dis_main_name_ ).SetBound( 0.0, 1.0, pangolin::Attach::Pix( menu_width_ ), 1.0 ).AddDisplay( display_cam ).AddDisplay( plot_show );

    gl_texture_m_left_image = std::make_unique<pangolin::GlTexture>( image_width_, image_height_, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE );
    gl_texture_right_image_ = std::make_unique<pangolin::GlTexture>( image_width_, image_height_, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE );
  }

  void PangolinWindowImpl::render()
  {
    pangolin::BindToContext( win_name_ );

    glEnable( GL_DEPTH_TEST );
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    // menu
    pangolin::CreatePanel( "menu" ).SetBounds( 0.0, 1.0, 0.0, pangolin::Attach::Pix( menu_width_ ) );
    // 第一个bool值是默认值，第二个bool值是是否可以更改
    pangolin::Var<bool> menu_follow_loc( "menu.Follow", false, true );
    pangolin::Var<bool> menu_reset_3d_view( "menu.Reset 3D View", false, false );
    pangolin::Var<bool> menu_show_mappoint( "menu.Show PointCloud", false, false );
    pangolin::Var<bool> menu_show_trajectory( "menu.Show Trajectory", false, false );

    createDisplayLayout();

    while ( !pangolin::ShouldQuit() && !exit_flag_ )
    {
      glClearColor( 255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0, 1.0 );
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
      pangolin::Display( dis_main_name_ ).Activate( render_state_main_ );

      following_camera_ = menu_follow_loc;

      if ( menu_reset_3d_view )
      {
        render_state_main_.SetModelViewMatrix( pangolin::ModelViewLookAt( view_point_x_, view_point_y_, view_point_z_, 0, 0, 0, static_cast<pangolin::AxisDirection>( view_axis_direction_ ) ) );
        menu_reset_3d_view = false;
      }

      renderAll();

      pangolin::FinishFrame();
    }
    pangolin::GetBoundWindow()->RemoveCurrent();
  }

  void PangolinWindowImpl::setViewImage( const cv::Mat &left_image, const cv::Mat &right_image )
  {
    assert( !left_image.empty() && left_image.cols > 0 && left_image.rows > 0 );
    assert( !right_image.empty() && right_image.cols > 0 && right_image.rows > 0 );

    cv::Mat left_dist, right_dist;
    cv::cvtColor( left_image, left_dist, cv::COLOR_BGR2RGB );
    cv::cvtColor( right_image, right_dist, cv::COLOR_BGR2RGB );
    std::unique_lock<std::mutex> lock( update_image_mutex_ );

    cv::resize( left_dist, m_left_image, cv::Size( image_width_, image_height_ ) );
    cv::resize( right_dist, right_image_, cv::Size( image_width_, image_height_ ) );
  }

  bool PangolinWindowImpl::renderViewImage()
  {
    assert( m_left_image.cols > 0 && m_left_image.rows > 0 );
    assert( right_image_.cols > 0 && right_image_.rows > 0 );

    {
      std::unique_lock<std::mutex> lock( update_image_mutex_ );
      gl_texture_m_left_image->Upload( m_left_image.data, GL_BGR, GL_UNSIGNED_BYTE );
      gl_texture_right_image_->Upload( right_image_.data, GL_BGR, GL_UNSIGNED_BYTE );
    }

    pangolin::Display( right_image_view_name_ ).Activate();
    glColor3f( 1.0, 1.0, 1.0 );
    gl_texture_right_image_->RenderToViewportFlipY();

    pangolin::Display( m_left_imageview_name_ ).Activate();
    glColor3f( 1.0, 1.0, 1.0 );
    gl_texture_m_left_image->RenderToViewportFlipY();

    return true;
  }

  void PangolinWindowImpl::setDefaultViewImage()
  {
    std::unique_lock<std::mutex> lock( update_image_mutex_ );
    m_left_image = cv::Mat( image_height_, image_width_, CV_8UC3, cv::Scalar( 255 ) );
    // left_img_：要绘制文本的图像。
    // "NoImg"：要绘制的文本内容。
    // cv::Point2i( image_width_ / 2 - 60, image_height_ / 2 )：文本的起始坐标，即文本左下角的坐标。
    // 2：字体的类型，取值为0到8之间的整数，表示不同的字体类型。
    // 2：字体的大小。
    // cv::Scalar( 255, 255, 255 )：文本的颜色，由红、绿、蓝三个通道的颜色值构成，取值范围为0到255之间。
    cv::putText( m_left_image, "No Image", cv::Point2i( image_width_ / 2 - 60, image_height_ / 2 ), 2, 2, cv::Scalar( 255, 255, 255 ) );

    right_image_ = cv::Mat( image_height_, image_width_, CV_8UC3, cv::Scalar( 255 ) );
    cv::putText( right_image_, "No Image", cv::Point2i( image_width_ / 2 - 60, image_height_ / 2 ), 2, 2, cv::Scalar( 255, 255, 255 ) );
  }

  void PangolinWindowImpl::renderAll()
  {
    renderViewImage();
    renderPlotterDataLog();
    {
      pangolin::Display( dis_3d_main_name_ ).Activate( render_state_main_ );
      std::unique_lock<std::mutex> lock( update_vo_state_ );
      no_backend_trajectory_ui_->render();
    }

    {
      pangolin::Display( dis_3d_name_ ).Activate( render_state_main_ );
      renderMapFrameAndMapPoints();
      backend_trajectory_ui_->render();
    }
  }

  void PangolinWindowImpl::renderMapFrameAndMapPoints()
  {
    float blue[ 3 ] = { 0, 0, 1 };
    float red[ 3 ]  = { 1, 0, 0 };

    if ( map_ == nullptr )
    {
      return;
    }

    std::unique_lock<std::mutex> lock( map_->update_map_mutex_ );

    {
      backend_trajectory_ui_->getTrajectoryPoses().clear();
      for ( auto &key_frame : map_->getAllKeyFrames() )
      {
        backend_trajectory_ui_->addTrajectoryPose( key_frame.second->getPose().inverse() );
        drawFrame( key_frame.second->getPose().inverse(), blue );
      }
    }

    glPointSize( 2 );
    glBegin( GL_POINTS );
    {
      for ( auto &map_point : map_->getAllMapPoints() )
      {
        auto position = map_point.second->getPosition();
        glColor3f( red[ 0 ], red[ 1 ], red[ 2 ] );
        glVertex3d( position[ 0 ], position[ 1 ], position[ 2 ] );
      }
    }
  }

  void PangolinWindowImpl::setEulerAngle( float yaw, float pitch, float roll )
  {
    std::unique_lock<std::mutex> lock( update_euler_angle_mutex_ );
    // std::get返回的是左值引用，可以直接赋值
    std::get<0>( euler_angle_ ) = yaw;
    std::get<1>( euler_angle_ ) = pitch;
    std::get<2>( euler_angle_ ) = roll;
  }

  bool PangolinWindowImpl::renderPlotterDataLog()
  {
    std::unique_lock<std::mutex> lock( update_euler_angle_mutex_ );
    log_angle_.Log( std::get<0>( euler_angle_ ), std::get<1>( euler_angle_ ), std::get<2>( euler_angle_ ) );
    return true;
  }

  void PangolinWindowImpl::updateTrajectory( const Sophus::SE3d &pose )
  {
    std::unique_lock<std::mutex> lock( update_vo_state_ );
    no_backend_trajectory_ui_->addTrajectoryPose( pose );
  }

  void PangolinWindowImpl::updatePointCloud( const Eigen::Vector3d &point )
  {
    std::unique_lock<std::mutex> lock( update_vo_cloud_ );
    cloud_ui_->addPoint( point );
  }

  void PangolinWindowImpl::drawFrame( const Sophus::SE3d &pose, const float *color )
  {
    Sophus::SE3d Twc        = pose;
    const float  sz         = 1.3;
    const int    line_width = 2.0;
    const float  fx         = 400;
    const float  fy         = 400;
    const float  cx         = 700;
    const float  cy         = 400;
    const float  width      = 1400;
    const float  height     = 800;

    glPushMatrix();
    Sophus::Matrix4f m = Twc.matrix().cast<float>();  // Twc.matrix().template cast<float>();
    glMultMatrixf( (GLfloat *)m.data() );

    if ( color == nullptr )
    {
      glColor3f( 1, 0, 0 );
    }
    else
    {
      glColor3f( color[ 0 ], color[ 1 ], color[ 2 ] );
    }

    glLineWidth( line_width );
    glBegin( GL_LINES );
    glVertex3f( 0, 0, 0 );
    glVertex3f( sz * ( 0 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );
    glVertex3f( 0, 0, 0 );
    glVertex3f( sz * ( 0 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );
    glVertex3f( 0, 0, 0 );
    glVertex3f( sz * ( width - 1 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );
    glVertex3f( 0, 0, 0 );
    glVertex3f( sz * ( width - 1 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );

    glVertex3f( sz * ( width - 1 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );
    glVertex3f( sz * ( width - 1 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );

    glVertex3f( sz * ( width - 1 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );
    glVertex3f( sz * ( 0 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );

    glVertex3f( sz * ( 0 - cx ) / fx, sz * ( height - 1 - cy ) / fy, sz );
    glVertex3f( sz * ( 0 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );

    glVertex3f( sz * ( 0 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );
    glVertex3f( sz * ( width - 1 - cx ) / fx, sz * ( 0 - cy ) / fy, sz );


    glEnd();
    glPopMatrix();
  }

  //
}  // namespace ui