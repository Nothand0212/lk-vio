#include "thirdparty/orb/orbExtractor.hpp"

namespace lvio
{
const int PATCH_SIZE      = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD  = 19;

static float IC_Angle( const cv::Mat &image, cv::Point2f pt, const std::vector<int> &u_max )
{
  int m_01 = 0, m_10 = 0;

  const uchar *center = &image.at<uchar>( cvRound( pt.y ), cvRound( pt.x ) );

  // v = 0，也就是中间的那一行
  for ( int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u )
  {
    m_10 += u * center[ u ];
  }

  // Go line by line in the circular patch
  int step = (int)image.step1();
  for ( int v = 1; v <= HALF_PATCH_SIZE; ++v )
  {
    // 由于是对称的，因此一次可以处理两行
    int v_sum = 0;
    int d     = u_max[ v ];
    for ( int u = -d; u <= d; ++u )
    {
      int val_plus  = center[ u + v * step ];
      int val_minus = center[ u - v * step ];
      v_sum += ( val_plus - val_minus );
      m_10 += u * ( val_plus + val_minus );
    }
    m_01 += v * v_sum;
  }

  return cv::fastAtan2( (float)m_01, (float)m_10 );
}

const float factorPI = (float)( CV_PI / 180.f );

static void computeOrbDescriptor( const cv::KeyPoint &key_point, const cv::Mat &image, const cv::Point *pattern, uchar *descptor )
{
  float angle = static_cast<float>( key_point.angle ) * factorPI;
  float a     = static_cast<float>( cos( angle ) );
  float b     = static_cast<float>( sin( angle ) );

  const uchar *center = &image.at<uchar>( cvRound( key_point.pt.y ), cvRound( key_point.pt.x ) );
  const int    step   = (int)image.step;

#define GET_VALUE( idx ) center[ cvRound( pattern[ idx ].x * b + pattern[ idx ].y * a ) * step + cvRound( pattern[ idx ].x * a - pattern[ idx ].y * b ) ]

  for ( int i = 0; i < 32; ++i, pattern += 16 )
  {
    int t0, t1, val;
    t0  = GET_VALUE( 0 );
    t1  = GET_VALUE( 1 );
    val = t0 < t1;
    t0  = GET_VALUE( 2 );
    t1  = GET_VALUE( 3 );
    val |= ( t0 < t1 ) << 1;
    t0 = GET_VALUE( 4 );
    t1 = GET_VALUE( 5 );
    val |= ( t0 < t1 ) << 2;
    t0 = GET_VALUE( 6 );
    t1 = GET_VALUE( 7 );
    val |= ( t0 < t1 ) << 3;
    t0 = GET_VALUE( 8 );
    t1 = GET_VALUE( 9 );
    val |= ( t0 < t1 ) << 4;
    t0 = GET_VALUE( 10 );
    t1 = GET_VALUE( 11 );
    val |= ( t0 < t1 ) << 5;
    t0 = GET_VALUE( 12 );
    t1 = GET_VALUE( 13 );
    val |= ( t0 < t1 ) << 6;
    t0 = GET_VALUE( 14 );
    t1 = GET_VALUE( 15 );
    val |= ( t0 < t1 ) << 7;
    descptor[ i ] = static_cast<uchar>( val );
  }
#undef GET_VALUE
}

static void makeOffset( int pixel[ 25 ], int row_stride, int pattern_size )
{
  static const int offsets16[][ 2 ] = { { 0, 3 },
                                        { 1, 3 },
                                        { 2, 2 },
                                        { 3, 1 },
                                        { 3, 0 },
                                        { 3, -1 },
                                        { 2, -2 },
                                        { 1, -3 },
                                        { 0, -3 },
                                        { -1, -3 },
                                        { -2, -2 },
                                        { -3, -1 },
                                        { -3, 0 },
                                        { -3, 1 },
                                        { -2, 2 },
                                        { -1, 3 } };

  const int( *offsets )[ 2 ] = offsets16;

  CV_Assert( pixel && offsets );
  int k = 0;
  for ( ; k < pattern_size; k++ )
  {
    pixel[ k ] = offsets[ k ][ 0 ] + offsets[ k ][ 1 ] * row_stride;
  }
  for ( ; k < 25; k++ )
  {
    pixel[ k ] = pixel[ k - pattern_size ];
  }
}

// ************************************* ///!SECTION ORBextractor

ORBextractor::ORBextractor( int num_features, float scale_factor, int num_levels, int ini_fast_threshold, int min_fast_threshold )
    : num_features_( num_features ), scale_factor_( scale_factor ), num_levels_( num_levels ), ini_fast_threshold_( ini_fast_threshold ), min_fast_threshold_( min_fast_threshold )
{
  scale_factors_vec_.resize( num_levels_ );
  level_sigma_sq_vec_.resize( num_levels_ );
  scale_factors_vec_[ 0 ]  = 1.0f;
  level_sigma_sq_vec_[ 0 ] = 1.0f;

  for ( int i = 1; i < num_levels_; i++ )
  {
    scale_factors_vec_[ i ]  = scale_factors_vec_[ i - 1 ] * scale_factor_;
    level_sigma_sq_vec_[ i ] = scale_factors_vec_[ i ] * scale_factors_vec_[ i ];
  }

  inv_scale_factors_vec_.resize( num_levels_ );
  inv_level_sigma_sq_vec_.resize( num_levels_ );

  for ( int i = 0; i < num_levels_; i++ )
  {
    inv_scale_factors_vec_[ i ]  = 1.0f / scale_factors_vec_[ i ];
    inv_level_sigma_sq_vec_[ i ] = 1.0f / level_sigma_sq_vec_[ i ];
  }

  image_pyramid_vec_.resize( num_levels_ );
  mask_pyramid_vec_.resize( num_levels_ );
  features_per_level_vec_.resize( num_levels_ );

  float factor = 1.0f / scale_factor_;

  float num_desired_features_per_scale = num_features_ * ( 1.0f - factor ) / ( 1.0f - (float)pow( (double)factor, (double)num_levels_ ) );

  int sum_features = 0;
  for ( int level = 0; level < num_levels_ - 1; level++ )
  {
    features_per_level_vec_[ level ] = cvRound( num_desired_features_per_scale );
    sum_features += features_per_level_vec_[ level ];
    num_desired_features_per_scale *= factor;
  }
  features_per_level_vec_[ num_levels_ - 1 ] = std::max( num_features_ - sum_features, 0 );  // 最后一层的特征点数

  const int        num_points = 512;
  const cv::Point *pattern0   = (const cv::Point *)bit_pattern_31_;

  std::copy( pattern0, pattern0 + num_points, std::back_inserter( pattern_ ) );

  umax_.resize( HALF_PATCH_SIZE + 1 );

  int v, v0, vmax = cvFloor( HALF_PATCH_SIZE * sqrt( 2.f ) / 2 + 1 );

  int vmin = cvCeil( HALF_PATCH_SIZE * sqrt( 2.f ) / 2 );

  const double half_patch_size_squared = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
  for ( v = 0; v <= vmax; ++v )
  {
    umax_[ v ] = cvRound( sqrt( half_patch_size_squared - v * v ) );
  }

  // 确保对称性
  for ( v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v )
  {
    while ( umax_[ v0 ] == umax_[ v0 + 1 ] )
    {
      ++v0;
    }
    umax_[ v ] = v0;
    ++v0;
  }
}

static bool isFastCorner( cv::Mat &image, cv::KeyPoint &key_point, int threshold )
{
  int pattern_size = 16;

  const int descriptor_radius = pattern_size / 2;
  const int descriptor_size   = pattern_size + descriptor_radius + 1;

  int pixel[ 25 ];

  makeOffset( pixel, static_cast<int>( image.step ), pattern_size );

  threshold = std::min( std::max( threshold, 0 ), 255 );

  uchar threshold_tab[ 512 ];
  for ( int i = -255; i <= 255; i++ )
  {
    if ( i < -threshold )
    {
      threshold_tab[ i + 255 ] = static_cast<uchar>( 1 );
    }
    else if ( i > threshold )
    {
      threshold_tab[ i + 255 ] = static_cast<uchar>( 2 );
    }
    else
    {
      threshold_tab[ i + 255 ] = static_cast<uchar>( 0 );
    }
  }

  const uchar *ptr = image.ptr<uchar>( cvRound( key_point.pt.y ) ) + cvRound( key_point.pt.x );

  int v = ptr[ 0 ];

  const uchar *tab = &threshold_tab[ 0 ] - v + 255;

  int d = tab[ ptr[ pixel[ 0 ] ] ] | tab[ ptr[ pixel[ 8 ] ] ];
  if ( d == 0 )
  {
    return false;
  }

  d &= tab[ ptr[ pixel[ 2 ] ] ] | tab[ ptr[ pixel[ 10 ] ] ];
  d &= tab[ ptr[ pixel[ 4 ] ] ] | tab[ ptr[ pixel[ 12 ] ] ];
  d &= tab[ ptr[ pixel[ 6 ] ] ] | tab[ ptr[ pixel[ 14 ] ] ];
  if ( d == 0 )
  {
    return false;
  }
  d &= tab[ ptr[ pixel[ 1 ] ] ] | tab[ ptr[ pixel[ 9 ] ] ];
  d &= tab[ ptr[ pixel[ 3 ] ] ] | tab[ ptr[ pixel[ 11 ] ] ];
  d &= tab[ ptr[ pixel[ 5 ] ] ] | tab[ ptr[ pixel[ 13 ] ] ];
  d &= tab[ ptr[ pixel[ 7 ] ] ] | tab[ ptr[ pixel[ 15 ] ] ];

  if ( d & 1 )
  {
    int vt    = v - threshold;
    int count = 0;
    for ( int k = 0; k < descriptor_size; k++ )
    {
      int pv = ptr[ pixel[ k ] ];
      if ( pv < vt )
      {
        if ( ++count > descriptor_radius )
        {
          return true;
        }
      }
      else
      {
        count = 0;
      }
    }
  }

  if ( d & 2 )
  {
    int vt    = v + threshold;
    int count = 0;
    for ( int k = 0; k < descriptor_size; k++ )
    {
      int pv = ptr[ pixel[ k ] ];
      if ( pv > vt )
      {
        if ( ++count > descriptor_radius )
        {
          return true;
        }
      }
      else
      {
        count = 0;
      }
    }
  }

  return false;
}

static void computeOrientation( const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, const std::vector<int> &umax )
{
  for ( auto &key_point : key_points )
  {
    key_point.angle = IC_Angle( image, key_point.pt, umax );
  }
}

void ExtractorNode::divideNode( ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4 )
{
  const int half_x = ceil( static_cast<float>( up_right_.x - up_left_.x ) / 2 );
  const int half_y = ceil( static_cast<float>( below_right_.y - below_left_.y ) / 2 );

  // 分裂成四个子节点
  n1.up_left_     = up_left_;
  n1.up_right_    = cv::Point2i( up_left_.x + half_x, up_left_.y );
  n1.below_left_  = cv::Point2i( up_left_.x, up_left_.y + half_y );
  n1.below_right_ = cv::Point2i( up_left_.x + half_x, up_left_.y + half_y );
  n1.key_points_vec_.reserve( key_points_vec_.size() );

  n2.up_left_     = n1.up_right_;
  n2.up_right_    = up_right_;
  n2.below_left_  = n1.below_right_;
  n2.below_right_ = cv::Point2i( up_right_.x, up_right_.y + half_y );
  n2.key_points_vec_.reserve( key_points_vec_.size() );

  n3.up_left_     = n1.below_left_;
  n3.up_right_    = n1.below_right_;
  n3.below_left_  = below_left_;
  n3.below_right_ = cv::Point2i( below_left_.x + half_x, below_left_.y );
  n3.key_points_vec_.reserve( key_points_vec_.size() );

  n4.up_left_     = n3.up_right_;
  n4.up_right_    = n2.below_right_;
  n4.below_left_  = n3.below_right_;
  n4.below_right_ = below_right_;
  n4.key_points_vec_.reserve( key_points_vec_.size() );

  // 将母节点的key points 分配给四个子节点
  for ( std::size_t i = 0; i < key_points_vec_.size(); i++ )
  {
    const cv::KeyPoint &key_point = key_points_vec_[ i ];
    if ( key_point.pt.x < n1.below_right_.x )
    {
      if ( key_point.pt.y < n1.below_right_.y )
      {
        n1.key_points_vec_.push_back( key_point );
      }
      else
      {
        n3.key_points_vec_.push_back( key_point );
      }
    }
    else if ( key_point.pt.y < n1.below_right_.y )
    {
      n2.key_points_vec_.push_back( key_point );
    }
    else
    {
      n4.key_points_vec_.push_back( key_point );
    }
  }

  if ( n1.key_points_vec_.size() == 1 )
  {
    n1.no_more_ = true;
  }
  if ( n2.key_points_vec_.size() == 1 )
  {
    n2.no_more_ = true;
  }
  if ( n3.key_points_vec_.size() == 1 )
  {
    n3.no_more_ = true;
  }
  if ( n4.key_points_vec_.size() == 1 )
  {
    n4.no_more_ = true;
  }
}

std::vector<cv::KeyPoint> ORBextractor::distributeOctTree( const std::vector<cv::KeyPoint> &key_points_to_distribute,
                                                           const int &min_x, const int &max_x,
                                                           const int &min_y, const int &max_y,
                                                           const int &features_num, const int &level )
{
  const int   initial_node_num = std::round( static_cast<float>( max_x - min_x ) / ( max_y - min_y ) );
  const float node_width       = static_cast<float>( max_x - min_x ) / initial_node_num;

  std::list<ExtractorNode>     nodes_list;
  std::vector<ExtractorNode *> initial_nodes_vec_ptr;
  initial_nodes_vec_ptr.resize( initial_node_num );

  for ( int i = 0; i < initial_node_num; i++ )
  {
    ExtractorNode ni;
    ni.up_left_     = cv::Point2i( node_width * static_cast<float>( i ), 0 );
    ni.up_right_    = cv::Point2i( node_width * static_cast<float>( i + 1 ), 0 );
    ni.below_left_  = cv::Point2i( ni.up_left_.x, max_y - min_y );
    ni.below_right_ = cv::Point2i( ni.up_right_.x, max_y - min_y );
    ni.key_points_vec_.reserve( key_points_to_distribute.size() );

    nodes_list.push_back( ni );
    initial_nodes_vec_ptr[ i ] = &nodes_list.back();
  }

  // 分配key points
  for ( std::size_t i = 0; i < key_points_to_distribute.size(); i++ )
  {
    const cv::KeyPoint &key_point = key_points_to_distribute[ i ];
    initial_nodes_vec_ptr[ key_point.pt.x / node_width ]->key_points_vec_.push_back( key_point );
  }

  // 检查节点
  std::list<ExtractorNode>::iterator list_iter = nodes_list.begin();
  while ( list_iter != nodes_list.end() )
  {
    if ( list_iter->key_points_vec_.size() == 1 )  // 只有一个key point的节点，不用再考虑了
    {
      list_iter->no_more_ = true;
      list_iter++;
    }
    else if ( list_iter->key_points_vec_.empty() )  // 找不出key points的节点，不用再考虑了
    {
      list_iter = nodes_list.erase( list_iter );
    }
    else
    {
      list_iter++;
    }
  }

  bool finish    = false;
  int  iteration = 0;

  std::vector<std::pair<int, ExtractorNode *>> key_points_num_and_node_ptr_vec;
  key_points_num_and_node_ptr_vec.reserve( nodes_list.size() * 4 );

  while ( !finish )
  {
    iteration++;

    int prev_size = nodes_list.size();

    list_iter = nodes_list.begin();

    int to_expand_nodes_num = 0;

    key_points_num_and_node_ptr_vec.clear();

    while ( list_iter != nodes_list.end() )
    {
      if ( list_iter->no_more_ )
      {
        // 只有一个key point的节点，不用再考虑了
        list_iter++;
        continue;
      }
      else
      {
        ExtractorNode n1, n2, n3, n4;
        list_iter->divideNode( n1, n2, n3, n4 );

        if ( n1.key_points_vec_.size() > 0 )
        {
          nodes_list.push_front( n1 );
          if ( n1.key_points_vec_.size() > 1 )
          {
            to_expand_nodes_num++;
            key_points_num_and_node_ptr_vec.push_back( std::make_pair( n1.key_points_vec_.size(), &nodes_list.front() ) );
            nodes_list.front().list_iterator_ = nodes_list.begin();
          }
        }
        if ( n2.key_points_vec_.size() > 0 )
        {
          nodes_list.push_front( n2 );
          if ( n2.key_points_vec_.size() > 1 )
          {
            to_expand_nodes_num++;
            key_points_num_and_node_ptr_vec.push_back( std::make_pair( n2.key_points_vec_.size(), &nodes_list.front() ) );
            nodes_list.front().list_iterator_ = nodes_list.begin();
          }
        }
        if ( n3.key_points_vec_.size() > 0 )
        {
          nodes_list.push_front( n3 );
          if ( n3.key_points_vec_.size() > 1 )
          {
            to_expand_nodes_num++;
            key_points_num_and_node_ptr_vec.push_back( std::make_pair( n3.key_points_vec_.size(), &nodes_list.front() ) );
            nodes_list.front().list_iterator_ = nodes_list.begin();
          }
        }
        if ( n4.key_points_vec_.size() > 0 )
        {
          nodes_list.push_front( n4 );
          if ( n4.key_points_vec_.size() > 1 )
          {
            to_expand_nodes_num++;
            key_points_num_and_node_ptr_vec.push_back( std::make_pair( n4.key_points_vec_.size(), &nodes_list.front() ) );
            nodes_list.front().list_iterator_ = nodes_list.begin();
          }
        }

        list_iter = nodes_list.erase( list_iter );  // 当前节点已经分裂成四个子节点，因此可以删除了
        continue;
      }
    }

    // 检查节点特征数 是否满足要求
    // 检查节点数 是否满足要求
    int nodes_num = static_cast<int>( nodes_list.size() );
    if ( nodes_num >= features_num || nodes_num == prev_size )
    {
      finish = true;
    }
    else if ( ( nodes_num + to_expand_nodes_num * 3 ) > features_num )
    {
      while ( !finish )
      {
        prev_size                                                                         = nodes_list.size();
        std::vector<std::pair<int, ExtractorNode *>> prev_key_points_num_and_node_ptr_vec = key_points_num_and_node_ptr_vec;
        key_points_num_and_node_ptr_vec.clear();

        std::sort( prev_key_points_num_and_node_ptr_vec.begin(), prev_key_points_num_and_node_ptr_vec.end() );

        for ( int j = prev_key_points_num_and_node_ptr_vec.size() - 1; j >= 0; j-- )
        {
          ExtractorNode n1, n2, n3, n4;
          prev_key_points_num_and_node_ptr_vec[ j ].second->divideNode( n1, n2, n3, n4 );

          if ( n1.key_points_vec_.size() > 0 )
          {
            nodes_list.push_front( n1 );
            if ( n1.key_points_vec_.size() > 1 )
            {
              key_points_num_and_node_ptr_vec.push_back( std::make_pair( n1.key_points_vec_.size(), &nodes_list.front() ) );
              nodes_list.front().list_iterator_ = nodes_list.begin();
            }
          }
          if ( n2.key_points_vec_.size() > 0 )
          {
            nodes_list.push_front( n2 );
            if ( n2.key_points_vec_.size() > 1 )
            {
              key_points_num_and_node_ptr_vec.push_back( std::make_pair( n2.key_points_vec_.size(), &nodes_list.front() ) );
              nodes_list.front().list_iterator_ = nodes_list.begin();
            }
          }
          if ( n3.key_points_vec_.size() > 0 )
          {
            nodes_list.push_front( n3 );
            if ( n3.key_points_vec_.size() > 1 )
            {
              key_points_num_and_node_ptr_vec.push_back( std::make_pair( n3.key_points_vec_.size(), &nodes_list.front() ) );
              nodes_list.front().list_iterator_ = nodes_list.begin();
            }
          }
          if ( n4.key_points_vec_.size() > 0 )
          {
            nodes_list.push_front( n4 );
            if ( n4.key_points_vec_.size() > 1 )
            {
              key_points_num_and_node_ptr_vec.push_back( std::make_pair( n4.key_points_vec_.size(), &nodes_list.front() ) );
              nodes_list.front().list_iterator_ = nodes_list.begin();
            }
          }

          nodes_list.erase( prev_key_points_num_and_node_ptr_vec[ j ].second->list_iterator_ );

          if ( static_cast<int>( nodes_list.size() ) >= features_num )
          {
            break;
          }
        }

        if ( static_cast<int>( nodes_list.size() ) >= features_num || static_cast<int>( nodes_list.size() ) == prev_size )
        {
          finish = true;
        }
      }
    }
  }

  // 取每个节点的最大响应的key point
  std::vector<cv::KeyPoint> result_key_points_vec;
  result_key_points_vec.reserve( features_num );
  for ( auto &node : nodes_list )
  {
    std::vector<cv::KeyPoint> &node_key_points_vec = node.key_points_vec_;

    cv::KeyPoint *key_point = &node_key_points_vec[ 0 ];

    float max_response = key_point->response;

    for ( std::size_t k = 1; k < node_key_points_vec.size(); k++ )
    {
      if ( node_key_points_vec[ k ].response > max_response )
      {
        key_point    = &node_key_points_vec[ k ];
        max_response = node_key_points_vec[ k ].response;
      }
    }

    result_key_points_vec.push_back( *key_point );
  }

  return result_key_points_vec;
}


void ORBextractor::computeKeyPointsOctTree( std::vector<std::vector<cv::KeyPoint>> &all_key_points )
{
  all_key_points.resize( num_levels_ );

  const float block_size = 30;

  for ( int level = 0; level < num_levels_; level++ )
  {
    const int min_border_x = EDGE_THRESHOLD - 3;
    const int min_border_y = min_border_x;
    const int max_border_x = image_pyramid_vec_[ level ].cols - EDGE_THRESHOLD + 3;
    const int max_border_y = image_pyramid_vec_[ level ].rows - EDGE_THRESHOLD + 3;

    std::vector<cv::KeyPoint> to_distribute_key_points_vec;
    to_distribute_key_points_vec.reserve( num_features_ * 10 );

    const float width  = static_cast<float>( max_border_x - min_border_x );
    const float height = static_cast<float>( max_border_y - min_border_y );

    const int cols_num         = width / block_size;
    const int rows_num         = height / block_size;
    const int width_cell_size  = ceil( width / cols_num );
    const int height_cell_size = ceil( height / rows_num );

    for ( int i = 0; i < rows_num; i++ )
    {
      const float ini_y = min_border_y + height_cell_size * i;
      float       max_y = ini_y + height_cell_size + 6;

      if ( ini_y >= max_border_y - 3 )
      {
        continue;
      }
      if ( max_y > max_border_y )
      {
        max_y = max_border_y;
      }

      for ( int j = 0; j < cols_num; j++ )
      {
        const float ini_x = min_border_x + width_cell_size * j;
        float       max_x = ini_x + width_cell_size + 6;

        if ( ini_x >= max_border_x - 6 )
        {
          continue;
        }
        if ( max_x > max_border_x )
        {
          max_x = max_border_x;
        }

        std::vector<cv::KeyPoint> key_points_on_cell_vec;

        cv::FAST( image_pyramid_vec_[ level ].rowRange( ini_y, max_y ).colRange( ini_x, max_x ),
                  key_points_on_cell_vec, ini_fast_threshold_, true );

        if ( key_points_on_cell_vec.empty() )
        {
          cv::FAST( image_pyramid_vec_[ level ].rowRange( ini_y, max_y ).colRange( ini_x, max_x ),
                    key_points_on_cell_vec, min_fast_threshold_, true );
        }

        if ( !key_points_on_cell_vec.empty() )
        {
          for ( auto &key_point : key_points_on_cell_vec )
          {
            key_point.pt.x += j * width_cell_size;
            key_point.pt.y += i * height_cell_size;
            int temp_height = cvRound( key_point.pt.y );
            int temp_width  = cvRound( key_point.pt.x );
            if ( mask_pyramid_vec_[ level ].ptr<uchar>( temp_height, temp_width ) == 0 )
            {
              continue;
            }
            to_distribute_key_points_vec.push_back( key_point );
          }
        }
      }
    }

    // 分配key points
    std::vector<cv::KeyPoint> &key_points_vec = all_key_points[ level ];
    key_points_vec.reserve( num_features_ );
    key_points_vec = distributeOctTree( to_distribute_key_points_vec, min_border_x, max_border_x, min_border_y, max_border_y, features_per_level_vec_[ level ], level );

    const int scaled_patch_size   = PATCH_SIZE * scale_factors_vec_[ level ];
    const int key_points_vec_size = key_points_vec.size();

    for ( int i = 0; i < key_points_vec_size; i++ )
    {
      key_points_vec[ i ].octave = level;
      key_points_vec[ i ].size   = scaled_patch_size;
      // 还原真实坐标
      key_points_vec[ i ].pt.x += min_border_x;
      key_points_vec[ i ].pt.y += min_border_y;
    }
  }

  for ( int level = 0; level < num_levels_; level++ )
  {
    computeOrientation( image_pyramid_vec_[ level ], all_key_points[ level ], umax_ );
  }
}

static void computeDescriptors( const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors, const std::vector<cv::Point> &pattern )
{
  descriptors = cv::Mat::zeros( static_cast<int>( key_points.size() ), 32, CV_8UC1 );
  for ( std::size_t i = 0; i < key_points.size(); i++ )
  {
    const cv::KeyPoint &key_point = key_points[ i ];
    computeOrbDescriptor( key_points[ i ], image, &pattern[ 0 ], descriptors.ptr( static_cast<int>( i ) ) );
  }
}

void ORBextractor::detectFeatures( const cv::Mat &image, const cv::Mat &mask, std::vector<cv::KeyPoint> &key_points_vec )
{
  if ( image.empty() || mask.empty() )
  {
    return;
  }

  cv::Mat temp_image = image.clone();
  assert( temp_image.type() == CV_8UC1 );
  cv::Mat temp_mask = mask.clone();
  assert( temp_mask.type() == CV_8UC1 );

  const float block_size   = 30;
  const int   min_border_x = EDGE_THRESHOLD - 3;
  const int   min_border_y = min_border_x;
  const int   max_border_x = temp_image.cols - EDGE_THRESHOLD + 3;
  const int   max_border_y = temp_image.rows - EDGE_THRESHOLD + 3;

  std::vector<cv::KeyPoint> to_distribute_key_points_vec;
  to_distribute_key_points_vec.reserve( num_features_ * 10 );

  const float width  = static_cast<float>( max_border_x - min_border_x );
  const float height = static_cast<float>( max_border_y - min_border_y );

  const int cols_num         = width / block_size;
  const int rows_num         = height / block_size;
  const int width_cell_size  = ceil( width / cols_num );
  const int height_cell_size = ceil( height / rows_num );

  for ( int i = 0; i < rows_num; i++ )
  {
    const float ini_y = min_border_y + height_cell_size * i;
    float       max_y = ini_y + height_cell_size + 6;

    if ( ini_y >= max_border_y - 3 )
    {
      continue;
    }
    if ( max_y > max_border_y )
    {
      max_y = max_border_y;
    }

    for ( int j = 0; j < cols_num; j++ )
    {
      const float ini_x = min_border_x + width_cell_size * j;
      float       max_x = ini_x + width_cell_size + 6;

      if ( ini_x >= max_border_x - 6 )
      {
        continue;
      }
      if ( max_x > max_border_x )
      {
        max_x = max_border_x;
      }

      std::vector<cv::KeyPoint> key_points_on_cell_vec;

      cv::FAST( temp_image.rowRange( ini_y, max_y ).colRange( ini_x, max_x ),
                key_points_on_cell_vec, ini_fast_threshold_, true );

      if ( key_points_on_cell_vec.empty() )
      {
        cv::FAST( temp_image.rowRange( ini_y, max_y ).colRange( ini_x, max_x ),
                  key_points_on_cell_vec, min_fast_threshold_, true );
      }

      if ( !key_points_on_cell_vec.empty() )
      {
        for ( auto &key_point : key_points_on_cell_vec )
        {
          key_point.pt.x += j * width_cell_size;
          key_point.pt.y += i * height_cell_size;
          int temp_height = cvRound( key_point.pt.y );
          int temp_width  = cvRound( key_point.pt.x );
          if ( temp_mask.ptr<uchar>( temp_height, temp_width ) == 0 )
          {
            continue;
          }
          to_distribute_key_points_vec.push_back( key_point );
        }
      }
    }
  }

  key_points_vec.reserve( num_features_ );
  key_points_vec = distributeOctTree( to_distribute_key_points_vec, min_border_x, max_border_x, min_border_y, max_border_y, num_features_, 0 );

  const int key_points_vec_size = key_points_vec.size();
  for ( int i = 0; i < key_points_vec_size; i++ )
  {
    key_points_vec[ i ].pt.x += min_border_x;
    key_points_vec[ i ].pt.y += min_border_y;
  }
}

void ORBextractor::filterKeyPoints( const cv::Mat &image, std::vector<cv::KeyPoint> &key_points_vec, std::vector<cv::KeyPoint> &out_key_points_vec )
{
  if ( image.empty() || key_points_vec.empty() )
  {
    return;
  }

  cv::Mat temp_image = image.clone();
  assert( temp_image.type() == CV_8UC1 );

  std::size_t key_points_num = key_points_vec.size();
  out_key_points_vec.clear();
  out_key_points_vec.reserve( key_points_num );

  computePyramid( temp_image );

  for ( std::size_t i = 0; i < key_points_num; i++ )
  {
    cv::KeyPoint &key_point        = key_points_vec[ i ];
    int           level            = key_point.octave;
    float         scale            = scale_factors_vec_[ level ];
    cv::Mat       image_on_pyramid = image_pyramid_vec_[ level ];

    key_point.pt /= scale;

    if ( !( key_point.pt.x >= EDGE_THRESHOLD && key_point.pt.x + EDGE_THRESHOLD < image_on_pyramid.cols &&
            key_point.pt.y >= EDGE_THRESHOLD && key_point.pt.y + EDGE_THRESHOLD < image_on_pyramid.rows ) )
    {
      key_point.pt *= scale;
      continue;
    }

    if ( !isFastCorner( image_on_pyramid, key_point, min_fast_threshold_ ) )
    {
      key_point.pt *= scale;
      continue;
    }

    key_point.angle = IC_Angle( image_on_pyramid, key_point.pt, umax_ );
    key_point.size  = PATCH_SIZE * scale;
    key_point.pt *= scale;

    out_key_points_vec.push_back( key_point );
  }
}

void ORBextractor::calculateDescriptors( const cv::Mat &image, const std::vector<cv::KeyPoint> &key_points_vec, cv::Mat &descriptors )
{
  if ( image.empty() || key_points_vec.empty() )
  {
    LOG( ERROR ) << "No Image or KeyPoints to Calculate Descriptor!";
    return;
  }

  cv::Mat temp_image = image.clone();
  assert( temp_image.type() == CV_8UC1 );

  computePyramid( temp_image );

  std::vector<cv::Mat> working_image_on_pyramid_vec( num_levels_ );
  for ( int level = 0; level < num_levels_; level++ )
  {
    cv::Mat working_image = image_pyramid_vec_[ level ].clone();
    cv::GaussianBlur( working_image, working_image, cv::Size( 7, 7 ), 2, 2, cv::BORDER_REFLECT_101 );
    working_image_on_pyramid_vec[ level ] = working_image;
  }

  if ( key_points_vec.size() == 0 )
  {
    descriptors.release();
  }
  else
  {
    descriptors.release();
    descriptors.create( static_cast<int>( key_points_vec.size() ), 32, CV_8UC1 );
  }

  for ( std::size_t i = 0; i < key_points_vec.size(); i++ )
  {
    cv::KeyPoint key_point = key_points_vec[ i ];
    int          level     = key_point.octave;
    float        scale     = scale_factors_vec_[ level ];

    cv::Mat descriptor = descriptors.rowRange( i, i + 1 );
    descriptor         = cv::Mat::zeros( 1, 32, CV_8UC1 );
    key_point.pt /= scale;
    computeOrbDescriptor( key_point, working_image_on_pyramid_vec[ level ], &pattern_[ 0 ], descriptor.ptr<uchar>( 0 ) );
    key_point.pt *= scale;
  }
}


void ORBextractor::computePyramid( const cv::Mat &image, const cv::Mat &mask )
{
  image_pyramid_vec_[ 0 ] = image.clone();
  mask_pyramid_vec_[ 0 ]  = mask.clone();

  for ( int level = 1; level < num_levels_; ++level )
  {
    const float scale = scale_factors_vec_[ level ];
    cv::Size    temp_size( cvRound( static_cast<float>( image.cols * scale ) ), cvRound( static_cast<float>( image.rows * scale ) ) );

    cv::resize( image_pyramid_vec_[ level - 1 ], image_pyramid_vec_[ level ], temp_size, 0, 0, cv::INTER_LINEAR );
    cv::resize( mask_pyramid_vec_[ level - 1 ], mask_pyramid_vec_[ level ], temp_size, 0, 0, cv::INTER_LINEAR );
  }
}

void ORBextractor::computePyramid( const cv::Mat &image )
{
  image_pyramid_vec_[ 0 ] = image.clone();

  for ( int level = 1; level < num_levels_; ++level )
  {
    const float scale = scale_factors_vec_[ level ];
    cv::Size    temp_size( cvRound( static_cast<float>( image.cols * scale ) ), cvRound( static_cast<float>( image.rows * scale ) ) );

    cv::resize( image_pyramid_vec_[ level - 1 ], image_pyramid_vec_[ level ], temp_size, 0, 0, cv::INTER_LINEAR );
  }
}

}  // namespace lvio