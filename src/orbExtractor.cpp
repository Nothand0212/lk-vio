#include "orb/orbExtractor.hpp"

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

static void computeOrbDescriptor( const cv::KeyPoint &key_point, const Mat &image, const cv::Point *pattern, cv::uchar *descptor )
{
  float angle = static_cast<float>( key_point.angle ) * factorPI;
  float a     = static_cast<float>( cos( angle ) );
  float b     = static_cast<float>( sin( angle ) );

  const cv::uchar *center = &image.at<uchar>( cvRound( key_point.pt.y ), cvRound( key_point.pt.x ) );
  const int        step   = (int)image.step;

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
    descptor[ i ] = static_cast<cv::uchar>( val );
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

  cv::CV_Assert( pixel && offsets );
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

static bool isFastCorner( cv::mat &image, cv::KeyPoint &key_point, int threshold )
{
  int pattern_size = 16;

  const int descriptor_radius = pattern_size / 2;
  const int descriptor_size   = pattern_size + K + 1;

  int pixel[ 25 ];

  makeOffset( pixel, static_cast<int>( image.step ), pattern_size );

  threshold = std::min( std::max( threshold, 0 ), 255 );

  cv::uchar threshold_tab[ 512 ];
  for ( int i = -255; i <= 255; i++ )
  {
    if ( i < -threshold )
    {
      threshold_tab[ i + 255 ] = static_cast<cv::uchar>( 1 );
    }
    else if ( i > threshold )
    {
      threshold_tab[ i + 255 ] = static_cast<cv::uchar>( 2 );
    }
    else
    {
      threshold_tab[ i + 255 ] = static_cast<cv::uchar>( 0 );
    }
  }

  const cv::uchar *ptr = image.ptr<uchar>( cvRound( key_point.pt.y ) ) + cvRound( key_point.pt.x );

  int v = ptr[ 0 ];

  const cv::uchar *tab = &threshold_tab[ 0 ] - v + 255;

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

static void computeOrientation( const Mat &image, std::vector<cv::KeyPoint> &key_points, const std::vector<int> &umax )
{
  for ( auto &key_point : key_points )
  {
    key_point.angle = IC_Angle( image, key_point.pt, umax );
  }
}

void ExtractorNode::divideNode( ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4 )
{
  const int half_x = ceil( static_cast<float>( up_right_.x - up_left_.x ) / 2 );
  const int half_y = ceil( static_cast<float>( below_right_.y - below_left.y ) / 2 );

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
  for ( std::sizt_t i = 0; i < key_points_vec_size(); i++ )
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
                                                           const int &features_num, const int &level );
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

  bool finish     = false;
  int  interation = 0;

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
          node_list.push_front( n1 );
          if ( n1.key_points_vec_.size() > 1 )
          {
            to_expand_nodes_num++;
            key_points_num_and_node_ptr_vec.push_back( std::make_pair( n1.key_points_vec_.size(), &node_list.front() ) );
            nodes_list.front().list_iterator_ = node_list.begin();
          }
        }
        if ( n2.key_points_vec_.size() > 0 )
        {
          node_list.push_front( n2 );
          if ( n2.key_points_vec_.size() > 1 )
          {
            to_expand_nodes_num++;
            key_points_num_and_node_ptr_vec.push_back( std::make_pair( n2.key_points_vec_.size(), &node_list.front() ) );
            nodes_list.front().list_iterator_ = node_list.begin();
          }
        }
        if ( n3.key_points_vec_.size() > 0 )
        {
          node_list.push_front( n3 );
          if ( n3.key_points_vec_.size() > 1 )
          {
            to_expand_nodes_num++;
            key_points_num_and_node_ptr_vec.push_back( std::make_pair( n3.key_points_vec_.size(), &node_list.front() ) );
            nodes_list.front().list_iterator_ = node_list.begin();
          }
        }
        if ( n4.key_points_vec_.size() > 0 )
        {
          node_list.push_front( n4 );
          if ( n4.key_points_vec_.size() > 1 )
          {
            to_expand_nodes_num++;
            key_points_num_and_node_ptr_vec.push_back( std::make_pair( n4.key_points_vec_.size(), &node_list.front() ) );
            nodes_list.front().list_iterator_ = node_list.begin();
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
            node_list.push_front( n1 );
            if ( n1.key_points_vec_.size() > 1 )
            {
              key_points_num_and_node_ptr_vec.push_back( std::make_pair( n1.key_points_vec_.size(), &node_list.front() ) );
              nodes_list.front().list_iterator_ = node_list.begin();
            }
          }
          if ( n2.key_points_vec_.size() > 0 )
          {
            node_list.push_front( n2 );
            if ( n2.key_points_vec_.size() > 1 )
            {
              key_points_num_and_node_ptr_vec.push_back( std::make_pair( n2.key_points_vec_.size(), &node_list.front() ) );
              nodes_list.front().list_iterator_ = node_list.begin();
            }
          }
          if ( n3.key_points_vec_.size() > 0 )
          {
            node_list.push_front( n3 );
            if ( n3.key_points_vec_.size() > 1 )
            {
              key_points_num_and_node_ptr_vec.push_back( std::make_pair( n3.key_points_vec_.size(), &node_list.front() ) );
              nodes_list.front().list_iterator_ = node_list.begin();
            }
          }
          if ( n4.key_points_vec_.size() > 0 )
          {
            node_list.push_front( n4 );
            if ( n4.key_points_vec_.size() > 1 )
            {
              key_points_num_and_node_ptr_vec.push_back( std::make_pair( n4.key_points_vec_.size(), &node_list.front() ) );
              nodes_list.front().list_iterator_ = node_list.begin();
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

}  // namespace lvio