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


}  // namespace lvio