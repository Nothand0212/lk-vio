#pragma once
#include <cmath>
#include <iostream>

namespace common
{
  template <typename T>
  class RunningStat
  {
  public:
    RunningStat() : data_count( 0 ) {}

    void clear()
    {
      data_count = 0;
    }

    void addValue( T x )
    {
      if ( !std::isnan( x ) && !isnanf( x ) )
      {
        data_count++;

        // See Knuth TAOCP vol 2, 3rd edition, page 232
        if ( data_count == 1 )
        {
          old_mean = new_mean = x;
          old_squared_sum     = 0.0;
        }
        else
        {
          new_mean        = old_mean + ( x - old_mean ) / data_count;
          new_squared_sum = old_squared_sum + ( x - old_mean ) * ( x - new_mean );

          // set up for next iteration
          old_mean        = new_mean;
          old_squared_sum = new_squared_sum;
        }
      }
    }

    std::size_t getCount() const
    {
      return data_count;
    }

    T getMean() const
    {
      return ( data_count > 0 ) ? new_mean : 0.0;
    }

    T getVariance() const
    {
      return ( ( data_count > 1 ) ? new_squared_sum / ( data_count - 1 ) : 0.0 );
    }

    T getStandardDeviation() const
    {
      return sqrt( getVariance() );
    }

  private:
    std::size_t data_count;
    T           old_mean, new_mean, old_squared_sum, new_squared_sum;
  };
}  // namespace common