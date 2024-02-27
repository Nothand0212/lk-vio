#ifndef LVIO_MAPPOINT_HPP
#define LVIO_MAPPOINT_HPP

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "feature.hpp"

namespace lvio
{
  class MapPoint
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<MapPoint> Ptr;
    MapPoint();
    MapPoint( const std::size_t &id, const Eigen::Vector3d &position );
    ~MapPoint() = default;

    void        setId( const std::size_t &id );
    std::size_t getId();

    void            setPosition( const Eigen::Vector3d &position );
    Eigen::Vector3d getPosition();

    int getObservedTimes();
    int getActiveObservedTimes();

    void addObservation( std::shared_ptr<lvio::Feature> feature );
    void addActiveObservation( std::shared_ptr<lvio::Feature> feature );

    std::list<std::weak_ptr<lvio::Feature>> getObservations();
    std::list<std::weak_ptr<lvio::Feature>> getActiveObservations();

    void removeObservation( std::shared_ptr<lvio::Feature> feature );
    void removeActiveObservation( std::shared_ptr<lvio::Feature> feature );

    void setOutlierFlag( bool is_outlier );
    bool getOutlierFlag();

  private:
    std::size_t id_;

    int observed_times_        = 0;
    int active_observed_times_ = 0;

    bool is_outlier_ = false;

    std::mutex update_position_mutex_;
    std::mutex update_observation_mutex_;

    std::list<std::weak_ptr<Feature>> observations_;
    std::list<std::weak_ptr<Feature>> active_observations_;

    Eigen::Vector3d position_;
  };
  //
}  // namespace lvio

#endif  // LVIO_MAPPOINT_HPP