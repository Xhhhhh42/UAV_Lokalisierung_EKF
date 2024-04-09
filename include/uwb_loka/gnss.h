#ifndef UWB_LOKA_GNSS_H
#define UWB_LOKA_GNSS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeographicLib/LocalCartesian.hpp>
#include <memory>

#include "common.hpp"
#include "observer.h"

namespace uwb_loka 
{

class GNSS : public Observer 
{
public:
  GNSS() = default;

  virtual ~GNSS() {}

  void set_params( GpsData_ConstPtr gps_data_ptr, const Eigen::Vector3d &I_p_Gps = Eigen::Vector3d::Zero() );

  virtual Eigen::MatrixXd measurement_function( const Eigen::MatrixXd &mat_x );

  virtual Eigen::MatrixXd measurement_residual( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z );

  virtual Eigen::MatrixXd measurement_jacobian( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z );

  virtual void check_jacobian( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ) {}

  Eigen::Vector3d g2l( GpsData_ConstPtr gps_data_ptr);

  Eigen::Vector3d l2g( const Eigen::Vector3d &p_wb);

  static inline void lla2enu( const Eigen::Vector3d &init_lla, const Eigen::Vector3d &point_lla, Eigen::Vector3d *point_enu );

  static inline void enu2lla( const Eigen::Vector3d &init_lla, const Eigen::Vector3d &point_enu, Eigen::Vector3d *point_lla );

private:
  Eigen::Vector3d init_lla_;
  Eigen::Vector3d I_p_Gps_ = Eigen::Vector3d::Zero(); // GPS 接收器在惯性坐标系中的位置
};

using GNSS_Ptr = std::shared_ptr<GNSS>;

}  // namespace uwb_loka

#include "gnss_impl.hpp"

#endif // UWB_LOKA_GNSS_H