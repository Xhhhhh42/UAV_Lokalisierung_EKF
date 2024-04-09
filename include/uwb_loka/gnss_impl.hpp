#ifndef UWB_LOKA_GNSS_IMPL_H
#define UWB_LOKA_GNSS_IMPL_H

#include "gnss.h"

namespace uwb_loka 
{

void GNSS::set_params( GpsData_ConstPtr gps_data_ptr, const Eigen::Vector3d &I_p_Gps ) 
{
    init_lla_ = gps_data_ptr->lla;
    I_p_Gps_ = I_p_Gps;
}


Eigen::MatrixXd GNSS::measurement_function( const Eigen::MatrixXd &mat_x ) 
{
    Eigen::Isometry3d Twb;
    Twb.matrix() = mat_x;
    return Twb * I_p_Gps_;
}


/// @brief This function calculates the measurement residual, which is the difference
///        between the actual measurement (mat_z) and the predicted measurement
//         (calculated by the measurement_function using the state estimate mat_x).
/// @param mat_x 
/// @param mat_z 
/// @return 
Eigen::MatrixXd GNSS::measurement_residual( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ) 
{
    return mat_z - measurement_function( mat_x );
}


Eigen::MatrixXd GNSS::measurement_jacobian( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ) 
{
    Eigen::Isometry3d Twb;
    Twb.matrix() = mat_x;

    Eigen::Matrix<double, kMeas_dim, kState_dim> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(0, 6) = -Twb.linear() * skew_matrix( I_p_Gps_ );

    return H;
}


/**
 * @brief global to local coordinate, convert WGS84 to ENU frame
 *
 * @param gps_data_ptr
 * @return Eigen::Vector3d
 */
Eigen::Vector3d GNSS::g2l( GpsData_ConstPtr gps_data_ptr ) 
{
    Eigen::Vector3d p_G_Gps;
    GNSS::lla2enu( init_lla_, gps_data_ptr->lla, &p_G_Gps );
    return p_G_Gps;
}


/**
 * @brief local to glocal coordinate, convert ENU to WGS84 lla
 *
 * @param p_wb
 * @return Eigen::Vector3d
 */
Eigen::Vector3d GNSS::l2g(const Eigen::Vector3d &p_wb) 
{
    Eigen::Vector3d lla;
    GNSS::enu2lla( init_lla_, p_wb, &lla );
    return lla;
}


inline void GNSS::lla2enu( const Eigen::Vector3d &init_lla, const Eigen::Vector3d &point_lla, Eigen::Vector3d *point_enu ) 
{
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset( init_lla(0), init_lla(1), init_lla(2) );
    local_cartesian.Forward(
        point_lla(0), point_lla(1), point_lla(2), point_enu->data()[0], point_enu->data()[1], point_enu->data()[2] );
}


inline void GNSS::enu2lla( const Eigen::Vector3d &init_lla, const Eigen::Vector3d &point_enu, Eigen::Vector3d *point_lla ) 
{
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Reverse(
        point_enu(0), point_enu(1), point_enu(2), point_lla->data()[0], point_lla->data()[1], point_lla->data()[2]);
}

}  // namespace uwb_loka

#endif // UWB_LOKA_GNSS_IMPL_H