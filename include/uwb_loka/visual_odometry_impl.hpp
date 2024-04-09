#ifndef UWB_LOKA_VISUAL_ODOMETRY_IMPL_H
#define UWB_LOKA_VISUAL_ODOMETRY_IMPL_H

#include "visual_odometry .h"
#include "common.hpp"

namespace uwb_loka 
{
constexpr int kMeasDim = 6;

void V_Odom::set_params( const Eigen::Isometry3d &Tvw, const Eigen::Isometry3d &Tcb ) 
{
    Tvw_ = Tvw;
    Tcb_ = Tcb;
}


/**
   * @brief measurement estimation h(x), Twb in frame V --> Tc0cn
   *
   * @param mat_x
   * @return Eigen::MatrixXd
   */
Eigen::MatrixXd V_Odom::measurement_function( const Eigen::MatrixXd &mat_x ) 
{
    Eigen::Isometry3d iso_x;
    iso_x.matrix() = mat_x;
    Eigen::Isometry3d Twb_in_V = Tvw_ * iso_x * Tcb_.inverse();
    return Twb_in_V.matrix();
}


/**
 * @brief residual = z - h(x)
 *
 * @param mat_x
 * @param mat_z
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd V_Odom::measurement_residual( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ) 
{
    Eigen::Isometry3d iso_x_in_z;
    iso_x_in_z.matrix() = measurement_function( mat_x );

    Eigen::Isometry3d iso_z;
    iso_z.matrix() = mat_z;

    Eigen::Matrix<double, kMeasDim, 1> residual;
    residual.topRows(3) = iso_z.translation() - iso_x_in_z.translation();
    residual.bottomRows(3) = State::rotation_residual( iso_z.linear(), iso_x_in_z.linear() );

    return residual;
}


/**
   * @brief h(x)/delta X or -r(x)/delta X
   *
   * @param mat_x
   * @param mat_z
   * @return Eigen::MatrixXd
   */
Eigen::MatrixXd V_Odom::measurement_jacobian( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ) 
{
    Eigen::Matrix<double, kMeasDim, kState_dim> H;
    H.setZero();

    Eigen::Isometry3d T;
    T.matrix() = mat_x;

    Eigen::Isometry3d Tvo;
    Tvo.matrix() = mat_z;

    Eigen::Quaterniond vo_q(Tvo.linear());

    const Eigen::Matrix3d &Rvw = Tvw_.linear();

    Eigen::Quaterniond q_vw(Rvw);
    Eigen::Quaterniond q_cb(Tcb_.linear());
    Eigen::Quaterniond q(T.linear());

    Eigen::Matrix4d m4;
    H.block<3, 3>(0, 0) = -Rvw;
    H.block<3, 3>(0, 6) = Rvw * T.linear() * skew_matrix(Tcb_.inverse().translation());
    m4 = quat_left_matrix((vo_q.conjugate() * q_vw * q).normalized()) * quat_right_matrix(q_cb.conjugate());
    H.block<3, 3>(3, 6) = -m4.block<3, 3>(1, 1);
    H *= -1.0;

    return H;
}


/**
   * @brief check jacobian
   * 
   * @param mat_x 
   * @param mat_z 
   */
void V_Odom::check_jacobian( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ) 
{
    Eigen::Vector3d delta(0.0012, -0.00034, -0.00056);

    Eigen::Isometry3d T0;
    T0.matrix() = mat_x;
    
    // perturbation on t
    Eigen::Isometry3d T1 = T0;
    T1.translation() += delta;

    // perturbation on R
    Eigen::Isometry3d T2 = T0;
    T2.linear() = State::rotation_update(T2.linear(), State::delta_rot_mat(delta, 1));

    Eigen::Isometry3d Tx0 = Tvw_ * T0 * Tcb_.inverse();
    Eigen::Isometry3d Tx1 = Tvw_ * T1 * Tcb_.inverse();
    Eigen::Isometry3d Tx2 = Tvw_ * T2 * Tcb_.inverse();

    const auto &H = measurement_jacobian( mat_x, mat_z );
}

}  // namespace uwb_loka

#endif // UWB_LOKA_VISUAL_ODOMETRY_IMPL_H
