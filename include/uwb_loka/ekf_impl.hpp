#ifndef UWB_LOKA_EKF_IMPL_H
#define UWB_LOKA_EKF_IMPL_H

#include "ekf.h"

namespace uwb_loka 
{

EKF::EKF(double acc_n, double gyro_n, double acc_w, double gyro_w )
    : KF( acc_n, gyro_n, acc_w, gyro_w ) 
{
    state_ptr_i_ = std::make_shared<State>();
}


/**
 * @brief predict procedure
 * @param last_imu
 * @param curr_imu
 */
void EKF::predict( ImuData_ConstPtr last_imu, ImuData_ConstPtr curr_imu ) 
{
    State last_state = *state_ptr_;

    state_ptr_->set_timestamp( curr_imu->timestamp );

    propagate_state( last_imu, curr_imu, last_state, *state_ptr_ );
    propagate_state_cov( last_imu, curr_imu, last_state, *state_ptr_ );
}


/// @brief 更新卡尔曼增益矩阵 K
/// @tparam H_type 
/// @tparam R_type 
/// @tparam K_type 
/// @param H    测量矩阵 H, 描述了状态变量到测量空间的映射
/// @param R    测量噪声协方差矩阵 R
/// @param K    卡尔曼增益矩阵 K
template <class H_type, class R_type, class K_type>
void EKF::update_K( const Eigen::MatrixBase<H_type> &H, const Eigen::MatrixBase<R_type> &R, Eigen::MatrixBase<K_type> &K ) 
{
    // 获取了状态协方差矩阵 P，它是 EKF 状态估计的不确定性度量
    const auto &P = state_ptr_->cov_;
    const R_type &S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();
    // const Eigen::Matrix<double, kStateDim, R_type::RowsAtCompileTime> K = P * H.transpose() * S.inverse();
}

  
template <class H_type, class R_type, class K_type>
void EKF::update_P( const Eigen::MatrixBase<H_type> &H, const Eigen::MatrixBase<R_type> &R, const Eigen::MatrixBase<K_type> &K ) 
{
    const MatrixSD &I_KH = MatrixSD::Identity() - K * H;
    state_ptr_->cov_ = I_KH * state_ptr_->cov_ * I_KH.transpose() + K * R * K.transpose();
}

}  // namespace uwb_loka

#endif // UWB_LOKA_EKF_IMPL_H