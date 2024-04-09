#ifndef UWB_LOKA_EKF_H
#define UWB_LOKA_EKF_H

#include "kalman_filter.h"

namespace uwb_loka 
{

class EKF : public KF 
{

public:
    EKF() = delete;

    EKF( const EKF & ) = delete;

    ~EKF() {}

    explicit EKF( double acc_n = 1e-2, double gyro_n = 1e-4, double acc_w = 1e-6, double gyro_w = 1e-8 );

    void predict( ImuData_ConstPtr last_imu, ImuData_ConstPtr curr_imu );

    template <class H_type, class R_type, class K_type>
    void update_K( const Eigen::MatrixBase<H_type> &H, const Eigen::MatrixBase<R_type> &R, Eigen::MatrixBase<K_type> &K );
  
    template <class H_type, class R_type, class K_type>
    void update_P( const Eigen::MatrixBase<H_type> &H, const Eigen::MatrixBase<R_type> &R, const Eigen::MatrixBase<K_type> &K );

public:
    State_Ptr state_ptr_i_;  // for IEKF
    
};

using EKF_Ptr = std::unique_ptr<EKF>;

}  // namespace uwb_loka

#include "ekf_impl.hpp"

#endif // UWB_LOKA_EKF_H