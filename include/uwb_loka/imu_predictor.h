#ifndef UWB_LOKA_IMU_PREDICTOR_H
#define UWB_LOKA_IMU_PREDICTOR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <ros/ros.h>

#include "state.h"

namespace uwb_loka 
{

class IMU_Predictor 
{
public:
    IMU_Predictor() {}

    IMU_Predictor( State_Ptr &state_ptr, double acc_n = 1e-2, double gyro_n = 1e-4, double acc_w = 1e-6, double gyro_w = 1e-8 );

    IMU_Predictor( const IMU_Predictor & ) = delete;

    virtual ~IMU_Predictor() {}

    bool init( double ts_meas );

    // void imu_callback( const sensor_msgs::ImuConstPtr &imu_msg );

    virtual void predict( ImuData_ConstPtr last_imu, ImuData_ConstPtr curr_imu ) = 0;

    bool push_data( ImuData_Ptr imu_ptr, const bool &inited );

    static bool init_rot_from_imudata(const std::deque<ImuData_ConstPtr> &imu_buf, Eigen::Matrix3d &Rwb);
    
    void propagate_state( ImuData_ConstPtr last_imu, ImuData_ConstPtr curr_imu, const State &last_state,
                          State &state, bool with_noise = true, bool with_const_noise = true,
                          const Eigen::Matrix<double, kNoise_dim, 1> &vec_noise = Eigen::Matrix<double, kNoise_dim, 1>::Zero() );

    void propagate_state_cov(ImuData_ConstPtr last_imu, ImuData_ConstPtr curr_imu, const State &last_state, State &state);

    Eigen::Matrix<double, kNoise_dim, kNoise_dim> noise_cov( double dt );

    Eigen::Matrix<double, kState_dim, kState_dim> noise_cov_discret_time( double dt );
  
private:
    bool init( State &state, double ts_meas, ImuData_ConstPtr &last_imu_ptr );

    static const int kImu_BufSize = 200;
    std::deque<ImuData_ConstPtr> imu_buf_;
    double acc_noise_;
    double gyro_noise_;
    double acc_bias_noise_;
    double gyro_bias_noise_;

    // test
    bool einmal_test_ = false;

public:
    bool inited_ = false;

    ImuData_ConstPtr last_imu_ptr_;

private:
    State_Ptr state_p_;
};

using IMU_Predictor_Ptr = std::unique_ptr<IMU_Predictor>;


}  // namespace uwb_loka

#include "imu_predictor_impl.hpp"

#endif // UWB_LOKA_IMU_PREDICTOR_H
