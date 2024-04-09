#ifndef IMU_H
#define IMU_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <ros/ros.h>

#include "state.h"

namespace uwb_loka 
{

struct ImuData {
    double timestamp;       // In second.
    Eigen::Vector3d acc;    // Acceleration in m/s^2
    Eigen::Vector3d gyro;    // Angular velocity in radian/s.
};

using ImuData_Ptr = std::shared_ptr<ImuData>;
using ImuData_ConstPtr = std::shared_ptr<const ImuData>;

class IMU 
{
public:
    IMU( double acc_n = 1e-2, double gyro_n = 1e-4, double acc_w = 1e-6, double gyro_w = 1e-8 );

    bool push_data( ImuData_Ptr imu_ptr, const bool &inited );

    bool init( State &state, double ts_meas, ImuData_ConstPtr &last_imu_ptr );

    static bool init_rot_from_imudata(const std::deque<ImuData_ConstPtr> &imu_buf, Eigen::Matrix3d &Rwb);
    
    void propagate_state( ImuData_ConstPtr last_imu, ImuData_ConstPtr curr_imu, const State &last_state,
                          State &state, bool with_noise = true, bool with_const_noise = true,
                          const Eigen::Matrix<double, kNoise_dim, 1> &vec_noise = Eigen::Matrix<double, kNoise_dim, 1>::Zero() );

    void propagate_state_cov(ImuData_ConstPtr last_imu, ImuData_ConstPtr curr_imu, const State &last_state, State &state);

    Eigen::Matrix<double, kNoise_dim, kNoise_dim> noise_cov( double dt );

    Eigen::Matrix<double, kState_dim, kState_dim> noise_cov_discret_time( double dt );
  
private:
    static const int kImu_BufSize = 200;
    std::deque<ImuData_ConstPtr> imu_buf_;
    double acc_noise_;
    double gyro_noise_;
    double acc_bias_noise_;
    double gyro_bias_noise_;
};

}  // namespace uwb_loka

#include "imu_impl.hpp"

#endif // IMU_H
