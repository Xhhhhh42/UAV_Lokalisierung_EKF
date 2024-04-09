#ifndef UWB_LOKA_PREDICTOR_IMPL_HPP
#define UWB_LOKA_PREDICTOR_IMPL_HPP

#include <sensor_msgs/Imu.h>

#include "predictor.h"

namespace uwb_loka 
{

Predictor::Predictor( State_Ptr &state_ptr, double acc_n, double gyro_n, double acc_w, double gyro_w )
    : state_p_( state_ptr ), 
      imu_model_( acc_n, gyro_n, acc_w, gyro_w ) 
{

}


bool Predictor::init( double ts_meas ) 
{ 
    return inited_ = imu_model_.init( *state_p_, ts_meas, last_imu_ptr_ ); 
}


void Predictor::imu_callback( const sensor_msgs::ImuConstPtr &imu_msg ) 
{
    ImuData_Ptr imu_data_ptr = std::make_shared<ImuData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg->linear_acceleration.x, 
                         imu_msg->linear_acceleration.y,
                         imu_msg->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg->angular_velocity.x,
                          imu_msg->angular_velocity.y,
                          imu_msg->angular_velocity.z;

    // ROS_INFO_STREAM ( "Time: " << imu_data_ptr->timestamp );
    // ROS_INFO("Time : %d", imu_data_ptr->timestamp );
    // ros::Time time_obj(1709596391, 92369657);  // 92369657是纳秒部分
    // ROS_INFO("Time: %f", time_obj.toSec());
    // // 或者
    // ROS_INFO_STREAM("Time: " << time_obj.toSec());
    // ROS_INFO_STREAM("Time: " << time_obj.sec << "." << std::setfill('0') << std::setw(9) << time_obj.nsec);
    // double timestamp = 1709596391.092369657;
    // ROS_INFO_STREAM(std::fixed << std::setprecision(9) << imu_data_ptr->timestamp);

    if( !imu_model_.push_data( imu_data_ptr, inited_ )) return;
    
    // if( !inited_ ) {
    //     last_imu_ptr_ = imu_data_ptr;
    //     if( !init( imu_data_ptr->timestamp )) return;
    //     ROS_INFO( "init SUCCESS" );
    //     return;
    // }

    predict( last_imu_ptr_, imu_data_ptr );
    last_imu_ptr_ = imu_data_ptr;
}

}  // namespace uwb_loka

#endif // UWB_LOKA_PREDICTOR_IMPL_HPP