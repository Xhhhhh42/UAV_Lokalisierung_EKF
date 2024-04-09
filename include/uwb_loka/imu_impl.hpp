#ifndef IMU_IMPL_HPP
#define IMU_IMPL_HPP

#include "imu.h"
#include "common.hpp"

namespace uwb_loka 
{

IMU::IMU( double acc_n, double gyro_n, double acc_w, double gyro_w )
    : acc_noise_( acc_n ), 
      gyro_noise_( gyro_n ), 
      acc_bias_noise_( acc_w ), 
      gyro_bias_noise_( gyro_w ) 
{
    // // ROS_INFO( "IMU::init..." );
    // Eigen::Vector3d rvec = { M_PI / 100, M_PI / 100, 0};
    // ROS_INFO_STREAM( "test_rot_vec_to_mat" << rot_vec_to_mat(rvec) );
    // ROS_INFO_STREAM( "test_rot_mat_to_vec" << rot_mat_to_vec(rot_vec_to_mat(rvec)) );
    // ROS_INFO_STREAM( "test_vec" << rvec );

    // ROS_INFO( "-----------------------------------------" );

    // Eigen::Vector3d rvec_2 = { 0, M_PI / 4, M_PI / 4 };
    // ROS_INFO_STREAM( "test_rot_vec_to_mat_2" << rot_vec_to_mat(rvec_2) );
    // // ROS_INFO_STREAM( "test_rot_mat_to_vec" << rot_mat_to_vec(rot_vec_to_mat(rvec)) );
    // // ROS_INFO_STREAM( "test_vec" << rvec );

    // ROS_INFO( "-----------------------------------------" );

    // Eigen::Matrix3d rotation_matrix;
    // rotation_matrix = Eigen::AngleAxisd(rvec.norm(), rvec.normalized()).toRotationMatrix();
    // Eigen::AngleAxisd angleAxis(rotation_matrix);
    // Eigen::Vector3d res = angleAxis.angle() * angleAxis.axis();
    // ROS_INFO_STREAM( "test_rot_vec_to_mat" << rotation_matrix );
    // ROS_INFO_STREAM( "test_rot_mat_to_vec" << res );
    // ROS_INFO_STREAM( "test_vec" << rvec );
}


/// @brief 将 IMU 数据加入到 IMU 缓冲区中，并在需要时移除异常值。同时根据系统是否已初始化来决定是否执行加入缓冲区的操作
/// @param imu_ptr 
/// @param inited 
/// @return 
bool IMU::push_data( ImuData_Ptr imu_ptr, const bool &inited ) 
{
    // remove spikes 移除异常数据
    static Eigen::Vector3d last_a_m = Eigen::Vector3d::Zero();

    if( imu_ptr->acc.norm() > kAcc_GravityLimit * G ) { imu_ptr->acc = last_a_m; } 
    else { last_a_m = imu_ptr->acc; }

    // imu_buf_.push_back( imu_ptr );
    // if ( imu_buf_.size() > kImu_BufSize ) { 
    //     imu_buf_.pop_front(); 
    //     return true;
    // } else return false;

    if( !inited ) {
      imu_buf_.push_back( imu_ptr );
      if( imu_buf_.size() > kImu_BufSize ) imu_buf_.pop_front();
      return false;
    }

    return true;
}


/// @brief 初始化IMU状态，并接收当前时间戳 ts_meas 以及指向上一个IMU数据的指针 last_imu_ptr
/// @param state 
/// @param ts_meas 当前时间戳
/// @param last_imu_ptr 
/// @return 
bool IMU::init( State &state, double ts_meas, ImuData_ConstPtr &last_imu_ptr ) 
{
    // ROS_INFO( "IMU::init" );
    // 检查IMU缓冲区中的数据是否足够进行初始化，如果不足，则输出警告消息
    if ( imu_buf_.size() < kImu_BufSize ) {
        ROS_WARN_STREAM_NAMED( "IMU", "Initialization ERROR: Not Enough IMU data for Initialization!!!" );
        return false;
    }

    // 如果数据足够，将 last_imu_ptr 设置为IMU缓冲区中的最后一个数据，表示使用最新的数据进行初始化
    last_imu_ptr = imu_buf_.back();
    // ROS_INFO_STREAM( "last_imu_ptr" << std::fixed << std::setprecision(9) << last_imu_ptr->timestamp);
    // ROS_INFO_STREAM( "ts_meas" << std::fixed << std::setprecision(9) << ts_meas );

    // 检查当前时间戳 ts_meas 与 last_imu_ptr 中记录的时间戳之间的差值是否大于0.05秒
    if ( std::abs( ts_meas - last_imu_ptr->timestamp ) > 0.05 ) {
        ROS_WARN_STREAM_NAMED( "IMU", "Initialization ERROR: timestamps are not synchronized!!!" );
        return false;
    }

    double temp_stamp = last_imu_ptr->timestamp;
    state.set_timestamp( temp_stamp );
    Eigen::Matrix3d R = state.get_Rotate();
    // ROS_INFO_STREAM("R: \n" << std::fixed << std::setprecision(9) << R);
    init_rot_from_imudata( imu_buf_, R );
    // ROS_INFO_STREAM("R: \n" << R);
    state.set_Rotate( R );

    return true;
}


/// @brief 根据一系列 IMU 数据来初始化旋转矩阵 R，表示从 IMU 坐标系到世界坐标系的变换
/// @param imu_buf 
/// @param R 
/// @return 
bool IMU::init_rot_from_imudata( const std::deque<ImuData_ConstPtr> &imu_buf, Eigen::Matrix3d &R ) 
{
    // Compute mean and std of the imu buffer.
    Eigen::Vector3d sum_acc( 0., 0., 0. );
    for ( const auto imu_data : imu_buf ) {
      sum_acc += imu_data->acc;
    }

    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buf.size();

    ROS_INFO_STREAM_NAMED( "IMU-Function : Init_rot_from_imudata", 
                           "Mean_acceleration: ( " << mean_acc[0] << ", " << mean_acc[1] << ", " << mean_acc[2] << " )!" );

    // 所有 IMU 数据的加速度误差的平方和
    Eigen::Vector3d sum_err2( 0., 0., 0. );
    for ( const auto imu_data : imu_buf ) {
        // cwiseAbs2() 返回一个新的向量，其中每个元素都是原向量中对应位置元素的绝对值的平方
        sum_err2 += ( imu_data->acc - mean_acc ).cwiseAbs2();
    }

    const Eigen::Vector3d std_acc = ( sum_err2 / (double)imu_buf.size() ).cwiseSqrt();

    // acc std limit: 3
    if ( std_acc.maxCoeff() > kAccStdLimit ) {
        ROS_WARN_STREAM_NAMED( "IMU-Function : Init_rot_from_imudata", 
                               "Error: Too big acc std: ( " << std_acc[0] << ", " << std_acc[1] << ", " << std_acc[2] << " )!" );
        return false;
    }

    // Compute rotation
    // ref: https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp

    // Three axises of the ENU frame in the IMU frame.
    // 计算从IMU坐标系到地球参考框架(ENU框架)的旋转矩阵
    // z-axis
    // const Eigen::Vector3d refe_acc = Eigen::Vector3d( 0, 0, G );
    // const Eigen::Vector3d &z_axis = refe_acc.normalized();
    const Eigen::Vector3d &z_axis = mean_acc.normalized();

    // ROS_INFO_STREAM_NAMED( "IMU-Function : Init_rot_from_imudata", 
    //                        "z_axis: ( " << z_axis[0] << ", " << z_axis[1] << ", " << z_axis[2] << " )!" );

    // x-axis --- Schmidt正交化
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // ROS_INFO_STREAM_NAMED( "IMU-Function : Init_rot_from_imudata", 
    //                        "x_axis: ( " << x_axis[0] << ", " << x_axis[1] << ", " << x_axis[2] << " )!" );

    // y-axis
    Eigen::Vector3d y_axis = z_axis.cross( x_axis );
    y_axis.normalize();

    // ROS_INFO_STREAM_NAMED( "IMU-Function : Init_rot_from_imudata", 
    //                        "y_axis: ( " << y_axis[0] << ", " << y_axis[1] << ", " << y_axis[2] << " )!" );

    Eigen::Matrix3d Rbw;
    Rbw.block<3,1>(0, 0) = x_axis;
    Rbw.block<3,1>(0, 1) = y_axis;
    Rbw.block<3,1>(0, 2) = z_axis;

    // ROS_INFO_STREAM("Rbw: \n" << Rbw);

    R = Rbw.transpose();

    // ROS_INFO_STREAM("R: \n" << R);
    // ROS_INFO_STREAM("R*ACC_MEANS: \n" << R*mean_acc);

    return true;
}


/**
 * @brief
 *
 * @ref ESKF 5.4.1 The nominal state kinematics (without noise)
 *
 * @param last_imu
 * @param curr_imu
 * @param last_state
 * @param state
 * @param vec_na 加速度噪声向量
 * @param vec_ng 陀螺仪噪声向量
 * @param vec_wa 加速度偏置噪声向量
 * @param vec_wg 陀螺仪偏置噪声向量
 */
void IMU::propagate_state( ImuData_ConstPtr last_imu, ImuData_ConstPtr curr_imu, const State &last_state, State &state, 
                           bool with_noise, bool with_const_noise, const Eigen::Matrix<double, kNoise_dim, 1> &vec_noise ) 
{
    const double dt = curr_imu->timestamp - last_imu->timestamp;
    const double dt2 = dt * dt;

    Eigen::Vector3d vec_na = Eigen::Vector3d::Zero();
    Eigen::Vector3d vec_ng = Eigen::Vector3d::Zero();
    Eigen::Vector3d vec_wa = Eigen::Vector3d::Zero();
    Eigen::Vector3d vec_wg = Eigen::Vector3d::Zero();

    if ( with_noise ) {
        // TODO: check vec_noise empty or not
        if ( with_const_noise ) {
            vec_na = Eigen::Vector3d( acc_noise_, acc_noise_, acc_noise_);
            vec_ng = Eigen::Vector3d( gyro_noise_, gyro_noise_, gyro_noise_);
            vec_wa = Eigen::Vector3d( acc_bias_noise_, acc_bias_noise_, acc_bias_noise_);
            vec_wg = Eigen::Vector3d( gyro_bias_noise_, gyro_bias_noise_, gyro_bias_noise_);
        } else {
            vec_na = vec_noise.segment<3>(0);
            vec_ng = vec_noise.segment<3>(3);
            vec_wa = vec_noise.segment<3>(6);
            vec_wg = vec_noise.segment<3>(9);
        }
    }

    // 计算了未校准的加速度和陀螺仪测量值
    const Eigen::Vector3d acc_unbias = 0.5 * ( last_imu->acc + curr_imu->acc ) - last_state.get_acc_bias() - vec_na;
    const Eigen::Vector3d gyro_unbias = 0.5 * ( last_imu->gyro + curr_imu->gyro ) - last_state.get_gyro_bias() - vec_ng;

    // ROS_INFO_STREAM( "acc_unbias" << ": ax = " <<  acc_unbias[0] << ", ay = " <<  acc_unbias[1] 
    //                 << " , az = " <<  acc_unbias[2] );

    const Eigen::Vector3d acc_nominal = last_state.get_Rotate() * acc_unbias + Eigen::Vector3d( 0, 0, -G );

    // ROS_INFO_STREAM("R: \n" << last_state.get_Rotate());
    // ROS_INFO_STREAM("R*acc_unbias: \n" << last_state.get_Rotate() * acc_unbias);

    // ROS_INFO_STREAM( "acc_nominal" << ": ax = " << acc_nominal[0] << ", ay = " << acc_nominal[1] << " , az = " << acc_nominal[2] );

    const Eigen::Vector3d delta_angle_axis = gyro_unbias * dt;
    if( delta_angle_axis.norm() > 1e-12 ){
        const Eigen::Matrix3d dR = State::delta_rot_mat( delta_angle_axis );
        Eigen::Matrix3d Rwb = last_state.get_Rotate();
        // ROS_INFO_STREAM( "Rwb: \n" << Rwb );
        // ROS_INFO_STREAM( "dR: \n" << dR );
        state.set_Rotate( State::rotation_update( Rwb, dR ));
        // ROS_INFO_STREAM( "update_Rwb: \n" << State::rotation_update( Rwb, dR ) );
    } 
    
    // ROS_INFO_STREAM( "gyro_unbias: \n" << gyro_unbias * dt );
    // ROS_INFO_STREAM( "dt: \n" << dt );
    
    Eigen::Matrix<double, 9, 1> last_state_vec = last_state.vec_vb();
    Eigen::Matrix<double, 9, 1> state_vec;
    const Eigen::Vector3d new_posi = last_state_vec.segment<3>(0) + last_state_vec.segment<3>(3) * dt + 0.5 * acc_nominal * dt2;
    state.set_positon( new_posi );
    state_vec.segment<3>(0) = last_state_vec.segment<3>(0)+ acc_nominal * dt;
    state_vec.segment<3>(3) = last_state_vec.segment<3>(3) + vec_wa * dt;
    state_vec.segment<3>(6) = last_state_vec.segment<3>(6) + vec_wg * dt;
    state.set_vec_vb( state_vec );
}


/**
 * @brief
 *
 * @ref ESKF
 *      5.4.3 (local angular error)
 *      7.2.3 (global angular error)
 *      The error-state Jacobian and perturbation matrices
 * @param last_imu
 * @param curr_imu
 * @param last_state
 * @param state
 */
void IMU::propagate_state_cov(ImuData_ConstPtr last_imu, ImuData_ConstPtr curr_imu, const State &last_state, State &state ) 
{
    const double dt = curr_imu->timestamp - last_imu->timestamp;

    // Eigen::Matrix<double, kState_dim, 1> last_state_vec = last_state.vec();

    const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + curr_imu->acc) - last_state.get_acc_bias();
    const Eigen::Vector3d gyro_unbias = 0.5 * (last_imu->gyro + curr_imu->gyro) - last_state.get_gyro_bias();

    const Eigen::Vector3d delta_angle_axis = gyro_unbias * dt;
    const auto &dR = State::delta_rot_mat( delta_angle_axis );

    // Fx
    MatrixSD Fx = MatrixSD::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(3, 6) = -state.get_Rotate() * skew_matrix( acc_unbias ) * dt;
    Fx.block<3, 3>(3, 9) = -state.get_Rotate() * dt;
    if ( delta_angle_axis.norm() > 1e-12 ) { Fx.block<3, 3>(6, 6) = dR.transpose(); } 
    Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

    // P: error-state covariance
    state.cov_ = Fx * last_state.cov_ * Fx.transpose() + noise_cov_discret_time( dt );
}


Eigen::Matrix<double, kNoise_dim, kNoise_dim> IMU::noise_cov( double dt ) 
{
    const double dt2 = dt * dt;
    Eigen::Matrix<double, kNoise_dim, kNoise_dim> Q_i = Eigen::Matrix<double, kNoise_dim, kNoise_dim>::Zero();
    Q_i.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt2 * acc_noise_;
    Q_i.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dt2 * gyro_noise_;
    Q_i.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * dt * acc_bias_noise_;
    Q_i.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * dt * gyro_bias_noise_;
    return Q_i;
}


Eigen::Matrix<double, kState_dim, kState_dim> IMU::noise_cov_discret_time( double dt ) 
{
    Eigen::Matrix<double, kState_dim, kNoise_dim> F_i = Eigen::Matrix<double, kState_dim, kNoise_dim>::Zero();
    F_i.block<12, kNoise_dim>(3, 0) = Eigen::Matrix<double, 12, kNoise_dim>::Identity();
    return F_i * noise_cov( dt ) * F_i.transpose();
}

}  // namespace uwb_loka

#endif // IMU_IMPL_HPP
