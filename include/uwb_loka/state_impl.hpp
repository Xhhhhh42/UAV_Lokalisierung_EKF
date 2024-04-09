#ifndef UWB_LOKA_STATE_IMPL_H
#define UWB_LOKA_STATE_IMPL_H

#include "state.h"

namespace uwb_loka 
{

State::State() 
{
    cov_.setZero();
    p_.setZero();
    v_.setZero();
    R_.setIdentity();
    acc_bias_.setZero();
    gyro_bias_.setZero();
    // Ang_Error_ = LOCAL_ANGULAR_ERROR;
}


Eigen::Matrix3d State::get_Rotate() const
{
    Eigen::Matrix3d R = R_;
    return R;
}


void State::set_Rotate( const Eigen::Matrix3d &R )
{
    R_ = R;
}


Eigen::Vector3d State::get_acc_bias() const
{
    Eigen::Vector3d acc_bias;
    acc_bias = acc_bias_;
    return acc_bias;
}


Eigen::Vector3d State::get_gyro_bias() const
{
    Eigen::Vector3d gyro_bias;
    gyro_bias = gyro_bias_;
    return gyro_bias;
}


/// @brief 设置 State 对象的加速度偏差（acc_bias_）和陀螺仪偏差（gyro_bias_）
/// @param ba 引用参数，表示加速度偏差的向量
/// @param bg 引用参数，表示陀螺仪偏差的向量
void State::set_bias( const Eigen::Vector3d &ba, const Eigen::Vector3d &bg ) 
{
    acc_bias_ = ba;
    gyro_bias_ = bg;
}


Eigen::Vector3d State::get_position() const
{
    Eigen::Vector3d p;
    p = p_;
    return p;
}


void State::set_positon( const Eigen::Vector3d &new_posi )
{
    p_ = new_posi;
}


/// @brief 获取 State 对象的姿态（pose）信息
/// @return 返回一个 Eigen::Isometry3d 类型的对象
const Eigen::Isometry3d State::get_pose() const 
{
    // 存储从世界坐标系（World Frame）到机体坐标系（Body Frame）的变换矩阵。
    Eigen::Isometry3d Twb;
    Twb.linear() = R_;
    Twb.translation() = p_;
    return Twb;
}


void State::set_pose( const Eigen::Isometry3d &pose ) 
{
    R_ = pose.linear();
    p_ = pose.translation();
}


/// @brief 这个函数vec_pq返回一个长度为7的列向量，表示状态中的位置和姿态
/// @return 
const Eigen::Matrix<double, 7, 1> State::vec_pq() const 
{
    Eigen::Matrix<double, 7, 1> vec;

    vec.head(3) = p_;
    const auto &q = Eigen::Quaterniond( R_ );
    // 将四元数q的向量部分（旋转轴的三个分量）分配给vec的第4到第6个元素，以表示旋转
    vec.segment<3>(3) = q.vec();
    vec(6) = q.w();

    return vec;
}


const void State::set_vec_pq( const Eigen::Matrix<double, 7, 1> &vec_pq ) 
{
    p_ = vec_pq.head(3);
    Eigen::Quaterniond q;
    q.vec() = vec_pq.segment<3>(3);
    q.w() = vec_pq(6);
    R_ = q.toRotationMatrix();
    // R_ = Eigen::Quaterniond(vec_pq.tail(4).data()).toRotationMatrix();  // xyzw
}


/// @brief 函数vec_vb返回一个长度为9的列向量，表示状态中的速度和传感器的偏置（加速度和角速度）
/// @return 
const Eigen::Matrix<double, 9, 1> State::vec_vb() const 
{
    Eigen::Matrix<double, 9, 1> vec;

    vec.segment<3>(0) = v_;
    vec.segment<3>(3) = acc_bias_;
    vec.segment<3>(6) = gyro_bias_;

    return vec;
}


const void State::set_vec_vb( const Eigen::Matrix<double, 9, 1> &vec_vb ) 
{
    v_ = vec_vb.segment<3>(0);
    acc_bias_ = vec_vb.segment<3>(3);
    gyro_bias_ = vec_vb.segment<3>(6);
}


/// @brief 函数vec返回一个长度为kState_dim的列向量，表示状态中的位置、速度、姿态、加速度偏置和角速度偏置
/// @return 
const Eigen::Matrix<double, kState_dim, 1> State::vec() const 
{
    Eigen::Matrix<double, kState_dim, 1> vec;

    vec.segment<3>(0) = p_;
    vec.segment<3>(3) = v_;
    vec.segment<3>(6) = rot_mat_to_vec( R_ );
    vec.segment<3>(9) = acc_bias_;
    vec.segment<3>(12) = gyro_bias_;

    return vec;
}


void State::from_vec(const Eigen::Matrix<double, kState_dim, 1> &vec) {
    p_ = vec.segment<3>(0);
    v_ = vec.segment<3>(3);
    R_ = rot_vec_to_mat( vec.segment<3>(6) );
    acc_bias_ = vec.segment<3>(9);
    gyro_bias_ = vec.segment<3>(12);
}


/**
 * @brief Set the cov_ object
 *
 * @param sigma_p       pos std, m
 * @param sigma_v       vel std, m/s
 * @param sigma_rp      roll / pitch std
 * @param sigma_yaw     yaw std
 * @param sigma_ba      Acc bias
 * @param sigma_bg      Gyro bias
 */
void State::set_cov_( double sigma_p, double sigma_v, double sigma_rp, double sigma_yaw, double sigma_ba, double sigma_bg ) 
{
    cov_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sigma_p * sigma_p;
    cov_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sigma_v * sigma_v;
    cov_.block<2, 2>(6, 6) = Eigen::Matrix2d::Identity() * sigma_rp * sigma_rp;
    cov_(8, 8) = sigma_yaw * sigma_yaw;
    cov_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * sigma_ba * sigma_ba;
    cov_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * sigma_bg * sigma_bg;
}


void State::set_timestamp( double timestamp )
{
    timestamp_ = timestamp;
}


const double State::get_timestamp() const
{
    double timestamp = timestamp_;
    return timestamp;    
}


/// @brief 右操作数 rhs 的状态信息赋值给当前对象
/// @param rhs 
/// @return 
State &State::operator=( const State &rhs ) 
{
    if( this == &rhs ) return *this;
    p_ = rhs.p_;
    v_ = rhs.v_;
    R_ = rhs.R_;
    acc_bias_ = rhs.acc_bias_;
    gyro_bias_ = rhs.gyro_bias_;
    return *this;
}


State State::operator+( const Eigen::Matrix<double, kState_dim, 1> &delta_x ) const 
{
    State state;
    state.p_ = this->p_ + delta_x.block<3, 1>(0, 0);
    state.v_ = this->v_ + delta_x.block<3, 1>(3, 0);
    state.R_ = rotation_update( this->R_, delta_rot_mat( delta_x.block<3, 1>(6, 0) ));
    state.acc_bias_ = this->acc_bias_ + delta_x.block<3, 1>(9, 0);
    state.gyro_bias_ = this->gyro_bias_ + delta_x.block<3, 1>(12, 0);
    return state;
}

  
Eigen::Matrix<double, kState_dim, 1> State::operator-( const State &rhs ) const 
{
    Eigen::Matrix<double, kState_dim, 1> delta_x;
    delta_x.block<3, 1>(0, 0) = this->p_ - rhs.p_;
    delta_x.block<3, 1>(3, 0) = this->v_ - rhs.v_;
    delta_x.block<3, 1>(6, 0) = rotation_residual(this->R_, rhs.R_);
    delta_x.block<3, 1>(9, 0) = this->acc_bias_ - rhs.acc_bias_;
    delta_x.block<3, 1>(12, 0) = this->gyro_bias_ - rhs.gyro_bias_;
    return delta_x;
}


/// @brief 这个函数update_pose接受一个Eigen::Isometry3d对象pose和一个表示小位姿更新的6元素向量delta_pose
/// @param pose 
/// @param delta_pose 
void State::update_pose( Eigen::Isometry3d &pose, const Eigen::Matrix<double, 6, 1> &delta_pose ) 
{
    // noalias 是 Eigen 库中的一个优化标记（Optimization Tag），用于指示不要使用临时变量进行中间计算，而是在原地直接进行运算，以提高计算效率和减少内存分配
    pose.translation().noalias() += delta_pose.head(3);
    pose.linear() = rotation_update( pose.rotation(), delta_rot_mat(delta_pose.tail(3) ));
}


/**
 * @brief 该函数接受一个三维向量 delta_rot_vec 和一个整数 flag 作为参数，并返回一个三维旋转矩阵 deltaR。
 *
 * @param delta_rot_vec small rotation vector
 * @param flag 0: angle axis, 1: quaternion
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d State::delta_rot_mat( const Eigen::Vector3d &delta_rot_vec, int flag ) 
{
    Eigen::Matrix3d deltaR = Eigen::Matrix3d::Identity();
    if ( flag == 0 && delta_rot_vec.norm() > std::numeric_limits<double>::epsilon() )  {
        // todo！
        // deltaR = Utils::rot_vec_to_mat(delta_rot_vec);
        deltaR = rot_vec_to_mat( delta_rot_vec );
    }
    if ( flag == 1 ) {
        Eigen::Quaterniond delta_q;
        delta_q.w() = 1;
        delta_q.vec() = 0.5 * delta_rot_vec;
        deltaR = delta_q.toRotationMatrix();
    }
    // ROS_INFO_STREAM( "deltaR: \n" << deltaR );
    return deltaR;
}


/**
 * @brief Rwb + delta_rot_mat
 *
 * @details Rwb: 1) active 2) local-to-global
 *
 * @param Rwb
 * @param delta_rot_mat small rotation matrix, local or global perturbation
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d State::rotation_update(const Eigen::Matrix3d &Rwb, const Eigen::Matrix3d &delta_rot_mat) 
{
    Eigen::Matrix3d updatedR = Eigen::Matrix3d::Identity();
    // switch ( Ang_Error_ ) {
    //     case ANGULAR_ERROR::LOCAL_ANGULAR_ERROR:
    //         updatedR = Rwb * delta_rot_mat;
    //         break;
    //     case ANGULAR_ERROR::GLOBAL_ANGULAR_ERROR:
    //         updatedR = delta_rot_mat * Rwb;
    //         break;
    // }
    updatedR = Rwb * delta_rot_mat;
    return updatedR;
}


/**
 * @brief Robs - Rest
 *
 * @details Robs, Rest: 1) active 2) local-to-global
 *
 * @param Robs
 * @param Rest
 * @return Eigen::Vector3d
 */
Eigen::Vector3d State::rotation_residual( const Eigen::Matrix3d &Robs, const Eigen::Matrix3d &Rest ) 
{
    Eigen::Quaterniond q_res;
    // switch (Ang_Error_) {
    //     case ANGULAR_ERROR::LOCAL_ANGULAR_ERROR:
    //         q_res = Eigen::Quaterniond( Rest.transpose() * Robs );
    //         break;
    //     case ANGULAR_ERROR::GLOBAL_ANGULAR_ERROR:
    //         q_res = Eigen::Quaterniond( Robs * Rest.transpose() );
    //         break;
    // }
    q_res = Eigen::Quaterniond( Rest.transpose() * Robs );
    return 2.0 * q_res.vec() / q_res.w();
}

}  // namespace uwb_loka

#endif // UWB_LOKA_STATE_IMPL_H