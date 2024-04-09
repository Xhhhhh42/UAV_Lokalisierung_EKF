#ifndef UWB_LOKA_COMMON_H
#define UWB_LOKA_COMMON_H

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>


namespace uwb_loka
{

constexpr double G = 9.81007;

constexpr double kDegreeToRadian = M_PI / 180.0;
constexpr double kRadianToDegree = 180. / M_PI;

constexpr int kState_dim = 15;
constexpr int kNoise_dim = 12;
constexpr int kMeas_dim = 3;

constexpr int kImuData_BufferLength = 200;
constexpr int kAccStdLimit = 3.;
constexpr int kAcc_GravityLimit = 3.;


struct PolarCoordinate {
    float r;
    float theta; // 极角，单位为度
};


struct CartesianCoordinate {
    float x; 
    float y; 
};


struct ImuData {
    double timestamp;       // In second

    Eigen::Vector3d acc;    // Acceleration in m/s^2
    Eigen::Vector3d gyro;   // Angular velocity in radian/s
};
using ImuData_Ptr = std::shared_ptr<ImuData>;
using ImuData_ConstPtr = std::shared_ptr<const ImuData>;


struct GpsData {
    double timestamp;       // In second

    Eigen::Vector3d lla;    // Latitude in degree, longitude in degree, and altitude in meter
    Eigen::Matrix3d cov;    // Covariance in m^2
};
using GpsData_Ptr = std::shared_ptr<GpsData>;
using GpsData_ConstPtr = std::shared_ptr<const GpsData>;


static Eigen::Vector3d rot_mat_to_vec( const Eigen::Matrix3d &R ) 
{
    // 旋转矩阵转换为旋转向量
    Eigen::AngleAxisd rotation_vec;
    rotation_vec.fromRotationMatrix( R );

    double angle = rotation_vec.angle();
    Eigen::Vector3d axis = rotation_vec.axis();
    Eigen::Vector3d res = rotation_vec.angle() * rotation_vec.axis();
    return res;

    // Eigen::Vector3d eulerAngle = R.eulerAngles( 2, 1, 0 ); // ZYX顺序，yaw,pitch,roll
    // return eulerAngle;
}


static Eigen::Matrix3d rot_vec_to_mat( const Eigen::Vector3d &rvec ) {
    return Eigen::AngleAxisd( rvec.norm(), rvec.normalized() ).toRotationMatrix();
    // Eigen::Matrix3d rotation_matrix;
    // rotation_matrix = Eigen::AngleAxisd( rvec[2], Eigen::Vector3d::UnitZ()) *
    //                    Eigen::AngleAxisd( rvec[1], Eigen::Vector3d::UnitY()) *
    //                    Eigen::AngleAxisd( rvec[0], Eigen::Vector3d::UnitX());
    // return rotation_matrix;
}


/**
 * @brief 生成一个给定向量 v 的斜对称矩阵。
 *
 * @details 斜对称矩阵的特性是主对角线元素为零，而非对角线元素是由向量 v 的分量按特定顺序组成的
 *
 * @param v
 * @return Eigen::Matrix3d
 */
static Eigen::Matrix3d skew_matrix( const Eigen::Vector3d &v ) 
{
    Eigen::Matrix3d w;
    w <<  0.,    -v(2),  v(1),
          v(2),  0.,     -v(0),
          -v(1), v(0),   0.;
    return w;
}


/**
   * @brief 接收一个四元数q，并返回一个Eigen::Matrix4d类型的矩阵,
   *        矩阵在四元数代数中被用来表示四元数的左乘操作
   *
   * @param q
   * @return Eigen::Matrix4d
   */
static Eigen::Matrix4d quat_left_matrix( const Eigen::Quaterniond &q ) 
{
    Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
    m4.block<3, 1>(1, 0) = q.vec();
    m4.block<1, 3>(0, 1) = -q.vec();
    m4.block<3, 3>(1, 1) = skew_matrix(q.vec());
    m4 += Eigen::Matrix4d::Identity() * q.w();
    return m4;
}


/**
   * @brief 接收一个四元数q，并返回一个Eigen::Matrix4d类型的矩阵,
   *        矩阵在四元数代数中被用来表示四元数的右乘操作
   *
   * @param q
   * @return Eigen::Matrix4d
   */
static Eigen::Matrix4d quat_right_matrix( const Eigen::Quaterniond &q ) 
{
    Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
    m4.block<3, 1>(1, 0) = q.vec();
    m4.block<1, 3>(0, 1) = -q.vec();
    m4.block<3, 3>(1, 1) = -skew_matrix(q.vec());
    m4 += Eigen::Matrix4d::Identity() * q.w();
    return m4;
}


static void polarToCartesian( const PolarCoordinate& polar, CartesianCoordinate &cart ) 
{
    float theta_rad = polar.theta * kDegreeToRadian; 
    cart.x = polar.r * cos( theta_rad );
    cart.y = polar.r * sin( theta_rad );
}


static Eigen::Matrix4d getKalibrStyleTransform( const ros::NodeHandle &nh, const std::string &field ) 
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();  // Initialize with identity
    XmlRpc::XmlRpcValue lines;
    if( !nh.getParam( field, lines )) {
        throw std::runtime_error( "Cannot find transform " + field );
    }

    if( lines.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
        throw std::runtime_error( "Invalid transform " + field );
    }

    if( lines.size() != 4 ) {
        ROS_INFO_STREAM( "size" << lines.size() );
        throw std::runtime_error( "Invalid transform _____b" + field );
    }

    for( int i = 0; i < 4; ++i ) 
    {
        if( lines[i].getType() != XmlRpc::XmlRpcValue::TypeArray || lines[i].size() != 4 ) {
            throw std::runtime_error( "bad line for transform " + field );
        }

        for( int j = 0; j < lines[i].size(); ++j ) 
        {
            if( lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble ) {
                throw std::runtime_error( "bad value for transform " + field );
            }
            T(i, j) = static_cast<double>( lines[i][j] );
        }
    }
    return T;
}


static Eigen::Isometry3d getTransformEigen( const ros::NodeHandle &nh, const std::string &field ) 
{
    Eigen::Isometry3d T_isometry3d;
    Eigen::Matrix4d T = getKalibrStyleTransform( nh, field );

    T_isometry3d.linear()(0, 0) = T(0, 0);
    T_isometry3d.linear()(0, 1) = T(0, 1);
    T_isometry3d.linear()(0, 2) = T(0, 2);
    T_isometry3d.linear()(1, 0) = T(1, 0);
    T_isometry3d.linear()(1, 1) = T(1, 1);
    T_isometry3d.linear()(1, 2) = T(1, 2);
    T_isometry3d.linear()(2, 0) = T(2, 0);
    T_isometry3d.linear()(2, 1) = T(2, 1);
    T_isometry3d.linear()(2, 2) = T(2, 2);
    T_isometry3d.translation()(0) = T(0, 3);
    T_isometry3d.translation()(1) = T(1, 3);
    T_isometry3d.translation()(2) = T(2, 3);
    return T_isometry3d;
  }

} // namespace uwb_loka

#endif // UWB_LOKA_COMMON_H
