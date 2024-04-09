#ifndef UWB_LOKA_STATE_H
#define UWB_LOKA_STATE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "common.hpp"

namespace uwb_loka 
{

using MatrixSD = Eigen::Matrix<double, kState_dim, kState_dim>;

class State 
{

public:
    State();

    Eigen::Matrix3d get_Rotate() const;

    void set_Rotate( const Eigen::Matrix3d &R );

    Eigen::Vector3d get_acc_bias() const;

    Eigen::Vector3d get_gyro_bias() const;

    void set_bias( const Eigen::Vector3d &ba, const Eigen::Vector3d &bg );   

    Eigen::Vector3d get_position() const;

    void set_positon( const Eigen::Vector3d &new_posi );  

    const Eigen::Isometry3d get_pose() const;

    void set_pose(const Eigen::Isometry3d &pose);

    const Eigen::Matrix<double, 7, 1> vec_pq() const;

    const void set_vec_pq(const Eigen::Matrix<double, 7, 1> &vec_pq);

    const Eigen::Matrix<double, 9, 1> vec_vb() const;

    const void set_vec_vb(const Eigen::Matrix<double, 9, 1> &vec_vb);

    const Eigen::Matrix<double, kState_dim, 1> vec() const;

    void from_vec(const Eigen::Matrix<double, kState_dim, 1> &vec);
  
    void set_cov_( double sigma_p, double sigma_v, double sigma_rp, double sigma_yaw, double sigma_ba, double sigma_bg );

    void set_timestamp( double timestamp );
    
    const double get_timestamp() const;

    State &operator=( const State &rhs );

    State operator+( const Eigen::Matrix<double, kState_dim, 1> &delta_x ) const;

    Eigen::Matrix<double, kState_dim, 1> operator-(const State &rhs) const;

    static void update_pose(Eigen::Isometry3d &pose, const Eigen::Matrix<double, 6, 1> &delta_pose);

    static Eigen::Matrix3d delta_rot_mat(const Eigen::Vector3d &delta_rot_vec, int flag = 0);

    static Eigen::Matrix3d rotation_update(const Eigen::Matrix3d &Rwb, const Eigen::Matrix3d &delta_rot_mat);

    static Eigen::Vector3d rotation_residual(const Eigen::Matrix3d &Robs, const Eigen::Matrix3d &Rest);

    // static Eigen::Vector3d rot_mat_to_vec( const Eigen::Matrix3d &R );

    // static Eigen::Matrix3d rot_vec_to_mat( const Eigen::Vector3d &rvec );

    // error-state
    MatrixSD cov_;

public:
    // memory alignment 内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // nominal-state   
    Eigen::Vector3d p_;             // Position
    Eigen::Vector3d v_;             // Velocity
    Eigen::Matrix3d R_;             // Rotation Matrix from the IMU frame to the Global frame.

    Eigen::Vector3d acc_bias_;      // The bias of the acceleration sensor.
    Eigen::Vector3d gyro_bias_;     // The bias of the gyroscope sensor.

    double timestamp_;
};

using State_Ptr = std::shared_ptr<State>;

}  // namespace uwb_loka

#include "state_impl.hpp"

#endif // UWB_LOKA_STATE_H