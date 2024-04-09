#ifndef UWB_LOKA_VISUAL_ODOMETRY_H
#define UWB_LOKA_VISUAL_ODOMETRY_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>

#include "state.h"
#include "observer.h"
#include "common.hpp"

namespace uwb_loka 
{

class V_Odom : public Observer 
{
public:
    V_Odom() = default;

    virtual ~V_Odom() {}

    void set_params( const Eigen::Isometry3d &Tvw, const Eigen::Isometry3d &Tcb );

    virtual Eigen::MatrixXd measurement_function( const Eigen::MatrixXd &mat_x ); 

    virtual Eigen::MatrixXd measurement_residual( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z );

    virtual Eigen::MatrixXd measurement_jacobian( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ); 

    virtual void check_jacobian( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ); 
  
private:
    Eigen::Isometry3d Tvw_;
    Eigen::Isometry3d Tcb_;
};

using V_Odom_ptr = std::shared_ptr<V_Odom>;

}  // namespace uwb_loka

#include "visual_odometry_impl.hpp"

#endif // UWB_LOKA_VISUAL_ODOMETRY_H
