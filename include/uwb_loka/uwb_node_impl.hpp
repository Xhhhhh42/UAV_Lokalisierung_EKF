#ifndef UWB_LOKA_UWB_NODE_IMPL_H
#define UWB_LOKA_UWB_NODE_IMPL_H

#include "uwb_node.h"

namespace uwb_loka
{
using namespace std;

std::unordered_map<Node_Type, std::string> UWB_Node::node_type_map_ = 
{
    {TAG, "TAG"},
    {ANCHOR, "ANCHOR"},
    {MONITOR, "MONITOR"},
    {NODE, "Uninit_Node"}
};

// ***************************************************************************** //
// ***************************** UWB_Anchor ************************************ //
// ***************************************************************************** //

UWB_Anchor::UWB_Anchor( const int node_id, const double x, const double y, const double z ) 
    : UWB_Node( node_id )
{
    std::string node = "ANCHOR";
    setNodeType( node );
    setNodePosition( x, y, z );
    setNodeOrientation( 1, 0, 0, 0 );
}


// ***************************************************************************** //
// ***************************** UWB_Tag *************************************** //
// ***************************************************************************** //

UWB_Tag::UWB_Tag( const int node_id )
    : UWB_Node( node_id ),
      tag_pose_map_()
{
    std::string node = "TAG";
    setNodeType( node );
    setNodePosition( 0, 0, 0 );
    setNodeOrientation( 1, 0, 0, 0 );    
}


bool UWB_Tag::tagWarning( int anchor_id ) const
{
    if( !checkPoseInvolved( anchor_id ) ) {
        ROS_WARN_STREAM_NAMED( "UWB_Tag", 
                               "Error: Pose with Anchor_ID " << anchor_id << " does not exists." );
        return false;
    } else return true;
}


bool UWB_Tag::addTagPose( int anchor_id, const geometry_msgs::PoseStamped& tag_pose ) 
{
    if( tagWarning( anchor_id ) ) {
        ROS_WARN_STREAM_NAMED( "UWB_Tag", 
                               "Error: Pose with Anchor_ID " << anchor_id << " already exists." );
        return false;
    }
    tag_pose_map_[anchor_id] = tag_pose;
    return true;
}


bool UWB_Tag::removeTagPose( int anchor_id ) 
{
    if( !tagWarning( anchor_id ) ) { return false; }
    tag_pose_map_.erase( anchor_id );
    return true;
}


bool UWB_Tag::getTagPose( int anchor_id, geometry_msgs::PoseStamped& tag_pose ) const 
{
    if( !tagWarning( anchor_id ) ) { return false; }
    tag_pose = tag_pose_map_.at( anchor_id );
    return true;
}


bool UWB_Tag::updateTagPose( int anchor_id, const geometry_msgs::PoseStamped& tag_pose ) 
{
    if( !tagWarning( anchor_id ) ) { return false; }
    tag_pose_map_[anchor_id] = tag_pose;
    return true;
}


void UWB_Tag::set_params( TagData_ConstPtr tag_data_ptr )
{
    timestamp_ = tag_data_ptr->timestamp;
    dis_ = tag_data_ptr->dis;
    ang_ = tag_data_ptr->angle;
}


Eigen::Vector3d UWB_Tag::pos( TagData_ConstPtr tag_data_ptr ) 
{
    Eigen::Vector3d pos;

    // PolarCoordinate polar;
    CartesianCoordinate cart; 
    float theta_rad = tag_data_ptr->angle * kDegreeToRadian; 
    cart.x = tag_data_ptr->dis * cos( theta_rad );
    cart.y = tag_data_ptr->dis * sin( theta_rad );
    pos[0] = cart.x;
    pos[1] = cart.y;
    pos[2] = 0;
    //     ts.transform.translation.y = cart.y;
    //     ts.transform.translation.z = 0;
    return pos;
}


Eigen::MatrixXd UWB_Tag::measurement_function( const Eigen::MatrixXd &mat_x ) 
{
    Eigen::Isometry3d Twb;
    Twb.matrix() = mat_x;
    Eigen::Vector3d pos;
    pos[0] = node_pose_.pose.position.x;
    pos[1] = node_pose_.pose.position.x;
    pos[2] = node_pose_.pose.position.x;
    return Twb * pos;
}


Eigen::MatrixXd UWB_Tag::measurement_residual( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ) 
{
    return mat_z - measurement_function( mat_x );
}


Eigen::MatrixXd UWB_Tag::measurement_jacobian( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ) 
{
    Eigen::Isometry3d Twb;
    Twb.matrix() = mat_x;

    Eigen::Matrix<double, kMeas_dim, kState_dim> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::Vector3d pos;
    pos[0] = node_pose_.pose.position.x;
    pos[1] = node_pose_.pose.position.x;
    pos[2] = node_pose_.pose.position.x;
    H.block<3, 3>(0, 6) = -Twb.linear() * skew_matrix( pos );

    return H;
}

} // namespace uwb_loka

#endif // UWB_LOKA_UWB_NODE_IMPL_H