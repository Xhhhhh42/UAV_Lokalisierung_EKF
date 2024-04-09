#ifndef UWB_LOKA_UWB_NODE_H
#define UWB_LOKA_UWB_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <memory> // 引入 std::shared_ptr
#include <unordered_map>

#include <map>

#include <nlink_parser/LinktrackAoaNodeframe0.h>

#include "observer.h"
#include "common.hpp"

namespace uwb_loka
{
enum Node_Type 
{
    NODE,
    TAG,
    ANCHOR,
    MONITOR,
};

class UWB_Node
{ 
public:
    explicit UWB_Node( const int node_id ) : node_id_( node_id ) {
        node_type_ = NODE;
    }

    int getNodeId() const {
        return node_id_;
    }

    std::string getNodeType() const {
        auto it = node_type_map_.find( node_type_ );
        if ( it != node_type_map_.end() ) { return it->second; } 
        else { return "UNKNOWN"; }
    }

    geometry_msgs::PoseStamped getNodePose() const {
        return node_pose_;
    }

    geometry_msgs::PoseStamped* getNodePosePtr() {
        return &node_pose_;
    }

    void setNodePosition( const double x, const double y, const double z ) 
    {
        node_pose_.header.stamp = ros::Time::now();
        node_pose_.pose.position.x = x;
        node_pose_.pose.position.y = y;
        node_pose_.pose.position.z = z;
    }

    void setNodeOrientation( const double qw, const double qx, const double qy, const double qz )
    {
        node_pose_.header.stamp = ros::Time::now();
        node_pose_.pose.orientation.w = qw;
        node_pose_.pose.orientation.x = qx;
        node_pose_.pose.orientation.y = qy;
        node_pose_.pose.orientation.z = qz;
    }

    void setNodeType( std::string node ) 
    {
        for (const auto& pair : node_type_map_) {
            if ( pair.second == node ) {
                node_type_ = pair.first;
                return;
            }
        }
        // If no match is found, set to a default value or handle the case accordingly.
        node_type_ = NODE;
    }

protected:
    int node_id_;
    Node_Type node_type_;
    geometry_msgs::PoseStamped node_pose_;
    static std::unordered_map<Node_Type, std::string> node_type_map_;
};


class UWB_Anchor : public UWB_Node 
{
public:
    explicit UWB_Anchor( const int node_id, const double x, const double y, const double z );
};


struct Tag_Data {
    int id;
    double timestamp;
    double dis; 
    double angle;
    double fp_rssi;
    double rx_rssi;
};

using TagData_Ptr = std::shared_ptr<Tag_Data>;
using TagData_ConstPtr = std::shared_ptr<const Tag_Data>;


class UWB_Tag : public UWB_Node, public Observer
{
public:
    explicit UWB_Tag( const int node_id );

    inline bool checkPoseInvolved( int &anchor_id ) const {
        return tag_pose_map_.count( anchor_id ) > 0;
    }

    bool tagWarning( int anchor_id ) const;

    bool addTagPose( int anchor_id, const geometry_msgs::PoseStamped& tag_pose );

    bool removeTagPose( int anchor_id );

    bool getTagPose( int anchor_id, geometry_msgs::PoseStamped& tag_pose ) const;

    bool updateTagPose( int anchor_id, const geometry_msgs::PoseStamped& tag_pose );

    void set_params( TagData_ConstPtr tag_data_ptr );

    Eigen::Vector3d pos( TagData_ConstPtr tag_data_ptr );

    virtual Eigen::MatrixXd measurement_function( const Eigen::MatrixXd &mat_x );

    virtual Eigen::MatrixXd measurement_residual( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z );

    virtual Eigen::MatrixXd measurement_jacobian( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ); 

    virtual void check_jacobian(const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z) {}

private:
    std::map<int,geometry_msgs::PoseStamped> tag_pose_map_;
    double timestamp_;
    double dis_;
    double ang_;
};

using Tag_Ptr = std::shared_ptr<UWB_Tag>;

} //namespace uwb_loka

#include "uwb_node_impl.hpp"

#endif // UWB_LOKA_UWB_NODE_H