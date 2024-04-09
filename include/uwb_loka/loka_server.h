#ifndef LOKA_SERVER_H
#define LOKA_SERVER_H

#include "uwb_node.h"
#include "ekf.h"
#include "state.h"

// ros
#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <fstream>

#include <Eigen/Core>

// test
#include <nav_msgs/Path.h>

namespace uwb_loka
{

class Loka_Server 
{
    
public:
    Loka_Server( const ros::NodeHandle &nh, const ros::NodeHandle &pnh );

    ~Loka_Server();

    void createMapList();

    inline bool checkAnchorInvolved( int &anchor_id ) const {
        return anchor_map_.count( anchor_id ) > 0;
    }

    inline bool checkTagInvolved( int &tag_id ) const {
        return tag_map_.count( tag_id ) > 0;
    }

    inline int getAnchorList_Size() const{
        return anchor_map_.size();
    }

    void anchorWarning( int anchor_id ) const;

    bool getAnchorPose( const int anchor_id, geometry_msgs::PoseStamped& anchor_position ) const;

    bool addAnchor( int anchor_id, std::shared_ptr<UWB_Anchor> anchor );

    bool removeAnchor( int anchor_id ); 

    bool updateAnchor( int anchor_id, std::shared_ptr<UWB_Anchor> anchor );

    bool addTag( int &tag_id, std::shared_ptr<UWB_Tag> tag );

    bool removeTag( int &tag_id );

    void imu_callback( const sensor_msgs::ImuConstPtr &imu_msg ); 

    void uwbStateCallback( const nlink_parser::LinktrackAoaNodeframe0 &uwb_msg );

    void gps_callback( const sensor_msgs::NavSatFixConstPtr &gps_msg );

    void vo_callback( const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg );

private:

    //test
    void predict_visualisierung();

    void vo_publish( const uwb_loka::State_Ptr &state_ptr, const Eigen::Isometry3d &TvoB );

    void gnss_publish( const uwb_loka::State_Ptr &state_ptr );

    void odo_publish( const uwb_loka::State_Ptr &state_ptr, nav_msgs::Odometry &odom_msg );
    
    void LogState( const uwb_loka::State_Ptr &state_ptr );
    
    void LogGps( const uwb_loka::GpsData_Ptr gps_data );

    std::vector<int> anchor_list_;
    std::map<int,std::shared_ptr<UWB_Anchor>> anchor_map_;
    std::vector<int> tag_list_;
    std::map<int,std::shared_ptr<UWB_Tag>> tag_map_;

    // parameters
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber uwb_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber vo_sub_;

    // IMU
    double imu_acc_n_, imu_gyro_n_, imu_acc_w_, imu_gyro_w_;
    double sigma_pv_, sigma_rp_, sigma_yaw_;

    string imu_topic_, gnss_topic_, vo_topic_, uwb_topic_;

    Eigen::Isometry3d Tcb_;
    Eigen::Isometry3d Tvw_;

    int n_ite_;

    // test
    tf2_ros::TransformBroadcaster pub_;

    EKF_Ptr ekf_ptr_;

    std::ofstream file_state_;
    std::ofstream file_gps_;

    // test
    ros::Publisher fused_path_pub_, fused_odom_pub_, vo_path_pub_;
    // ros::Publisher path_pub_;
    nav_msgs::Path fused_path_, vo_path_;
};

} // namespace uwb_loka

#include "loka_server_impl.hpp"

#endif // LOKA_SERVER_H