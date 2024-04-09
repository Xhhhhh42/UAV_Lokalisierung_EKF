#ifndef LOKA_SERVER_IMPL_H
#define LOKA_SERVER_IMPL_H

#include "loka_server.h"
#include "common.hpp"
#include "gnss.h"
#include "visual_odometry .h"

#include <geometry_msgs/TransformStamped.h>

// #include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace uwb_loka
{

Loka_Server::Loka_Server( const ros::NodeHandle &nh, const ros::NodeHandle &pnh )
    : nh_( nh ), pnh_( pnh )
{
    // subscribe uav state
    uwb_sub_ = nh_.subscribe( "/nlink_linktrack_aoa_nodeframe0", 10, &Loka_Server::uwbStateCallback, this );

    // create UWB_Tag/UWB_Anchor Map
    createMapList();

    nh_.param<double>( "acc_noise", imu_acc_n_, 1e-2 );
    nh_.param<double>( "gyro_noise", imu_gyro_n_, 1e-4 );
    nh_.param<double>( "acc_bias_noise", imu_acc_w_, 1e-6 );
    nh_.param<double>( "gyro_bias_noise", imu_gyro_w_, 1e-8 );

    nh_.param<string>( "imu_topic", imu_topic_, "/imu/data" );
    nh_.param<string>( "gnss_topic", gnss_topic_, "/fix" );
    nh_.param<string>( "vo_topic", vo_topic_, "/odom_vo" );
    nh_.param<string>( "uwb_topic", uwb_topic_, "/uwb" );

    nh_.param( "init_sigma_pv", sigma_pv_, 0.01 );
    nh_.param( "init_sigma_rp", sigma_rp_, 0.01 );
    nh_.param( "init_sigma_yaw", sigma_yaw_, 5.0 );

    nh_.param( "EKF_number_iterations", n_ite_, 1 );
    
    ekf_ptr_ = std::make_unique<EKF>( imu_acc_n_, imu_gyro_n_, imu_acc_w_, imu_gyro_w_ );
    
    // Set covariance :
    //      position std: 10 m
    //      velocity std: 10 m/s
    //      roll pitch std 10 degree
    //      yaw std: 100 degree
    //      Acc bias / Gyro bias.
    // sigma_pv = 10;
    sigma_rp_ *= kDegreeToRadian;
    sigma_yaw_ *= kDegreeToRadian;
    ekf_ptr_->state_ptr_->set_cov_( sigma_pv_, sigma_pv_, sigma_rp_, sigma_yaw_, imu_acc_w_, imu_gyro_w_ );

    // ekf_ptr_->observer_ptr_ = std::make_shared<GNSS>();
    ekf_ptr_->observer_ptr_ = std::make_shared<V_Odom>();

    // std::string topic_imu = "/imu/data";
    // std::string topic_imu = "/bb_imu";
    // std::string topic_gps = "/fix";

    // std::string topic_vo = "/odom_vo";
    // std::string topic_imu = "/imu0";

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>( imu_topic_, 10, &Loka_Server::imu_callback, this );
    gps_sub_ = nh_.subscribe( gnss_topic_, 10, &Loka_Server::gps_callback, this );
    vo_sub_ = nh_.subscribe( vo_topic_, 10, &Loka_Server::vo_callback, this );

    Tcb_ = getTransformEigen( pnh_, "cam0/T_cam_imu" );

    // Log
    std::string log_folder = "/home";
    ros::param::get( "log_folder", log_folder );
    file_state_.open( log_folder + "/state.csv" );
    file_gps_.open( log_folder +"/gps.csv" );

    //test
    // pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "pose_topic", 10 );
    fused_path_pub_ = nh_.advertise<nav_msgs::Path>( "fused_trajectory", 10, true );
    fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>( "odom_est", 10 );
    vo_path_pub_ = nh_.advertise<nav_msgs::Path>( "path_vo", 10 );
}


Loka_Server::~Loka_Server()
{
    file_state_.close();
    file_gps_.close();
}


void Loka_Server::createMapList()
{
    // get anchor list from parameter server
    std::string param_key( "/uwb_loka_node/anchor_list" );
    if( !nh_.getParam( param_key, anchor_list_ )) {
        ROS_ERROR( "Can't find anchor list param." );
        return;
    }

    // get anchor position, build anchor_map
    for( unsigned int i = 0; i < anchor_list_.size(); ++i ) 
    {
        int anchor_id = anchor_list_.at(i);
        param_key = std::string( "anchor_" ) + std::to_string( anchor_id );
        std::vector<double> position;
        if( nh_.getParam( param_key, position ) && position.size() == 3 ) {
            // has valid position
            std::shared_ptr<UWB_Anchor> anchor( new UWB_Anchor( anchor_id, position.at(0), 
                                                                position.at(1), position.at(2) ));
            anchor_map_.insert( std::pair<int, std::shared_ptr<UWB_Anchor>>( anchor_id, anchor ));
        }
    }


    // get tag list from parameter server
    param_key = "/uwb_loka_node/tag_id";
    if( !nh_.getParam( param_key, tag_list_ )) {
        ROS_ERROR( "Can't find tag list param." );
        return;
    }

    for( unsigned int j = 0; j < tag_list_.size(); ++j ) 
    {
        int tag_id = tag_list_.at( j );
        param_key = std::string( "tag_" ) + std::to_string( tag_id );
        std::shared_ptr<UWB_Tag> tag( new UWB_Tag( tag_id ));
        tag_map_.insert( std::pair<int, std::shared_ptr<UWB_Tag>>( tag_id, tag ));
    }

    // ROS_INFO( "UWB_Map created." );
}


bool Loka_Server::addTag( int &tag_id, std::shared_ptr<UWB_Tag> tag )
{
    if( checkTagInvolved( tag_id ) ) {
        ROS_WARN_STREAM_NAMED( "UWB_Anchor", 
                               "Error: Tag with ID " << tag_id << " already exists." );
        return false;
    } else {
        tag_map_[tag_id] = tag;
        return true;
    }
}

bool Loka_Server::removeTag( int &tag_id )
{
    if( !checkTagInvolved( tag_id ) ) {
        ROS_WARN_STREAM_NAMED( "UWB_Anchor", 
                               "Error: Tag with ID " << tag_id << " does not exists." );
        return false;
    } else {
        tag_map_.erase( tag_id );
        return true;
    }
}


void Loka_Server::imu_callback( const sensor_msgs::ImuConstPtr &imu_msg ) 
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

    // 如果EKF仍未初始化，加入IMU数据到缓冲区后返回
    if( !ekf_ptr_->push_data( imu_data_ptr, ekf_ptr_->inited_ )) return;

    ImuData_ConstPtr last_imu_ptr = ekf_ptr_->last_imu_ptr_;
    ekf_ptr_->predict( last_imu_ptr, imu_data_ptr );
    ekf_ptr_->last_imu_ptr_ = imu_data_ptr;

    // Log fused state.
    LogState( ekf_ptr_->state_ptr_ );

    // predict_visualisierung();
}


void Loka_Server::uwbStateCallback( const nlink_parser::LinktrackAoaNodeframe0 &uwb_msg )
{
    // for( const auto &msg : uwb_msg.nodes ) {
    //     int tag_id = msg.id;
    //     PolarCoordinate polar;
    //     CartesianCoordinate cart;
    //     polar.r = msg.dis;
    //     polar.theta = msg.angle;
    //     polarToCartesian( polar, cart );
    //     // ROS_INFO_STREAM_NAMED( "UWB_Tag", 
    //     //                        "X: " << cart.x << " ; Y: " << cart.y );
        
    //     geometry_msgs::TransformStamped ts;
    //     ts.header.frame_id = "world";
    //     ts.child_frame_id = "uwb";
    //     ts.header.stamp = ros::Time::now();
    //     ts.transform.translation.x = cart.x;
    //     ts.transform.translation.y = cart.y;
    //     ts.transform.translation.z = 0;
    //     tf2::Quaternion q;
    //     // q.setRPY( 0, 0, polar.theta );
    //     q.setRPY( 0, 0, 0 );
    //     // ts.transform.rotation.w = q.getW();
    //     // ts.transform.rotation.x = q.getX();
    //     // ts.transform.rotation.y = q.getY();
    //     // ts.transform.rotation.z = q.getZ();
    //     ts.transform.rotation = tf2::toMsg(q);
    //     pub_.sendTransform( ts );
    // }

    // TagData_Ptr tag_data_ptr = std::make_shared<Tag_Data>();
    // tag_data_ptr->timestamp = uwb_msg.stamp.toSec();
    // tag_data_ptr->dis = uwb_msg.nodes[0].dis;
    // tag_data_ptr->angle = uwb_msg.nodes[0].angle;
    // tag_data_ptr->fp_rssi = uwb_msg.nodes[0].fp_rssi;
    // tag_data_ptr->rx_rssi = uwb_msg.nodes[0].rx_rssi;
}


void Loka_Server::gps_callback( const sensor_msgs::NavSatFixConstPtr &gps_msg_ptr ) 
{
    // status 字段通常表示 GPS 信号的质量或定位的准确.
    // 根据 ROS 中 sensor_msgs/NavSatStatus 消息的定义，状态值 2 通常表示定位数据的质量是好的（例如，表示定位是基于固定解）
    if( gps_msg_ptr->status.status != 2 ) {
        ROS_WARN_STREAM_NAMED( "GNSS", 
                               "GPS-Error: Bad GPS Message!!!" );
        return;
    }

    GpsData_Ptr gps_data_ptr = std::make_shared<GpsData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->latitude,
                        gps_msg_ptr->longitude,
                        gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>( gps_msg_ptr->position_covariance.data() );

    if ( !ekf_ptr_->inited_ ) {
        if ( !ekf_ptr_->init( gps_data_ptr->timestamp )) return;

        std::dynamic_pointer_cast<GNSS>( ekf_ptr_->observer_ptr_ )->set_params( gps_data_ptr );
        ROS_INFO( "System initialized." );
        return;
    }

    const Eigen::Isometry3d &Twb = ekf_ptr_->state_ptr_->get_pose();

    // G_p_Gps : GPS Position in global coordinate ( WGS84 to ENU frame )
    const auto &G_p_Gps = std::dynamic_pointer_cast<GNSS>( ekf_ptr_->observer_ptr_ )->g2l( gps_data_ptr );

    const auto &residual = ekf_ptr_->observer_ptr_->measurement_residual( Twb.matrix(), G_p_Gps );
    const auto &H = ekf_ptr_->observer_ptr_->measurement_jacobian( Twb.matrix(), G_p_Gps );

    Eigen::Matrix<double, kState_dim, 3> K;
    const Eigen::Matrix3d &R = gps_data_ptr->cov;
    ekf_ptr_->update_K( H, R, K );
    ekf_ptr_->update_P( H, R, K );

    *ekf_ptr_->state_ptr_ = *ekf_ptr_->state_ptr_ + K * residual;

    LogGps( gps_data_ptr );
}


void Loka_Server::vo_callback( const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg )
{
    Eigen::Isometry3d Tvo;  // VO in frame V --> Tc0cn
    tf::poseMsgToEigen( vo_msg->pose.pose, Tvo );

    const Eigen::Matrix<double, kMeasDim, kMeasDim> &R =
        Eigen::Map<const Eigen::Matrix<double, kMeasDim, kMeasDim>>( vo_msg->pose.covariance.data() );

    if( !ekf_ptr_->inited_ ) 
    {
        if( !ekf_ptr_->init( vo_msg->header.stamp.toSec() )) return;

        Eigen::Isometry3d Tb0bm;
        Tb0bm.linear() = ekf_ptr_->state_ptr_->R_;
        Tb0bm.translation().setZero();

        const Eigen::Isometry3d &Tc0cm = Tvo;

        // 将VO的位姿变换到世界坐标系( c0 --> visual frame V, b0 --> world frame W )
        Tvw_ = Tc0cm * Tcb_ * Tb0bm.inverse();  

        std::dynamic_pointer_cast<V_Odom>( ekf_ptr_->observer_ptr_ )->set_params( Tvw_, Tcb_ );

        ROS_INFO( "System initialized." );

        return;
    }

    // IEKF iteration update, same with EKF when n_ite = 1
    // n_ite_ = 10;
    Eigen::Matrix<double, kMeasDim, kState_dim> H_i;
    Eigen::Matrix<double, kState_dim, kMeasDim> K_i;
    for( int i = 0; i < n_ite_; i++ )
    {
        if( i == 0 ) { *ekf_ptr_->state_ptr_i_ = *ekf_ptr_->state_ptr_; }

        const Eigen::Isometry3d &Twb_i = ekf_ptr_->state_ptr_i_->get_pose();  // x_i

        // J
        H_i = std::dynamic_pointer_cast<V_Odom>( ekf_ptr_->observer_ptr_ )->measurement_jacobian( Twb_i.matrix(), Tvo.matrix() );
        ekf_ptr_->observer_ptr_->check_jacobian( Twb_i.matrix(), Tvo.matrix() );  // for debug

        // r
        auto residual = ekf_ptr_->observer_ptr_->measurement_residual( Twb_i.matrix(), Tvo.matrix() );
        // for IEKF, residual -= H (x_prior - x_i)
        residual -= H_i * ( *ekf_ptr_->state_ptr_ - *ekf_ptr_->state_ptr_i_ );
        // std::cout << "res: " << residual.transpose() << std::endl;

        // K
        ekf_ptr_->update_K( H_i, R, K_i );

        // update state
        *ekf_ptr_->state_ptr_i_ = *ekf_ptr_->state_ptr_ + K_i * residual;
    }

    // update state and cov
    *ekf_ptr_->state_ptr_ = *ekf_ptr_->state_ptr_i_;
    ekf_ptr_->update_P( H_i, R, K_i );

    // std::cout << "acc bias: " << ekf_ptr_->state_ptr_->acc_bias_.transpose() << std::endl;
    // std::cout << "gyr bias: " << ekf_ptr_->state_ptr_->gyr_bias_.transpose() << std::endl;

    // view
    // for publish, Tvo in frame W --> Tb0bn
    Eigen::Isometry3d TvoB = Tvw_.inverse() * Tvo * Tcb_;
    // viewer_.publish_vo(*ekf_ptr_->state_ptr_, TvoB);

    vo_publish( ekf_ptr_->state_ptr_, TvoB );
}


void Loka_Server::anchorWarning( int anchor_id ) const
{
    if( !checkAnchorInvolved( anchor_id ) ) {
        ROS_WARN_STREAM_NAMED( "UWB_Tag", 
                               "Error: Anchor with ID " << anchor_id << " does not exists." );
    }
}


bool Loka_Server::getAnchorPose( const int anchor_id, geometry_msgs::PoseStamped& anchor_pose ) const
{
    if( anchor_map_.find( anchor_id ) != anchor_map_.end() ) {
        anchor_pose = anchor_map_.find( anchor_id )->second->getNodePose();
        return true;
    } else {
        anchorWarning( anchor_id );
        return false;
    }
}


bool Loka_Server::addAnchor( int anchor_id, std::shared_ptr<UWB_Anchor> anchor )
{
    if( checkAnchorInvolved( anchor_id ) ) {
        ROS_WARN_STREAM_NAMED( "UWB_Tag", 
                               "Error: Anchor with ID " << anchor_id << " already exists." );
        return false;
    }
    anchor_map_[anchor_id] = anchor;
    return true;
}


bool Loka_Server::removeAnchor( int anchor_id )
{
    if( !checkAnchorInvolved( anchor_id ) ) {
        anchorWarning( anchor_id );
        return false;
    }
    anchor_map_.erase(anchor_id);
    return true;
}


bool Loka_Server::updateAnchor( int anchor_id, std::shared_ptr<UWB_Anchor> anchor )
{
    if( !checkAnchorInvolved( anchor_id ) ) {
        anchorWarning( anchor_id );
        addAnchor( anchor_id, anchor );
        return false;
    }
    anchor_map_[anchor_id] = anchor;
    return true;
}    


void Loka_Server::predict_visualisierung() 
{
    State curr_state = *ekf_ptr_->state_ptr_;

    fused_path_.header.frame_id = "base_link";
    ros::Time current_time = ros::Time::now();
    fused_path_.header.stamp = current_time;

    geometry_msgs::PoseStamped gt;
    gt.header = fused_path_.header;
    Eigen::Matrix<double, 7, 1> pose = curr_state.vec_pq();

    gt.pose.position.x = pose(0);
    gt.pose.position.y = pose(1);
    gt.pose.position.z = pose(2);

    gt.pose.orientation.x = pose(3);
    gt.pose.orientation.y = pose(4);
    gt.pose.orientation.z = pose(5);
    gt.pose.orientation.w = pose(6);

    // pose_pub_.publish( gt );

    fused_path_.poses.push_back( gt );
    fused_path_pub_.publish( fused_path_ );
}


void Loka_Server::vo_publish( const uwb_loka::State_Ptr &state_ptr, const Eigen::Isometry3d &TvoB )
{
    nav_msgs::Odometry odom_msg;
    odo_publish( state_ptr, odom_msg );

    // publish the fused path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    fused_path_.header = pose_stamped.header;
    fused_path_.poses.push_back( pose_stamped );
    fused_path_pub_.publish( fused_path_ );

    // publish vo path
    geometry_msgs::Pose vo_pose;
    tf::poseEigenToMsg( TvoB, vo_pose );
    geometry_msgs::PoseStamped vo_pose_stamped;
    vo_pose_stamped.header = pose_stamped.header;
    vo_pose_stamped.pose = vo_pose;
    vo_path_.header = vo_pose_stamped.header;
    vo_path_.poses.push_back( vo_pose_stamped );
    vo_path_pub_.publish( vo_path_ );
}


void Loka_Server::gnss_publish( const uwb_loka::State_Ptr &state_ptr )
{
    nav_msgs::Odometry odom_msg;
    odo_publish( state_ptr, odom_msg );

    // // publish the odometry
    // std::string fixed_id = "global";
    // nav_msgs::Odometry odom_msg;
    // odom_msg.header.frame_id = fixed_id;
    // odom_msg.header.stamp = ros::Time::now();
    // Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    // T_wb.linear() = state.Rwb_;
    // T_wb.translation() = state.p_wb_;
    // tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
    // tf::vectorEigenToMsg(state.v_wb_, odom_msg.twist.twist.linear);
    // Eigen::Matrix3d P_pp = state_ptr->cov_.block<3, 3>(0, 0);
    // Eigen::Matrix3d P_po = state_ptr->cov_.block<3, 3>(0, 6);
    // Eigen::Matrix3d P_op = state_ptr->cov_.block<3, 3>(6, 0);
    // Eigen::Matrix3d P_oo = state_ptr->cov_.block<3, 3>(6, 6);
    // Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
    // P_imu_pose << P_pp, P_po, P_op, P_oo;
    // for (int i = 0; i < 36; i++) odom_msg.pose.covariance[i] = P_imu_pose.data()[i];
    // odom_pub_.publish(odom_msg);

    // publish the path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    fused_path_.header = pose_stamped.header;
    fused_path_.poses.push_back( pose_stamped );
    fused_path_pub_.publish( fused_path_ );
}


void Loka_Server::odo_publish( const uwb_loka::State_Ptr &state_ptr, nav_msgs::Odometry &odom_msg )
{
    // publish the odometry
    std::string fixed_id = "global";
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = fixed_id;
    odom_msg.child_frame_id = "odom";

    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = state_ptr->R_;
    T_wb.translation() = state_ptr->p_;
    tf::poseEigenToMsg( T_wb, odom_msg.pose.pose );
    tf::vectorEigenToMsg( state_ptr->v_, odom_msg.twist.twist.linear );

    const Eigen::Matrix3d &P_pp = state_ptr->cov_.block<3, 3>(0, 0);
    const Eigen::Matrix3d &P_po = state_ptr->cov_.block<3, 3>(0, 6);
    const Eigen::Matrix3d &P_op = state_ptr->cov_.block<3, 3>(6, 0);
    const Eigen::Matrix3d &P_oo = state_ptr->cov_.block<3, 3>(6, 6);

    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
    P_imu_pose << P_pp, P_po, P_op, P_oo;

    for( int i = 0; i < 36; i++ ) { odom_msg.pose.covariance[i] = P_imu_pose.data()[i]; }
    fused_odom_pub_.publish( odom_msg );
}


void Loka_Server::LogState( const uwb_loka::State_Ptr &state_ptr ) 
{
    const Eigen::Quaterniond G_q_I( state_ptr->get_Rotate() );
    const Eigen::Vector3d position = state_ptr->get_position();
    const auto &vec = state_ptr->vec_vb();
    file_state_ << std::fixed << std::setprecision(15)
                << state_ptr->get_timestamp() << ","
                // << state_ptr.lla[0] << "," << state_ptr.lla[1] << "," << state_ptr.lla[2] << ","
                << position(0) << "," << position(1) << "," << position(2) << ","
                << vec(0) << "," << vec(1)  << "," << vec(2)  << "," << "\n";
                // << G_q_I.x() << "," << G_q_I.y() << "," << G_q_I.z() << "," << G_q_I.w() << ","
                // << vec(3) << "," << vec(4)  << "," << vec(5)  << ","
                // << vec(6) << "," << vec(7)  << "," << vec(8)  << "\n";
}


void Loka_Server::LogGps( const uwb_loka::GpsData_Ptr gps_data ) 
{
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
}


} // namespace uwb_loka

#endif// LOKA_SERVER_IMPL_H