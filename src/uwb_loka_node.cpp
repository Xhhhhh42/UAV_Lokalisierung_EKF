#include <ros/ros.h>

#include "uwb_loka/loka_server.h"

using namespace uwb_loka;

int main(int argc, char **argv)
{
  // Initialize ros
  ros::init( argc, argv, "uwb_loka_node" );
  ros::NodeHandle nh;
  ros::NodeHandle pnh( "~" );

  Loka_Server loka_server( nh, pnh );

  // Situation 1
  ros::spin();
  return 1;

  // // Situation 2
  // ros::Rate loop_rate(10);  // 10Hz
  // while (ros::ok()) {
  //       loka_server.predict_visualisierung();
        
  //       ros::spinOnce();  // 处理一次回调
  //       loop_rate.sleep();  // 等待直到达到10Hz的频率
  //   }

  // return 0;
}