#include <ros/ros.h>
#include <numeric> 
#include <iostream>
#include <plan_manage/kino_manage.h>

int main(int argc, char ** argv){ 
  
  ros::init(argc, argv, "resilient_planner_node");
  ros::NodeHandle nh("~");

  KINOManage kino_manager;
  kino_manager.init(nh);

  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  
  spinner.spin(); // spin() will not return until the node has been shutdown

  return 0;
}
