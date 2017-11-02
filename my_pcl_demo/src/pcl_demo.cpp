#include"pcl_pointcloud.h"


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  pcl_pointcloud    pcl_test;

  // Spin
  ros::AsyncSpinner spinner(0); // Use 2 threads
  spinner.start();
  ros::waitForShutdown();
}
