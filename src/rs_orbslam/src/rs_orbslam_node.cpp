#include <ros/ros.h>

#include <stdio.h>
#include <iostream>

#include "rs_orbslam/realsense_orb_slam.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rs_orbslam");

  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  Realsense_ORB_SLAM rs_orb_slam(nh, pnh);

  ros::spin();
  return 0;
}
