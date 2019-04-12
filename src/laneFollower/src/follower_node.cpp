#include <ros/ros.h>
#include "laneFollower/follower.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laneFollower");
  ros::NodeHandle nh;

  //initialize laneFollower class
  ros::spin();
  return 0;
}
