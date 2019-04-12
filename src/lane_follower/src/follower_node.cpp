#include <ros/ros.h>
#include "lane_follower/follower.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_follower");
  ros::NodeHandle nh;

  lane::Follower follower_obj;

  ros::spin();

  return 0;
}
