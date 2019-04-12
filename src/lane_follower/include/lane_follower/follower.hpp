#ifndef LANEFOLLOWER_HPP
#define LANEFOLLOWER_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include "autopilot/Controller_Commands.h"

namespace lane
{

class Follower
{
public:
  Follower();
  ~Follower();

protected:
  void imgCallback(const sensor_msgs::ImageConstPtr &msg);
  cv::Point2f calcMoment(const cv::Mat &img);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;

  ros::Subscriber img_sub_;
  ros::Publisher cmd_pub_;

  cv::Rect roi;
  double h_min, h_max;
  double s_min, s_max;
  double v_min, v_max;
};

} // end namespace lane

#endif // LANEFOLLOWER_HPP
