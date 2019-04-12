#ifndef LANEFOLLOWER_HPP
#define LANEFOLLOWER_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <autopilot/Controller_Commands.h>

namespace lane
{

class Follower
{
public:
  Follower();
  ~Follower();

protected:
  void imgCallback(const sensor_msgs::ImagePtr &msg);
  cv::Point2f calcMoment(const cv::Mat &img);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;

  ros::Subscriber img_sub_;
  ros::Publisher cmd_pub_;

  autopilot::Controller_Commands cmd_msg_;
  double vel_cmd_;

  cv::Rect roi_;
  double h_min_, h_max_;
  double s_min_, s_max_;
  double v_min_, v_max_;
};

} // end namespace lane

#endif // LANEFOLLOWER_HPP
