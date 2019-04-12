#ifndef LANEFOLLOWER
#define LANEFOLLOWER

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

namespace lane
{
  class Follower
  {
  public:
    Follower();
    ~Follower();

  private:
    void imgCallback(const sensor_msgs::ImgPtrConstPtr &msg); //Not sure what the message type is

    cv::Point2f calcMoment(const cv::Mat &img);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_p;

    ros::Subscriber img_sub;
    ros::Publisher cmd_pub;

    cv::Rect roi;
    double h_min, h_max;
    double s_min, s_max;
    double v_min, v_max;
  };
}
#endif
