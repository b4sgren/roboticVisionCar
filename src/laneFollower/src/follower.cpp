#include "laneFollower/follower.hpp"
#include <cmath>
#include <cv_bridge/cv_bridge.h>

namespace lane
{

Follower::Follower(): nh_(ros::NodeHandle()), nh_p("~")
{
  roi.x = 0;
  roi.y = 600;
  roi.width = 1280;
  roi.height = 4;

  uint32_t queue_size{5};
  img_sub_ = nh_.subscribe("camera/color/color_rect_img",queue_size,
                           &Follower::imgCallback, this);
  cmd_pub_ = nh_.advertise<autopilot::Controller_Commands>("command",queue_size);
}

Follower::~Follower(){}

void Follower::imgCallback(const sensor_msgs::ImageConstPtr &msg) 
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("[laneFollower] cv_bridge exception: %s", e.what());
      return;
  }

  cv::Mat img; 
  cv_ptr->image.copyTo(img);
}

cv::Point2f Follower::calcMoment(const cv::Mat &img)
{
  cv::Moments m{cv::moments(img, true)};
  cv::Point2f center;
  if (m.m00 != 0.0)
    center = cv::Point2f{float(m.m10/m.m00), float(m.m01/m.m00)};
  else
    center = cv::Point2f{0,0};

 return center;
}

} // end namespace lane
