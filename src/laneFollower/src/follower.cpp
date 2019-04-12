#include "laneFollower/follower.h"
#include "autopilot/Controll_Commands.h"
#include <cmath>
#include <sensor_msgs/ImagePtr.h>

namespace lane
{
  Follower::Follower(): nh_(ros::NodeHandle()), nh_p("~")
  {
    roi.x = 0;
    roi.y = 600;
    roi.width = 1280;
    roi.height = 4;
  }

  Follower::~Follower(){}

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

  void Follower::imgCallback(const sensor_msgs::ImgPtrConstPtr &msg) //not sure this is right
  {
    cv::Mat img; //convert msg to openCV form
  }
}
