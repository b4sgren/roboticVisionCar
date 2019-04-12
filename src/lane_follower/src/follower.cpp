#include "lane_follower/follower.hpp"
#include <cmath>
#include <cv_bridge/cv_bridge.h>

//#define TESTING
#define PRINT

#ifdef TESTING
int mouse_X, mouse_Y;

void on_mouse(int evt, int x, int y, int flags, void* param) 
{
  if(evt == cv::EVENT_LBUTTONDOWN)
  {
    mouse_X = x;
    mouse_Y = y;
  }
}
#endif

namespace lane
{

Follower::Follower(): 
  nh_(ros::NodeHandle()),
  nh_p_("~"),
  vel_cmd_(0.5)
{
  roi_l_.x = 0;
  roi_l_.y = 320;
  roi_l_.width = 320;
  roi_l_.height = 4;

  roi_r_.x = 320;
  roi_r_.y = 320;
  roi_r_.width = 320;
  roi_r_.height = 4;

  uint32_t queue_size{5};
  img_sub_ = nh_.subscribe("camera/color/image_raw",queue_size,
                           &Follower::imgCallback, this);
  cmd_pub_ = nh_.advertise<autopilot::Controller_Commands>("command",queue_size);
  test_pub_ = nh_.advertise<sensor_msgs::Image>("test_img",queue_size);
  color_test_pub_ = nh_.advertise<sensor_msgs::Image>("color_test_img",queue_size);
  crop_test_pub_ = nh_.advertise<sensor_msgs::Image>("crop_test_img",queue_size);
}

Follower::~Follower(){}

void Follower::imgCallback(const sensor_msgs::ImagePtr &msg) 
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

  cv::Mat img, hsv, bw_img, cropped_img_l, cropped_img_r, print_img; 
  cv_ptr->image.copyTo(img);
  img.copyTo(print_img);
  img.convertTo(img, CV_8U);
  if (img.empty())
  {
      ROS_WARN("[laneFollower] received empty img.");
      return;
  }

  double x0{img.cols/2.0}, y0{double(img.rows)};

  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
  //Captures the table legs
  cv::inRange(hsv, cv::Scalar(93, 0, 0), cv::Scalar(96, 255, 255), bw_img); 
  cv::Mat element{cv::getStructuringElement(cv::MORPH_RECT, cv::Size{7,7})};
  cv::erode(bw_img, bw_img, element);

  cropped_img_l = bw_img(roi_l_);
  cropped_img_r = bw_img(roi_r_);

  cv::Point2f center_l = calcMoment(cropped_img_l);
  cv::Point2f center_r = calcMoment(cropped_img_r);
  center_l.x += roi_l_.x;
  center_l.y += roi_l_.y;
  center_r.x += roi_r_.x;
  center_r.y += roi_r_.y;
  double x_avg, y_avg;
  x_avg = (center_l.x + center_r.x) / 2.0;
  y_avg = (center_l.y + center_r.y) / 2.0;
  ROS_INFO("Center x: %f, Center y: %f", center.x, center.y);
  double psi = atan2(x_avg - x0, y0 - y_avg);
  ROS_INFO("Angle: %f", psi * 180/3.14159265);

#ifdef PRINT
  cv::circle(print_img, cv::Point{x_avg,y_avg}, 3, cv::Scalar{0,0,255});

  cv_bridge::CvImage out_img, crop_out_img, color_out_img;
  out_img.encoding = sensor_msgs::image_encodings::MONO8;
  out_img.image = bw_img;
  crop_out_img.encoding = sensor_msgs::image_encodings::MONO8;
  crop_out_img.image = cropped_img_l;
  crop_out_img.encoding = sensor_msgs::image_encodings::BGR8;
  color_out_img.image = print_img;

  test_pub_.publish(out_img.toImageMsg());
  crop_test_pub_.publish(crop_out_img.toImageMsg());
#endif

#ifdef TESTING
  cv::imshow("Window", bw_img);
  cv::imshow("Color", img);
  cv::imshow("Cropped", cropped_img);
  cv::setMouseCallback("Color", on_mouse);
  cv::waitKey(0);
#else
  cmd_msg_.u_c = vel_cmd_;
  cmd_msg_.psi_c = psi;

  cmd_pub_.publish(cmd_msg_);
#endif
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
