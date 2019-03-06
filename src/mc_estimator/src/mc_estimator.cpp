#include "mc_estimator.h"
#include <math.h>
#include <vector>

namespace mc_estimate
{

MCEstimator::MCEstimator():
    nh_(),
    nh_private_("~")
{
    prev_time_ = ros::Time::now();
    pose_sub_ = nh_.subscribe("/vrpn_client_node/VisionCar/pose", 10, &MCEstimator::pose_callback, this);
    rc_pub_ = nh_.advertise<autopilot::State>("/state",1);;
}

MCEstimator::~MCEstimator()
{
}

void MCEstimator::roll_pitch_yaw_from_quat(std::vector<float> q)
{
    double ex = q[0];
    double ey = q[1];
    double ez = q[2];
    double eo = q[3];
    // ROS_INFO("%f, %f, %f, %f ", ex, ey, ex, eo);
    roll_   = atan2(2.0*eo*ex + 2.0*ey*ez, eo*eo + ez*ez - ex*ex - ey*ey);
    pitch_ = asin(2.0*eo*ey - 2.0*ex*ez);
    yaw_   = atan2(2.0*eo*ez + 2.0*ex*ey, eo*eo + ex*ex - ey*ey - ez*ez);

}

void MCEstimator::pose_callback(const geometry_msgs::PoseStampedPtr &msg)
{

    ros::Time now = ros::Time::now();
    dt_ = (now-prev_time_).toSec();
    prev_time_ = now;

    x_ = msg->pose.position.x;
    y_ = msg->pose.position.z;
    z_ = -msg->pose.position.y;

    std::vector<float> q;
    q.push_back(msg->pose.orientation.x); //ROLL
    q.push_back(msg->pose.orientation.z); //PITCH
    q.push_back(-msg->pose.orientation.y); //YAW
    q.push_back(msg->pose.orientation.w);
    roll_pitch_yaw_from_quat(q);

    // publish();
    out_msg.p_north = x_;
    out_msg.p_east = y_;
    out_msg.psi = yaw_;
    out_msg.b_r = 0.0;
    out_msg.b_u = 0.0;
    out_msg.u = estimate_velocity();
    out_msg.psi_deg = yaw_ * 180 / PI;
    rc_pub_.publish(out_msg);

}

double MCEstimator::estimate_velocity()
{
  xdot = (2*tau_-dt_)/(2*tau_+dt_)*xdot_d1+(2/(2*tau_+dt_))*(x_ - x_d1);
  x_d1 = x_;
  xdot_d1 = xdot;

  ydot = (2*tau_-dt_)/(2*tau_+dt_)*ydot_d1+(2/(2*tau_+dt_))*(y_ - y_d1);
  y_d1 = y_;
  ydot_d1 = ydot;

  zdot = (2*tau_-dt_)/(2*tau_+dt_)*zdot_d1+(2/(2*tau_+dt_))*(z_ - z_d1);
  z_d1 = z_;
  zdot_d1 = zdot;

  return sqrt(pow(xdot,2)+pow(ydot,2)+pow(zdot,2));
}
} //end namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estimator_node");

    ros::NodeHandle nh;

    mc_estimate::MCEstimator est;

    ros::spin();
    return 0;
}
