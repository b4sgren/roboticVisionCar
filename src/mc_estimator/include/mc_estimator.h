#ifndef MCESTIMATOR_H
#define MCESTIMATOR_H

#include <ros/ros.h>

// #include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>

#include <autopilot/State.h>
// #include <std_msgs/Float64MultiArray.h>

double PI{3.14159265359};

namespace mc_estimate
{

class MCEstimator
{
  public:
      MCEstimator();
      ~MCEstimator();

  private:
      ros::NodeHandle nh_;
      ros::NodeHandle nh_private_;

      ros::Subscriber pose_sub_;
      ros::Publisher rc_pub_;
      ros::Publisher state_pub_;

      autopilot::State out_msg;
      ros::Time prev_time_;

      float x_;
      double x_d1{0};
      double xdot{0};
      double xdot_d1{0};

      float y_;
      double y_d1{0};
      double ydot{0};
      double ydot_d1{0};

      float z_;
      double z_d1{0};
      double zdot{0};
      double zdot_d1{0};

      float roll_;
      float pitch_;
      float yaw_;

      double tau_{0.1}; //Gain on dirty derivative
      double dt_{0.0};

      void roll_pitch_yaw_from_quat(std::vector<float> q);
      void pose_callback(const geometry_msgs::PoseStampedPtr &msg);
      double estimate_velocity();
};
}

#endif //MCESTIMATOR_H
