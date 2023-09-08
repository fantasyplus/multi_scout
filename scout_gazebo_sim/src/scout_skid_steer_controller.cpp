#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <string>

// #include "scout_gazebo/scout_skid_steer.hpp"
std::string robot_name_;
std::string motor_fr_topic_;
std::string motor_fl_topic_;
std::string motor_rl_topic_;
std::string motor_rr_topic_;
std::string cmd_topic_;

const double SCOUT_WHEELBASE = 0.498;
const double SCOUT_WHEEL_RADIUS = 0.16459;

ros::NodeHandle *nh_;

ros::Publisher motor_fr_pub_;
ros::Publisher motor_fl_pub_;
ros::Publisher motor_rl_pub_;
ros::Publisher motor_rr_pub_;

ros::Subscriber cmd_sub_;

void TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {
  // ROS_INFO("asdasdas");
  std_msgs::Float64 motor_cmd[4];

  double driving_vel = msg->linear.x;
  double steering_vel = msg->angular.z;

  double left_side_velocity =
      (driving_vel - steering_vel * SCOUT_WHEELBASE) / SCOUT_WHEEL_RADIUS;
  double right_side_velocity =
      (driving_vel + steering_vel * SCOUT_WHEELBASE) / SCOUT_WHEEL_RADIUS;

  motor_cmd[0].data = right_side_velocity;
  motor_cmd[1].data = -left_side_velocity;
  motor_cmd[2].data = -left_side_velocity;
  motor_cmd[3].data = right_side_velocity;

  // ROS_INFO("motor_cmd: %f, %f, %f, %f", motor_cmd[0].data, motor_cmd[1].data,
  //          motor_cmd[2].data, motor_cmd[3].data);

  motor_fr_pub_.publish(motor_cmd[0]);
  motor_fl_pub_.publish(motor_cmd[1]);
  motor_rl_pub_.publish(motor_cmd[2]);
  motor_rr_pub_.publish(motor_cmd[3]);
}


int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "scout_odom");
  ros::NodeHandle node(""), private_node("~");

  // fetch parameters
  std::string robot_namespace;
  private_node.param<std::string>("robot_namespace", robot_namespace,
                                  std::string(""));

  ROS_INFO("Namespace: %s", robot_namespace.c_str());

  // ScoutSkidSteer skid_steer_controller(&node, robot_namespace);
  // skid_steer_controller.SetupSubscription();

  motor_fr_topic_ = robot_namespace + "/scout_motor_fr_controller/command";
  motor_fl_topic_ = robot_namespace + "/scout_motor_fl_controller/command";
  motor_rl_topic_ = robot_namespace + "/scout_motor_rl_controller/command";
  motor_rr_topic_ = robot_namespace + "/scout_motor_rr_controller/command";
  cmd_topic_ = robot_namespace + "/cmd_vel";

  // command subscriber
  cmd_sub_ = node.subscribe<geometry_msgs::Twist>(
      cmd_topic_, 1, &TwistCmdCallback);

  // motor command publisher
  motor_fr_pub_ = node.advertise<std_msgs::Float64>(motor_fr_topic_, 50);
  motor_fl_pub_ = node.advertise<std_msgs::Float64>(motor_fl_topic_, 50);
  motor_rl_pub_ = node.advertise<std_msgs::Float64>(motor_rl_topic_, 50);
  motor_rr_pub_ = node.advertise<std_msgs::Float64>(motor_rr_topic_, 50);
  ROS_INFO("SetupSubscription");

  ros::spin();

  return 0;
}