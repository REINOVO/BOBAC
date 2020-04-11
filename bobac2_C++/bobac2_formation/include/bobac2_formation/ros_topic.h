#ifndef _FORMATION_ROS_TOPIC_H
#define _FORMATION_ROS_TOPIC_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

#define DEG2RAD(x) ((x)*M_PI/180.)

class NavRosTopic
{
public:
  NavRosTopic(std::string laser_topic, std::string cmd_topic);
  virtual ~NavRosTopic();
  std::vector<float> laser_ranges_;
	geometry_msgs::Twist cmd_vel_;//速度消息
  sensor_msgs::LaserScan laser_msgs_;
  double limit(double param, double max, double min);
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
  std::vector<double>  robot_pose(std::string global_frame, std::string robot_base_frame);
  double get_min_distance_in_range(float min_degree, float max_degree);

private:
  /* data */
  ros::NodeHandle _nh_;
  tf::TransformListener listener_;
  tf::StampedTransform transform_;
	ros::Publisher _cmd_vel_pub_;//发布
  ros::Subscriber _laser_sub_;//激光
  double _nearest_obstcale_angle_;
  double _nearest_obstcale_;
};


#endif