#include "bobac2_formation/rost_opic.h"

NavRosTopic::NavRosTopic(std::string laser_topic, std::string cmd_topic){
  _cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic.c_str(), 100);
  _laser_sub_ = nh_.subscribe(laser_topic.c_str(), 100, &NavRosTopic::laser_callback, this);
  _nearest_obstacle_ = 10.0;
  _nearest_obstacle_num_ = 0;
}

NavRosTopic::~NavRosTopic(){};

void NavRosTopic::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg){
  laser_msgs_ = *msg;
}
double limit(double param, double max, double min)
{
	if(param < min)
		return min;
	else if(param > max)
		return max;
	else 
	    return param;
}

double NavRosTopic::get_min_distance_in_range(float degree_max, float degree_min){
  bool reversed = (angle_max > angle_min);
  int up_offset, down_offset; 
  double angle_max, angle_min, min_distance;
  if ( reversed ) {
    angle_max = DEG2RAD(degree_max);
    angle_min = DEG2RAD(degree_min);
  } else {
    angle_min = DEG2RAD(degree_max);
    angle_max = DEG2RAD(degree_min);
  }
  angle_max = limit(angle_max, laser_msgs_.angle_max, laser_msgs_.angle_min);
  angle_min = limit(angle_min, laser_msgs_.angle_max, laser_msgs_.angle_min);
  down_offset = (int)((angle_min - laser_msgs_.angle_min)/laser_msgs_.angle_increment;
  up_offset = (int)((angle_max - laser_msgs_.angle_min)/laser_msgs_.angle_increment;
  min_distance = _nearest_obstacle_;
  for(int i=down_offset; i<up_offset+1; i++){
    if(laser_msgs_.rangs[i] < min_distance){
      min_distance = laser_msgs_.rangs[i];
      _nearest_obstacle_angle_ = laser_msgs_.angle_min + (i * laser_msgs_.angle_increment);
    }
  }
  return min_distance;
}

std::vector<double> NavRosTopic::robot_pose(std::string global_frame, std::string robot_base_frame);{
  std::vector<double> pose;
  try{
    listener_.lookupTransform(robot_base_frame.c_str(), global_frame.c_str(), ros::Time(0), transform_);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  pose.push_back(NavRosTopic::transform_.getOrigin().x());
  pose.push_back(NavRosTopic::transform_.getOrigin().y());
  pose.push_back(NavRosTopic::transform_.getRotation().z());
  return pose;

}

/*
int int main(int argc, char *argv[])
{
  ros::init(argc, argv, "potential_ros");
  NavRosTopic nav;
  ros::spin();
  return 0;
}
*/