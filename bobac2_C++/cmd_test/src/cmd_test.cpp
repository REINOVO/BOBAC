#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cmd_test");
  ros::NodeHandle nh;
  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",100);
  ros::Publisher pub1=nh.advertise<geometry_msgs::Twist>("cmd_vel_a",100);
  geometry_msgs::Twist msg;
  ros::Rate loop_rate(2);
  while(ros::ok()){
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.angular.z = 0.1;
    pub.publish(msg);
    msg.angular.z = 0.1;
    pub1.publish(msg);
    loop_rate.sleep();
  }
  return 0;
}
