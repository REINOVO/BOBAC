#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
using namespace std;
void call(){
    ros::spin();
}
void sub_cb(const geometry_msgs::Twist::ConstPtr &msg){
    cout<< "this is cmd : "<< msg->angular.z << endl;
}
void sub_cb_a(const geometry_msgs::Twist::ConstPtr &msg){
    cout<< "this is a : "<< msg->angular.z << endl;
}
int main(int argc, char *argv[]){
  ros::init(argc, argv, "cmd_sub");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cmd_vel", 10, sub_cb);
  ros::Subscriber sub1 = nh.subscribe("cmd_vel_a", 10, sub_cb_a);
  call();
  return 0;
}
