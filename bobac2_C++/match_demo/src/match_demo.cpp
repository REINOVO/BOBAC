#include <ros/ros.h>
#include <stdlib.h>
#include "bobac2_msgs/MatchDemo.h"
#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Quaternion.h>
using namespace std;

move_base_msgs::MoveBaseGoal goal;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> AC;
vector<string> position_name;
vector<double> position_x;
vector<double> position_y;
vector<double> posture_r;
vector<double> posture_p;
vector<double> posture_y;
vector<double> orientation_w;

int num[6];
void position_read()
{
  FILE* fp;
  char str[100];
  double tmp;

  if((fp = fopen("./source/voice_nav/position_info.txt", "r")) < 0){
          cout<<"open position info  file  error！！"<<endl;
  }

  while(1){
    if(fscanf(fp, "%s", str) == EOF)
            break;
    position_name.push_back(str);

    if(fscanf(fp,"%lf",&tmp) == EOF)
            break;
    position_x.push_back(tmp);

    if(fscanf(fp,"%lf",&tmp) == EOF)
            break;
    position_y.push_back(tmp);

    if(fscanf(fp,"%lf",&tmp) == EOF)
            break;
    posture_r.push_back(tmp);

    if(fscanf(fp,"%lf",&tmp) == EOF)
            break;
    posture_p.push_back(tmp);

    if(fscanf(fp,"%lf",&tmp) == EOF)
            break;
    posture_y.push_back(tmp);

  }
  fclose(fp);
}
bool nav(move_base_msgs::MoveBaseGoal m_goal){
   AC ac("move_base",true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  cout<< m_goal.target_pose.pose.orientation.z<<endl;
  ac.sendGoal(m_goal);
  while(ros::ok()){
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("arrive the goal");
      return true ;
      break;
    }
//    ROS_INFO("moving");
  }
}
bool move_cb(bobac2_msgs::MatchDemo::Request &req, bobac2_msgs::MatchDemo::Response &res){
    AC ac("move_base",true);
    if(req.start){
      int j;
      for(int i=0; i<5; i++){
        j = num[i]-1;
        cout<<position_name[j]<<endl;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = position_x[j];
        goal.target_pose.pose.position.y = position_y[j];
        geometry_msgs::Quaternion  quat = tf::createQuaternionMsgFromRollPitchYaw(posture_r[j],posture_p[j],posture_y[j]);
        goal.target_pose.pose.orientation = quat;
	      if(nav(goal)){
          ROS_INFO("reach the goal %d", num[i]);
        }
        else{ 
          ROS_INFO("the goal %d is failed", num[i]);
          return false;
        }
        switch(num[i]){
          case 1:
            system("aplay ./source/AIUI/audio/shanghai.wav");
            break;
          case 2:
            system("aplay ./source/AIUI/audio/beijing.wav");
            break;
          case 3:
            system("aplay ./source/AIUI/audio/tianjin.wav");
            break;
          case 4:
            system("aplay ./source/AIUI/audio/shenzhen.wav");
            break;
          case 5:
            system("aplay ./source/AIUI/audio/guangzhou.wav");
            break;
        }
      }
        goal.target_pose.pose.position.x = position_x[5];
        goal.target_pose.pose.position.y = position_y[5];
        geometry_msgs::Quaternion  quat = tf::createQuaternionMsgFromRollPitchYaw(posture_r[5],posture_p[5],posture_y[5]);
        goal.target_pose.pose.orientation = quat;
        if(nav(goal)){
          res.msg = "compeleted";
          system("roslaunch voice_integrated_application voice_integrated_application.launch");
          return true;
        }
        else{ 
          res.msg = "failed";
          system("roslaunch voice_integrated_application voice_integrated_application.launch");
        }
      
      
    }
    else{
        ROS_INFO("waiting start flag");
        res.msg = "error command";
    }
    return false;
}
int main(int argc, char** argv){
    ros::init(argc, argv, "match_demo");
    ros::NodeHandle nh;
    ros::ServiceServer move_ser = nh.advertiseService("match_start", move_cb);
    ros::param::get("~first", num[0]);
    ros::param::get("~second", num[1]);
    ros::param::get("~third", num[2]);
    ros::param::get("~fourth", num[3]);
    ros::param::get("~fifth", num[4]);
    position_read();
  //  relative_client = nh.serviceClient<bobac2_msgs::SetRelativeMove>("relative_move");
    ros::spin();
    return 0;
}
