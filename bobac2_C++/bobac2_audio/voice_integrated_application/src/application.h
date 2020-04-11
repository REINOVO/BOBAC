#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "voice_msgs/collect.h"
#include "voice_msgs/awake.h"
#include "voice_msgs/mode_choose.h"
#include "voice_msgs/control.h"
#include "voice_msgs/voice_nav.h"
#include "voice_msgs/aiui_server.h"
#include "voice_msgs/ss_server.h"

using namespace std;

class Application
{
public:
	Application();
	~Application();
    
private:
    int i, position_count;
	bool wakeup_flag;
	int mode_choose_flag;
	string aplay_file;
	geometry_msgs::Twist vel;	

    vector<string> position_name;
    vector<double> position_x;
    vector<double> position_y;
    vector<double> posture_r;
    vector<double> posture_p;
    vector<double> posture_y;

	ros::NodeHandle m_handle;
	ros::Publisher vcmd_pub;
	ros::Publisher cancel_pub;
	ros::ServiceClient audio_client;
	ros::ServiceClient awake_client;
	ros::ServiceClient mode_choose_client;
	ros::ServiceClient control_client;
	ros::ServiceClient voice_nav_client;
	ros::ServiceClient aiui_client;
	ros::ServiceClient tts_client;

	voice_msgs::collect audio_srv;	
	voice_msgs::awake awake_srv;
	voice_msgs::mode_choose mode_choose_srv;
	voice_msgs::control control_srv;
	voice_msgs::voice_nav voice_nav_srv;
	voice_msgs::aiui_server aiui_srv;
	voice_msgs::ss_server  tts_srv;
	timeval chat_start, chat_end, nav_start, nav_end,  control_start, control_end, mode_start, mode_end;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> AC;
	move_base_msgs::MoveBaseGoal goal;
	actionlib_msgs::GoalID first_goal;
	void position_read();
	void run();

};
