#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <vector>

#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include "ros/ros.h"
#include "voice_msgs/voice_nav.h"

#define BUFFER_SIZE     4096
#define FRAME_LEN       640 
#define HINTS_SIZE  100

using namespace std;

class VoiceNav 
{
public:
	VoiceNav();
	~VoiceNav();

	string userwords_file;
	vector<string> position_name;
	vector<double> position_x;
	vector<double> position_y;
	vector<double> posture_r;
	vector<double> posture_p;
	vector<double> posture_y;
	
	void login();
	void logout();
	void position_read();
	int upload_userwords();	
	string run_iat(const char* audio_file);
private:
	ros::NodeHandle m_handle;
	ros::ServiceServer voice_nav_service;
	bool voice_nav_deal(voice_msgs::voice_nav::Request &req,voice_msgs::voice_nav::Response &res);
	
};
