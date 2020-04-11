#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include "ros/ros.h"
#include "voice_msgs/control.h"

#define BUFFER_SIZE     4096
#define FRAME_LEN       640 
#define HINTS_SIZE  100

using namespace std;

class Control 
{
public:
	Control();
	~Control();

	string userwords_file;
	void login();
	void logout();
	int upload_userwords();	
	string run_iat(const char* audio_file);
private:
	ros::NodeHandle m_handle;
	ros::ServiceServer control_service;
	bool control_deal(voice_msgs::control::Request &req,voice_msgs::control::Response &res);
	
};
