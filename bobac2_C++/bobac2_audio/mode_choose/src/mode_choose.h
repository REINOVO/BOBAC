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
#include "voice_msgs/mode_choose.h"

#define BUFFER_SIZE     4096
#define FRAME_LEN       640 
#define HINTS_SIZE  100

using namespace std;

class ModeChoose 
{
public:
	ModeChoose();
	~ModeChoose();

	string userwords_file;
	void login();
	void logout();
	int upload_userwords();	
	string run_iat(const char* audio_file);
private:
	ros::NodeHandle m_handle;
	ros::ServiceServer mode_choose_service;
	bool mode_choose_deal(voice_msgs::mode_choose::Request &req,voice_msgs::mode_choose::Response &res);
	
};
