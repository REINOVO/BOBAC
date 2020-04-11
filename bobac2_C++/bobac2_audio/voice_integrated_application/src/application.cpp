#include "application.h"
string word = "到达"  ;
Application::Application()
{
	wakeup_flag = false;
	mode_choose_flag = 0;
	vcmd_pub = m_handle.advertise<geometry_msgs::Twist>("cmd_vel",1);	
	cancel_pub = m_handle.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);

	audio_client = m_handle.serviceClient<voice_msgs::collect>("collect");
	awake_client = m_handle.serviceClient<voice_msgs::awake>("awake");
	mode_choose_client = m_handle.serviceClient<voice_msgs::mode_choose>("mode_choose");
	control_client = m_handle.serviceClient<voice_msgs::control>("control");
	voice_nav_client = m_handle.serviceClient<voice_msgs::voice_nav>("voice_nav");
	aiui_client = m_handle.serviceClient<voice_msgs::aiui_server>("aiui");
	tts_client = m_handle.serviceClient<voice_msgs::ss_server>("tts");
	position_read();
	run();
}

Application::~Application()
{

}

void Application::position_read()
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
	position_count = position_name.size();
}

void Application::run()
{
	ros::Rate loop(10);
	system("aplay  ./source/user_audio/Introduction.wav");
	while(ros::ok()){
		audio_srv.request.collect_flag = 1;
		while(!wakeup_flag){
			if(audio_client.call(audio_srv)){
				awake_srv.request.audio_file = audio_srv.response.ret;
				if(awake_client.call(awake_srv)){
					//ROS_INFO("!!!!!!!");
					if(awake_srv.response.awake_flag){
						wakeup_flag = true;
						system("aplay  ./source/user_audio/awake.wav");
						//gettimeofday(&mode_start, null); 
					}
					else cout<< " 休眠中，请用唤醒词唤醒。" << endl;
				}
			}
		}
		ROS_INFO("mode_flag : %d", mode_choose_flag);
		if(mode_choose_flag == 0){
			wakeup_flag = false;            
			if(audio_client.call(audio_srv)){            
				aiui_srv.request.audio_file = audio_srv.response.ret;
				if(!aiui_client.call(aiui_srv)){
					cout<<"aiui response get failed"<<endl;
				}
				if(!aiui_srv.response.nlp_str.empty() || !aiui_srv.response.iat_str.empty()){
					if(aiui_srv.response.iat_str.find("切换模式") != string::npos){
						mode_choose_flag = 3;
						system("aplay  ./source/user_audio/change_mode.wav");
					}
					else{
						tts_srv.request.text = aiui_srv.response.nlp_str; 
						if(tts_client.call(tts_srv)) {
							aplay_file = tts_srv.response.voice_file;

							/*  pid_t pid; 
							    pid = fork(); 
							    if(pid < 0) {
							    cout<<"fork error"<<endl;
							    } else if(pid == 0) {
							    execlp("aplay","aplay",aplay_file.c_str(),(char*)0); 
							    } else if(pid > 0) {
							    int exitcode = 0,ret;
							    ret = wait(&exitcode);
							    if(ret == -1) {
							    cout<<"have no child process found"<<endl;
							    }
							    }*/
							system("aplay  ./source/AIUI/audio/tts.wav");
						}
					}
				}
			}
		}
		if(mode_choose_flag == 1){ 
			if(audio_client.call(audio_srv)){
				wakeup_flag = false;            
				control_srv.request.audio_file = audio_srv.response.ret;
				if(!control_client.call(control_srv)){
					cout<<"--control order get failed!--"<<endl;
				}	
				if(control_srv.response.control_order == "前进"){					
					vel.linear.x = 0.3;
					vcmd_pub.publish(vel);
					sleep(8);
					vel.linear.x = 0;
					vcmd_pub.publish(vel);
				}
				else if(control_srv.response.control_order == "后退"){					
					vel.linear.x = -0.3;
					vcmd_pub.publish(vel);
					sleep(8);
					vel.linear.x = 0;
					vcmd_pub.publish(vel);
				}
				else if(control_srv.response.control_order == "左转"){					
					vel.angular.z = 0.3;
					vcmd_pub.publish(vel);
					sleep(8);
					vel.angular.z = 0;
					vcmd_pub.publish(vel);
				}
				else if(control_srv.response.control_order == "右转"){					
					vel.angular.z = -0.3;
					vcmd_pub.publish(vel);
					sleep(4);
					vel.angular.z = 0;
					vcmd_pub.publish(vel);
				}
				else if(control_srv.response.control_order == "切换模式"){					
					mode_choose_flag = 3;
					system("aplay  ./source/user_audio/change_mode.wav");
				}
				//else
			}
		}
		if(mode_choose_flag == 2){
			actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
			string position_info;
			if(audio_client.call(audio_srv)){
				wakeup_flag = false; 
				voice_nav_srv.request.audio_file = audio_srv.response.ret;
				if(!voice_nav_client.call(voice_nav_srv)){
					cout<<"voice nav position get failed"<<endl;
				}

				if(voice_nav_srv.response.nav_goal == "切换模式"){					
					mode_choose_flag = 3;
					system("aplay  ./source/user_audio/change_mode.wav");
					goto end_flag;
				}
				if(voice_nav_srv.response.nav_goal == "取消导航"){					
					cancel_pub.publish(first_goal);
					system("aplay  ./source/user_audio/cancel_nav.wav");
					goto end_flag;
				}
				if(voice_nav_srv.response.nav_goal == "休眠"){			
						while(!ac.waitForServer(ros::Duration(5.0))){
							ROS_INFO("Waiting for the move_base action server to come up");
						}
						int sleep_num = position_name.size()-1;
						goal.target_pose.header.frame_id = "map";
						goal.target_pose.header.stamp = ros::Time::now();

						goal.target_pose.pose.position.x = position_x[sleep_num];
						goal.target_pose.pose.position.y = position_y[sleep_num];	

						geometry_msgs::Quaternion  quat_all = tf::createQuaternionMsgFromRollPitchYaw(posture_r[sleep_num],posture_p[sleep_num],posture_y[sleep_num]);
						goal.target_pose.pose.orientation = quat_all;

						ROS_INFO("Sending goal");
						ac.sendGoal(goal);
						while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
							ROS_INFO("moving, please wait.");
						}
						ROS_INFO("goal has reached.");
					goto end_flag;
				}

				if(voice_nav_srv.response.nav_goal == "导游"){			
					for(int j=0; j<position_name.size()-1; j++ ){
						while(!ac.waitForServer(ros::Duration(5.0))){
							ROS_INFO("Waiting for the move_base action server to come up");
						}

						goal.target_pose.header.frame_id = "map";
						goal.target_pose.header.stamp = ros::Time::now();

						goal.target_pose.pose.position.x = position_x[j];
						goal.target_pose.pose.position.y = position_y[j];	

						geometry_msgs::Quaternion  quat_all = tf::createQuaternionMsgFromRollPitchYaw(posture_r[j],posture_p[j],posture_y[j]);
						goal.target_pose.pose.orientation = quat_all;

						ROS_INFO("Sending goal");
						ac.sendGoal(goal);
						while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
							ROS_INFO("moving, please wait.");
						}
						ROS_INFO("goal has reached.");
						position_info = word + position_name[j];
						tts_srv.request.text = position_info;
						if(tts_client.call(tts_srv)) {
							aplay_file = tts_srv.response.voice_file;
							system("aplay  ./source/AIUI/audio/tts.wav");
						}
					}
					goto end_flag;
				}

				for(i=0;i<position_name.size();i++){
					if(voice_nav_srv.response.nav_goal == position_name[i])
						break;
				}
				if(i == position_name.size()){
					cout<<"error position name "<<endl;
					system("aplay  ./source/user_audio/nav_position.wav");
				}	
				else{
					while(!ac.waitForServer(ros::Duration(5.0))){
						ROS_INFO("Waiting for the move_base action server to come up");
					}

					goal.target_pose.header.frame_id = "map";
					goal.target_pose.header.stamp = ros::Time::now();

					goal.target_pose.pose.position.x = position_x[i];
					goal.target_pose.pose.position.y = position_y[i];	

					geometry_msgs::Quaternion  quat = tf::createQuaternionMsgFromRollPitchYaw(posture_r[i],posture_p[i],posture_y[i]);
					goal.target_pose.pose.orientation = quat;

					ROS_INFO("Sending goal");
					ac.sendGoal(goal);
					while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
						ROS_INFO("moving, please wait.");
					}
					ROS_INFO("goal has reached.");
					position_info = word + position_name[i];
					tts_srv.request.text = position_info;
					if(tts_client.call(tts_srv)) {
						aplay_file = tts_srv.response.voice_file;
						system("aplay  ./source/AIUI/audio/tts.wav");
					}
				}
			}
end_flag:;
		}
		if(mode_choose_flag == 3){
			wakeup_flag = true;
			if(audio_client.call(audio_srv)){  
				mode_choose_srv.request.audio_file = audio_srv.response.ret;
				if(!mode_choose_client.call(mode_choose_srv)){
					cout<<"--mode choose error!--"<<endl;
				}	
				if(mode_choose_srv.response.mode == "控制模式"){
					mode_choose_flag = 1;
					system("aplay  ./source/user_audio/control.wav");
				}
				else if(mode_choose_srv.response.mode == "导航模式"){
					mode_choose_flag = 2;
					system("aplay  ./source/user_audio/nav.wav");
				}
				else if(mode_choose_srv.response.mode == "闲聊模式"){
					mode_choose_flag = 0;
					system("aplay  ./source/user_audio/chat.wav");
				}
				else{
					cout<<"模式未能识别"<<endl;
					mode_choose_flag = 3;
					system("aplay  ./source/user_audio/mode_error.wav");
				}
			}
		}
	}

}


int main(int argc,char** argv)
{
	ros::init(argc,argv,"voice_integrated");	
	Application application;
	ros::spin();

}
