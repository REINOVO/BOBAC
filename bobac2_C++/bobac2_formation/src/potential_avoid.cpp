/*
A class of Potential Field Method

一、gravitation_params
  in parameters: 
  （1）、(vector<double>)robot_current_pose(current_pose.x, current_pose.y, current_pose.theta)
  （2）、(double)goal_x, goal_y;
  out parameters:
  （1）、(vector<double>)gravitation_goal(x, y, theta)  --------
二、repulsion_params                                           |
  in parameters:                                              | 
  （1）、(vector<double>)nestest_obstacle(distance, angle)     |
  out parameters:                                             | 
  （1）、(vector<double>)repulsion_goal(x, y, theta)-----------|-----
三、resultant_params                                           |    |
  in parameters:                                              |    |
  （1）、(vector<double>) gravitation_goal           <---------|    |
  （2）、(vector<double>)repulsion_goal              <---------------
params:
      _xx_: private 
      xx_: public
*/
#include "bobac2_formation/potential_avoid.h"
Potential::Potential(){
  _velocity_x_ = _velocity_y_ = _velocity_theta_ = 0;
  _tmep1_ = _tmep2_ = 0.0;
}
Potential::~Potential(){}

double Potential::limit(double param, double max, double min)
{
	if(param < min)
		return min;
	else if(param > max)
		return max;
	else 
	    return param;
}
std::vector<double> Potential::gravitation(goal_x, goal_y, std::vector<double> robot_current_pose){
  std::vector<double> gravitation_array;
  double angle_of_OriginToGaol, angle_of_OriginToRobot;
  double robotCos, robotSin;
  MatrixXd martrix1(3,3);
  MatrixXd martrix2(3,1);
  MatrixXd martrix3(3,3);
  MatrixXd martrix4(3,1);
  MatrixXd martrix5(3,1);

  angle_of_OriginToGaol = atan2(goal_y, goal_x);
  angle_of_OriginToRobot = atan2(robot_current_pose[1], robot_current_pose[0]);
  robotCos = cos(robot_current_pose[2]);
  robotSin = sin(robot_current_pose[2]);

  martrix1 << robotCos,-robotSin,0,robotSin,robotCos,0,0,0,1;
  martrix2 << robot_current_pose[0],robot_current_pose[1],angle_of_OriginToGaol;
  martrix3 << 1,0,0,0,1,0,0,0,1;
  martrix4 << robot_current_pose[0],robot_current_pose[1],angle_of_OriginToRobot;
  martrix5=martrix1.inverse()*(martrix2-martrix3*martrix4);
  _tmep1_ = martrix5(0,0);
  _tmep2_ = martrix5(1,0);
  double rEndx=fabs(_tmep1_);
	double rEndy=fabs(_tmep2_);
	double rEnd=sqrt((rEndx * rEndx)+(rEndy * rEndy));

	if(rEndx==0)
	{
		if(_tmep2_>robot_current_pose[1])
		{
			_gravitation_angle_=M_PI / 2;
		}
		else
		{
			_gravitation_angle_=-(M_PI / 2);
		}
	}
	else
	{
		_gravitation_angle_=atan2((_tmep2_),(_tmep1_));
	}

  _gravitation_ = k * rEnd;
  _gravitation_x_ = _gravitation_ * cos(_gravitation_angle_);
  _gravitation_y_ = _gravitation_ * sin(_gravitation_angle_);
  gravitation_array.push_back(_gravitation_x_);
  gravitation_array.push_back(_gravitation_y_);
  gravitation_array.push_back(_gravitation_angle_);

  return gravitation_array;
}

std::vector<double> Potential::repulsion(double obstacle_distance, double obstacle_angle){
  std::vector<double> repulsion_array;

  _repulsion_distance_ = obstacle_distance;
  if(_repulsion_distance_ > p){
    _repulsion_x_ = _repulsion_y_ = 0;
  } 
  else if(_repulsion_distance_>(p/2))
  {
    _repulsion_=c * ((1/_repulsion_distance_)-(1/p)) * (1/(_repulsion_distance_ * _repulsion_distance_));
    _repulsion_angle_=obstacle_angle;
    if(_repulsion_angle_>M_PI)
      {	
        _repulsion_angle_-=2*M_PI;
    }
      else if(_repulsion_angle_<-M_PI)
    {
        _repulsion_angle_+=2*M_PI;
    }
    _repulsion_x_=-_repulsion_ * cos(_repulsion_angle_);
    _repulsion_y_=-_repulsion_ * sin(_repulsion_angle_);
  }
  if(_repulsion_distance_>0.05&&_repulsion_distance_<(p/2))
  {
    _repulsion_=c * c * ((1/_repulsion_distance_)-(1/p)) * (1/(_repulsion_distance_ * _repulsion_distance_));
    _repulsion_angle_=obstacle_angle;
    if (_repulsion_angle_>M_PI)
      {
        _repulsion_angle_-=2*M_PI;
    }
      else if (_repulsion_angle_<-M_PI)
    {
        _repulsion_angle_+=2*M_PI;
    }
    _repulsion_x_=-_repulsion_ * cos(_repulsion_angle_);
    _repulsion_y_=-_repulsion_ * sin(_repulsion_angle_);
  }
  repulsion_array.push_back(_repulsion_x_);
  repulsion_array.push_back(_repulsion_y_);
  repulsion_array.push_back(_repulsion_angle_);
  return repulsion_array;
}

std::vector<double> Potential::potential_resultant_force(std::vector<double> gravitation_params, std::vector<double> repulsion_params){
  std::vector<double> resultant_vel;
  std::vector<double> gravitation_goal;
  std::vector<double> repulsion_goal;
  std::vector<double> robot_current_pose;
  double x_goal, y_goal;
  double nearest_obstacle_dist, nearest_obstacle_angle;

  x_goal = gravitation_params[0];
  y_goal = gravitation_params[1];
  robot_current_pose.push_back(gravitation_params[2]);
  robot_current_pose.push_back(gravitation_params[3]);
  robot_current_pose.push_back(gravitation_params[4]);

  nearest_obstacle_dist = repulsion_params[0];
  nearest_obstacle_angle = repulsion_params[1];

  gravitation_goal = gravitation(x_goal, y_goal, robot_current_pose);
  repulsion_goal = repulsion(nearest_obstacle_dist, nearest_obstacle_angle);
  
	if(fabs(gravitation_goal[2]-repulsion_goal[2])<0.13 || fabs(fabs(gravitation_goal[2]-repulsion_goal[2)-M_PI)<0.13)
	{
		_resultant_force_x_=gravitation_goal[0]+2*repulsion_goal[0];
		_resultant_force_y_=gravitation_goal[1]+repulsion_goal[1];
		_resultant_force_angle_=atan2(_resultant_force_y_,_resultant_force_x_);
	}
	else
	{
		_resultant_force_x_=gravitation_goal[0]+repulsion_goal[0];
		_resultant_force_y_=gravitation_goal[1]+repulsion_goal[1];
		_resultant_force_angle_=atan2(_resultant_force_y_,_resultant_force_x_);
	}
    //ROS_INFO("_resultant_force_x_....%f",x);
    //ROS_INFO("_resultant_force_y_...%f",y);
	_resultant_force_=_resultant_force_x_/cos(_resultant_force_angle_);
	_resultant_force_=limit(_resultant_force_, max_speed, 0);
	_velocity_x_=_resultant_force_;
	_velocity_x_ = limit(_resultant_force_, max_speed, -max_speed);
	_velocity_theta_=_resultant_force_angle_;
	_velocity_theta_ = limit(velocity_theta_, ANGULAR_LIMIT, -ANGULAR_LIMIT);
  resultant_vel.push_back(_velocity_x_);
  resultant_vel.push_back(_velocity_y_);
  resultant_vel.push_back(_velocity_angular_);
  return resultant_vel;
}
