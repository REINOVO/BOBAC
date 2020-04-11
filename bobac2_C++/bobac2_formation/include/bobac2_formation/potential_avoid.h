#ifndef _FOAMTION_POTENTIAL_H
#define _ROEMATION_POTENTIAL_H


#include <stdio.h>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>

using namespace Eigen;

#define MAX 100000000
#define k 6
#define c 12
#define p 1
#define ANGULAR_LIMIT 0.3

class Potential
{
public:
  Potential(std::string laser_topic, std::string cmd_topic);
  virtual ~Potential();
  double limit(double param, double max, double min);
  double gravitation(goal_x, goal_y, std::vector<double> robot_current_pose);
  double repulsion(double obstacle_distance, double obstacle_angle);
  double resultant_force(std::vector<double> gravitation_params, std::vector<double> repulsion_params);
private:
  /* data */
  double _gravitation_x_, _gravitation_y_, _gravitation_angle_, _gravitation_;
  double _repulsion_x_, _repulsion_y_, _repulsion_angle_, _repulsion_distance_, _repulsion_;
  double _resultant_force_x_, _resultant_force_y_, _resultant_force_angle_, _resultant_force_;
  double _velocity_x_, _velocity_y_, _velocity_theta_;
  double _temp1_, _temp2_;
};
#endif