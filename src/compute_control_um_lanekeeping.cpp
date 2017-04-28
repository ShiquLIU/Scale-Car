#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <cmath>

#include "ros/ros.h"
#include "um_acc/ComputeControl_L.h"
#include "qp_vars.h"

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Input vector
VectorXd input;

// Feedback vector
VectorXd feedback;

//qp_var class
qp_var qp;

//current state
VectorXd x0(4,1);

//optimization problem weights
MatrixXd Rx(4,4);
VectorXd rx(4,1);
VectorXd Ru(1,1);
VectorXd ru(1,1);
VectorXd u(1,1);

bool compute(um_acc::ComputeControl_L::Request  &req,
         um_acc::ComputeControl_L::Response &res)
{

  x0(0) = req.y;
  x0(1) = req.v;
  x0(2) = req.phi;
  x0(3) = req.r;
  input = qp.get_input(x0, Rx, rx, Ru, ru);
  feedback = qp.get_feedback(x0);
  res.u = input(0) - feedback(0);
  return true;
}

int main(int argc, char **argv)
{
  //set optimization problem weights
  Rx << 	1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
  rx <<	1,
		0,
		1,
		0;
  Ru <<	.1;
  ru <<	.1;

  ros::init(argc, argv, "compute_control_um_lanekeeping");
  ros::NodeHandle n;

  qp.read_matlab();

  ros::ServiceServer service = n.advertiseService("compute_control", compute);
  ROS_INFO("Control signal computation intialized");
  ros::spin();

  return 0;
}