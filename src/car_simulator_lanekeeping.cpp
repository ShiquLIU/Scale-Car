#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include "um_acc/ControlSignal.h"
#include "um_acc/CarState_L.h"
#include "qp_vars.h"

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;

//qp to update vehicle state
qp_var qp;

//vehicle state
VectorXd x0(4,1);

// Update frequency [hz]
double rate = 10;

// Simulator constants
double initial_y = .25;
double initial_v = 0; // Initial value
double initial_phi = 0;  // Initial value
double initial_r = 0;  // Initial value
VectorXd current_u = VectorXd::Zero(1,1);

// Callback that sets new control
void applyControl(const um_acc::ControlSignal& msg)
{
  current_u(0) = msg.u;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "um_acc");

  VectorXd current_d = VectorXd::Zero(1,1);

  x0(0) = initial_y;
  x0(1) = initial_v;
  x0(2) = initial_phi;
  x0(3) = initial_r;

  qp.read_matlab();
  
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("control_signal", 10, applyControl);
  ros::Publisher pub = n.advertise<um_acc::CarState_L>("car_state", 10);
//  ros::Publisher rpm_pub = n.advertise<std_msgs::UInt16>("hall_data", 10);
  
  ros::Rate loop_rate(rate);
  
  // check for initial conditions
  if (argc > 1)
    x0(0) = atof(argv[1]);
  if (argc > 2)
    x0(1) = atof(argv[2]);
  if (argc > 3)
    x0(2) = atof(argv[3]);
  if (argc > 4)
    x0(3) = atof(argv[4]);

  while (ros::ok()) {

    current_u = current_u;

    x0 = qp.update_state(x0, current_u, current_d);  //no disturbance

    ROS_INFO("Current state u=%.2f, d = %.2f, y=%.2f, v=%.2f, phi=%.2f, r=%.2f", current_u(0), current_d(0), x0(0), x0(1), x0(2), x0(3));

    um_acc::CarState_L msg;
    msg.y = x0(0);
    msg.v = x0(1);
    msg.phi = x0(2);
    msg.r = x0(3);
    pub.publish(msg);
/*
   if (delta_s >= 0.00932 * 30) {
      std_msgs::UInt16 rpm_msg;
      rpm_msg.data = 1;
      rpm_pub.publish(rpm_msg);
      delta_s = 0;
    }  
*/
    // Achieve loop rate
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}