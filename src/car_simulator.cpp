#include "ros/ros.h"

#include "std_msgs/UInt16.h"

#include "um_acc/ControlSignal.h"
#include "um_acc/CarStateExtended.h"

#define CAR_MASS 9.070000
#define CAR_F0 0.907000
#define CAR_F1 0.005000
#define CAR_F2 0.000250

// Update frequency [hz]
double rate = 100;

// Simulator constants
double current_vl = 2;
double current_v = 0.1; // Initial value
double current_h = 10;  // Initial value
double current_u = CAR_F0;  // Initial value - speed does not go negative

// Rolling resistance
double Fr(const double & v) {
  return CAR_F0 + CAR_F1*v + CAR_F2*pow(v,2);
}

// Callback that sets new control
void applyControl(const um_acc::ControlSignal& msg)
{
  current_u = msg.u;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "um_acc");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("control_signal", 10, applyControl);
  ros::Publisher pub = n.advertise<um_acc::CarStateExtended>("car_state", 10);
  ros::Publisher rpm_pub = n.advertise<std_msgs::UInt16>("hall_data", 10);
  
  ros::Rate loop_rate(rate);

  // check for initial conditions
  if (argc > 1)
    current_v = atof(argv[1]);
  if (argc > 2)
    current_h = atof(argv[2]);
  if (argc > 3)
    current_vl = atof(argv[3]);
  
  double delta_s = 0;

  while (ros::ok()) {
    // Update using Euler forward
    double dt = 1/rate;
    double vdot = (current_u - Fr(current_v))/CAR_MASS;
    current_v = current_v + dt*vdot;
    current_h = current_h + dt*(current_vl - current_v);
    delta_s = delta_s + dt*current_v;

    ROS_INFO("Current state u=%.2f, v=%.2f, h=%.2f, vl=%.2f", current_u, current_v, current_h, current_vl);

    um_acc::CarStateExtended msg;
    msg.v = current_v;
    msg.h = current_h;
    msg.vl = current_vl;
    pub.publish(msg);

    if (delta_s >= 0.00932 * 30) {
      std_msgs::UInt16 rpm_msg;
      rpm_msg.data = 1;
      rpm_pub.publish(rpm_msg);
      delta_s = 0;
    }

    // Achieve loop rate
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
