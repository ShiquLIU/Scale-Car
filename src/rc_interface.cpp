#include "ros/ros.h"
#include "um_acc/ControlSignal.h"
#include "um_acc/CarStateExtended.h"
#include "um_acc/ComputeControl.h"
#include "kalman.h"

#include <std_msgs/Float64.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"

#define CAR_MASS 9.070000
#define CAR_F0 0.907000
#define CAR_F1 0.005000
#define CAR_F2 0.000250

using namespace std;

// Update frequency [hz]
double rate = 200;
double dt = 1/rate;
unsigned int itcount = 0;

// pwm pub
double input_pwm = 1420;
int pwm_low = 1435;
int pwm_high = 1350;

// measured vel
double actual_vel = 0;
double current_h = 5;
double current_vl = 0;
double vel_error;

// P controller
double Kp_brake = 1.2;     // 0.8
double Kp_acc = 1.1;     // 0.8 

// Kalman variables
Matrix4d kalman_P = Matrix4d::Identity();
Vector4d kalman_x = Vector4d::Zero();

// some constants
double convert_rpm_to_vel = 0.00932;
double convert_vel_to_rpm = 1/ 0.00932;

// sensor readings
double dist_sensor = 5/(0.00562*2);
double rpm_sensor = 0;

// Simulator constants
double internal_vel= 0.1; // Initial value
bool internal_vel_initialized = false;

// control
double current_u = CAR_F0;  

// variables pertaining to RPM estimation
ros::Time rpm_clock;
double rpm_clock_treshold = 0.5;

// ******************* Setting up the Subscribers call backs for ROS ****************//

void hall_callback(const std_msgs::UInt16& msg) {
  ROS_INFO ("Received hall_data");
  rpm_sensor = msg.data;
  actual_vel = rpm_sensor * convert_rpm_to_vel;
  rpm_clock = ros::Time::now();
}

void distance_callback(const std_msgs::Int16& msg) {
  dist_sensor = msg.data;
}

void reset_internal_cb(const std_msgs::Bool& msg) {
  ROS_INFO ("Received reset_internal");
  if (msg.data) {
    internal_vel_initialized = false;
  }
}

// ******************* ******************* ******************* ****************//


void calc_speed(){

  vel_error = actual_vel - internal_vel;

  double Kp = 0;

  if (vel_error < 0) {
    Kp = Kp_acc;
  } else {
    Kp = Kp_brake; 
  }

  double P = vel_error*Kp;

  input_pwm = input_pwm + P;  // this value needs to be scaled or mapped to the operating range of angles

  if (abs(input_pwm)< pwm_high) {
    input_pwm = pwm_high;
  } 

  if (abs(input_pwm)> pwm_low) {
    input_pwm = pwm_low;
    // actual_vel = 0;
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_interface");

  ros::NodeHandle n;
  
  // subscribe to acc velocity
  ros::Subscriber sub_hall = n.subscribe("hall_data", 1000, hall_callback);

  // subscribe to reset
  ros::Subscriber sub_reset = n.subscribe("reset_internal", 1000, reset_internal_cb);

  // subscribe to encoder
  ros::Subscriber sub_distance = n.subscribe("encoder0_data", 1000, distance_callback);

  // publish state of system
  ros::Publisher pub_state = n.advertise<um_acc::CarStateExtended>("car_state", 1000);

  // publish pwm
  ros::Publisher pub_pwm = n.advertise<std_msgs::UInt16>("motor_pub", 1000);

  // control computation server
  ros::ServiceClient control_client = n.serviceClient<um_acc::ComputeControl> ("compute_control");

  um_acc::ComputeControl srv_control;

  ros::Rate loop_rate(rate);

  while (ros::ok()) {

    // update internal velocity
    double Fr = CAR_F0 + CAR_F1*internal_vel+ CAR_F2*pow(internal_vel,2);
    double vdot = (current_u - Fr)/CAR_MASS;
    internal_vel += dt*vdot;

    // update distance
    current_h = (dist_sensor)*0.00562*2; 

    // update lead car speed through Kalman filter
    Vector2d y;
    y << internal_vel, current_h;
    kalman(kalman_x, kalman_P, y, current_u, dt);
    current_vl = kalman_x(2);

    // compute control
    if (itcount % 20 == 0) {
      srv_control.request.v = internal_vel;
      srv_control.request.h = current_h;
      srv_control.request.vl = current_vl;
      if (control_client.call(srv_control)) {
	current_u = srv_control.response.u;
      } else {
	current_u = CAR_F0;
	ROS_WARN("Could not compute control for v=%f h=%f vl=%f", srv_control.request.v, srv_control.request.h, srv_control.request.vl);
      }
    }

    // reset internal vel to measured vel
    if (!internal_vel_initialized) {
      internal_vel = actual_vel;
      internal_vel_initialized = true;
    }

    // update velocity estimate
    if ( (ros::Time::now() - rpm_clock).toSec() > rpm_clock_treshold) {
      rpm_sensor = 0;
      actual_vel = 0;
    }

    // hack to keep speed positive
    if (internal_vel < 0) internal_vel = 0;

    // compute and publish desired pwm
    if (itcount % 10 == 0) {
      calc_speed();
      std_msgs::UInt16 msg_pwm;
      msg_pwm.data = input_pwm;
      pub_pwm.publish(msg_pwm);
    }

    // publish current state
    if (itcount % 20 == 0) {
      um_acc::CarStateExtended msg_state;
      msg_state.v = internal_vel;
      msg_state.h = current_h;
      msg_state.vl = current_vl;
      msg_state.actual_v = actual_vel;
      msg_state.u = current_u;
      msg_state.rpm = rpm_sensor;
      msg_state.pwm = input_pwm;
      pub_state.publish(msg_state);
    }

    // Achieve loop rate
    ros::spinOnce();
    loop_rate.sleep();

    ++itcount;
  }

  return 0;
}
