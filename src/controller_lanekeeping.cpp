#include "ros/ros.h"

#include "um_acc/ControlSignal.h"
#include "um_acc/ComputeControl_L.h"
#include "um_acc/CarState_L.h"
#include "um_acc/Pwm.h"
#include <algorithm>

double y = 0;
double v = 0;
double phi = 0;
double r = 0;

double drive_pwm = 0;

bool new_state = false;

void stateSub(const um_acc::CarState_L::ConstPtr & msg) {
  y = msg->y;
  v = msg->v;
  phi = msg->phi;
  r = msg->r;
  new_state = true;
}

void driveSub(const um_acc::Pwm::ConstPtr & msg) {
	drive_pwm = msg->vel_pwm;
}

int main(int argc, char **argv)
{
  //create ROS node
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;

  //create serviceClient, publisher, subscriber, & service
  ros::ServiceClient control_client = n.serviceClient<um_acc::ComputeControl_L> ("compute_control");
  ros::Subscriber state_sub = n.subscribe<um_acc::CarState_L> ("lane_data", 1000, stateSub);
  ros::Subscriber drive_sub = n.subscribe<um_acc::Pwm> ("drive_control", 1000, driveSub);
  ros::Publisher control_pub = n.advertise<um_acc::ControlSignal> ("control_signal", 1000);
  ros::Publisher pwm_pub = n.advertise<um_acc::Pwm> ("pwm_control", 1000);
  um_acc::ComputeControl_L srv;

  //create loop rate
  ros::Rate loop_rate(10);

  double const_speed = 1;

  while(ros::ok())
    {
      srv.request.y = y;
      srv.request.v = v;
      srv.request.phi = phi;
      srv.request.r = r;

      ROS_INFO("Received State: y=%f v=%f phi=%f r=%f", srv.request.y, srv.request.v, srv.request.phi, srv.request.r);

      if (control_client.call(srv)) {
	um_acc::ControlSignal u_msg;
	um_acc::Pwm control;
	u_msg.u = srv.response.u;
	control.vel_pwm = drive_pwm;
	control.steer_pwm = 1420 - srv.response.u*50 - 50;
	ROS_INFO("Velocity PWM: %d Steer PWM: %d", control.vel_pwm, control.steer_pwm);
	control_pub.publish(u_msg); // Publish new control
	pwm_pub.publish(control); // Publisher pwm control
	new_state = false;
      } else {
	ROS_WARN("Could not compute control for y=%f v=%f phi=%f r=%f", srv.request.y, srv.request.v, srv.request.phi, srv.request.r);
      }
      ros::spinOnce();
    }

  ros::spin();
  return 0;
}