#include "ros/ros.h"

#include "um_acc/CarStateExtended.h"
#include "um_acc/ControlSignal.h"
#include "um_acc/ComputeControl.h"
#include "um_acc/CarSignal.h"

double v = 0;
double h = 0;
double vl = 0;

bool new_state = false;

void stateSub(const um_acc::CarStateExtended::ConstPtr & msg) {
  v = msg->v;
  h = msg->h;
  vl = msg->vl;
  new_state = true;
}

int main(int argc, char **argv)
{
  //create ROS node
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;

  //create serviceClient, publisher, subscriber, & service
  ros::ServiceClient control_client = n.serviceClient<um_acc::ComputeControl> ("compute_control");
  ros::Subscriber state_sub = n.subscribe<um_acc::CarStateExtended> ("car_state", 1000, stateSub);
  ros::Publisher control_pub = n.advertise<um_acc::ControlSignal> ("control_signal", 1000);
  um_acc::ComputeControl srv;

  //create loop rate
  ros::Rate loop_rate(10);

  double const_speed = 1;

  while(ros::ok())
    {
      srv.request.v = v;
      srv.request.h = h;
      srv.request.vl = vl;

      if (control_client.call(srv)) {
	um_acc::ControlSignal u_msg;
	u_msg.u = srv.response.u;
	control_pub.publish(u_msg); // Publish new control
	new_state = false;
      } else {
	ROS_WARN("Could not compute control for v=%f h=%f vl=%f", srv.request.v, srv.request.h, srv.request.vl);
      }
      ros::spinOnce();
    }

  ros::spin();
  return 0;
}
