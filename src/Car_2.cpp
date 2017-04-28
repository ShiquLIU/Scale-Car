#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <um_acc/Pwm.h>
#include <um_acc/Sensor.h>
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include <pthread.h>

unsigned int STEER_PIN = 2;
unsigned int DRIVE_PIN = 0;

void pwmCallback(const um_acc::Pwm::ConstPtr& msg)
{
	pwm_set_duty(STEER_PIN, msg->steer_pwm*1000);
	pwm_set_duty(DRIVE_PIN, msg->vel_pwm*1000);
};

int main(int argc, char *argv[])
{
	pwm_export(STEER_PIN);
	pwm_set_period(STEER_PIN, 20000000);
	pwm_set_duty(STEER_PIN, 1500000);
	pwm_enable(STEER_PIN);

	pwm_export(DRIVE_PIN);
	pwm_set_period(DRIVE_PIN, 20000000);
	pwm_set_duty(DRIVE_PIN, 1500000);
	pwm_enable(DRIVE_PIN);

	ros::init(argc, argv, "car_control");
	ros::NodeHandle ncar;
	ros::Subscriber sub = ncar.subscribe("pwm_control", 1000, pwmCallback);

	ros::spin();
}
