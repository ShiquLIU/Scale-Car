#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <um_acc/Pwm.h>
#include <um_acc/Sensor.h>
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include <pthread.h>

unsigned int PING_GPIO = 161;

float microsecondsToCentimeters(float microseconds)
{
	return microseconds / 29 / 2;
};

int main(int argc, char *argv[])
{
	float duration, inches, cm;
	ros::init(argc, argv, "sonar");
	ros::NodeHandle nsonar;
	ros::Publisher pub = nsonar.advertise<um_acc::Sensor>("sensor", 1000);

	um_acc::Sensor sen;

	gpio_export(PING_GPIO);

	while(1)
	{
		gpio_set_dir(PING_GPIO, OUTPUT_PIN);
		gpio_set_value(PING_GPIO, LOW);
		usleep(2);
		gpio_set_value(PING_GPIO, HIGH);
		usleep(5);
		gpio_set_value(PING_GPIO, LOW);

		gpio_set_dir(PING_GPIO, INPUT_PIN);
 
		duration = gpio_pulse_in(PING_GPIO, HIGH);

		cm = microsecondsToCentimeters(duration);

		sen.distance = cm;

		//do something to get rpm from Elmo

		sen.rpm = 5;

		pub.publish(sen);

		usleep(100000);
	}
}
