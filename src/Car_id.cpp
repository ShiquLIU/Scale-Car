#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <um_acc/Pwm.h>
#include <um_acc/Sensor.h>
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include <pthread.h>
#include "rs232.h"

using namespace std;

int i=0,
    cport_nr=0,		/* /dev/ttyS0 (COM1 on windows) */
    bdrate=115200;	/* 115200 baud */

char mode[]={'8','N','1',0};


unsigned int PING_GPIO = 161;
unsigned int SW_GPIO = 164;
unsigned int STEER_GPIO = 163;
unsigned int STEER_PIN = 2;
unsigned int DRIVE_PIN = 0;

float microsecondsToCentimeters(float microseconds)
{
    return microseconds / 29 / 2;
};
 
void *Sonar(void *threadid)
{
    float duration, inches, cm;
    ros::NodeHandle nsonar;
    ros::Publisher pub = nsonar.advertise<um_acc::Sensor>("sensor", 1000);
 
    um_acc::Sensor sen;
 
    while(1)
    {
        gpio_export(PING_GPIO);
        gpio_set_dir(PING_GPIO, OUTPUT_PIN);
        gpio_set_value(PING_GPIO, LOW);
        usleep(2);
        gpio_set_value(PING_GPIO, HIGH);
        usleep(5);
        gpio_set_value(PING_GPIO, LOW);
 
        gpio_set_dir(PING_GPIO, INPUT_PIN);
  
        duration = gpio_pulse_in(PING_GPIO, HIGH);
 
        cm = microsecondsToCentimeters(duration);
cout << cm << endl;
        sen.distance = cm;
 
        //do something to get rpm from Elmo
 
        sen.rpm = 5;
 
        pub.publish(sen);
 
        usleep(100000);
    }
};
 
void pwmCallback(const um_acc::Pwm::ConstPtr& msg)
{
    pwm_set_duty(STEER_PIN, msg->steer_pwm*1000);
    pwm_set_duty(DRIVE_PIN, msg->vel_pwm*1000);
};
 
void *Car(void *threadid)
{

	gpio_export(STEER_GPIO);
	gpio_set_dir(STEER_GPIO, OUTPUT_PIN);
	
	while(1)
	{		
		gpio_set_value(STEER_GPIO, LOW);
		usleep(5);
		gpio_set_value(STEER_GPIO, HIGH);
		usleep(3);
	}
	ros::spin();

/*
    pwm_export(STEER_PIN);
    pwm_set_period(STEER_PIN, 5000000);
    pwm_set_duty(STEER_PIN, 1600000);
    pwm_enable(STEER_PIN);
 
    pwm_export(DRIVE_PIN);
    pwm_set_period(DRIVE_PIN, 5000000);
    pwm_set_duty(DRIVE_PIN, 1600000);
    pwm_enable(DRIVE_PIN);
 
    ros::NodeHandle ncar;
    ros::Subscriber sub = ncar.subscribe("pwm_control", 1000, pwmCallback);
 
    ros::spin();
*/
};

void set_current(int tc)
{

	if(tc >= 0)
	{
		unsigned char TC[7];
		TC[0] = 'T';
		TC[1] = 'C';
		TC[2] = '=';
		TC[3] = tc/10 + 0x30;
		TC[4] = '.';
		TC[5] = tc%10 + 0x30;
		TC[6] = 0x0d;
cout << TC << endl;
		RS232_SendBuf(cport_nr, TC, 7);
	}
	else
	{
		tc = -tc;
		unsigned char TC[8];
		TC[0] = 'T';
		TC[1] = 'C';
		TC[2] = '=';
		TC[3] = '-';
		TC[4] = tc/10 + 0x30;
		TC[5] = '.';
		TC[6] = tc%10 + 0x30;
		TC[7] = 0x0d;
cout << TC << endl;
		RS232_SendBuf(cport_nr, TC, 8);
	}

}


int main(int argc, char *argv[]){
 
    ros::init(argc, argv, "listener");
    pthread_t threads[2];
    pthread_create(&threads[0], NULL, Sonar, NULL);
    pthread_create(&threads[1], NULL, Car, NULL);

	unsigned char SW[3];
	SW[0] = 'M';
	SW[1] = 'O';
	SW[2] = 0x0d;
	
	unsigned char MO[5];
	MO[0] = 'M';
	MO[1] = 'O';
	MO[2] = '=';
	MO[3] = '1';
	MO[4] = 0x0d;

	if(RS232_OpenComport(cport_nr, bdrate, mode))
	{
		printf("Can not open comport\n");

		return(0);
	}
	else
	{
		printf("comport open\n");
	}

	RS232_SendBuf(cport_nr, MO, 5);


	while(1)
	{
		unsigned int signal;
		gpio_export(SW_GPIO);
		gpio_set_dir(SW_GPIO, INPUT_PIN);
		gpio_get_value(SW_GPIO, &signal);
		cout << signal << endl;

/*
		// part 1

		if(signal == 1)
		{
			set_current(10);
			usleep(1000000);
		}
		else
		{
			set_current(0);
			usleep(1000000);
		}


		// part 2
		if(signal == 1)
		{
			set_current(10);
			usleep(5000000);
			set_current(0);
			usleep(5000000);
		}
		else
		{
			set_current(0);
		}
*/


		for(int i; i < 5; i++)
		{
			set_current(10);
			usleep(2000000);
			set_current(-10);
			usleep(2000000);
		}
		set_current(0);
		return(0);

	}
 
}
