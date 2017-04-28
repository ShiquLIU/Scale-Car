#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <um_acc/Pwm.h>
#include <um_acc/Sensor.h>
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include <pthread.h>
 
using namespace std;
 
unsigned int PING_GPIO = 161;
unsigned int SW_GPIO = 164;
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
 
        unsigned int signal;
        gpio_export(SW_GPIO);
        gpio_set_dir(SW_GPIO, INPUT_PIN);
        gpio_get_value(SW_GPIO, &signal);
        cout << signal << endl;
 
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
};
 
int main(int argc, char *argv[]){
 
    ros::init(argc, argv, "listener");
    pthread_t threads[2];
    pthread_create(&threads[0], NULL, Sonar, NULL);
    pthread_create(&threads[1], NULL, Car, NULL);
 
    while(1)
    {
    pwm_export(STEER_PIN);
    pwm_set_period(STEER_PIN, 5000000);
    pwm_set_duty(STEER_PIN, 1600000);
    pwm_enable(STEER_PIN);
 
    pwm_export(DRIVE_PIN);
    pwm_set_period(DRIVE_PIN, 5000000);
    pwm_set_duty(DRIVE_PIN, 1600000);
    pwm_enable(DRIVE_PIN);
    };
 
}
