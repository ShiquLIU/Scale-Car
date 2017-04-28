#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <um_acc/Pwm.h>
#include <um_acc/Sensor.h>
#include <algorithm>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

#define VEL_STEP 10
#define STEER_STEP 10

#define VEL_SAT 1550
#define P_GAIN 75
#define DESIRED_RPM 500
#define CONTROL_OFFSET 1500

class TeleopCar
{
public:
  TeleopCar();
  void keyLoop();

private:
 
  ros::NodeHandle nh_;
  uint16_t steer_;
  uint16_t vel_;
  float rpm_;
  uint16_t control_vel_(float rpm);
  void get_rpm(const um_acc::Sensor::ConstPtr& car);
  ros::Publisher pwm_pub_;
  ros::Subscriber car_sub_;
  
};

void TeleopCar::get_rpm(const um_acc::Sensor::ConstPtr& car)
{
  rpm_ = car->rpm;
}

TeleopCar::TeleopCar(): steer_(1500), vel_(1500)
{
  pwm_pub_ = nh_.advertise<um_acc::Pwm>("pwm_control", 1);
  car_sub_ = nh_.subscribe<um_acc::Sensor>("sensor", 1000, &TeleopCar::get_rpm, this);
}

uint16_t TeleopCar::control_vel_(float rpm)
{
  uint16_t u = P_GAIN * (DESIRED_RPM - rpm) + CONTROL_OFFSET;  //calculate control response

  //saturate PWM
  if (u > VEL_SAT) u = VEL_SAT;
  else if (u < 1500) u = 1500;

  return u;
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_car");
  TeleopCar teleop_car;

  signal(SIGINT,quit);

  teleop_car.keyLoop();
  
  return(0);
}


void TeleopCar::keyLoop()
{
  char c;
  bool dirty=false;

  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the car.");

  ros::Rate loop_rate(10);


  while (ros::ok()) {

      std::cout << "hi" << std::endl;

      /*if(read(kfd, &c, 1) < 0) {
	  perror("read():");
	  exit(-1);
      }*/

      read(kfd, &c, 1);

      uint16_t steer, vel;
      ROS_DEBUG("value: 0x%02X\n", c);
  
      switch(c)	{
	case KEYCODE_L:
	  ROS_DEBUG("LEFT");
	  steer_ = std::max(1100, steer_-STEER_STEP);
	  dirty = true;
	  break;
	case KEYCODE_R:
	  ROS_DEBUG("RIGHT");
	  steer_ = std::min(1900, steer_+STEER_STEP);
	  dirty = true;
	  break;
	case KEYCODE_U:
	  ROS_DEBUG("UP");
	  //vel_ = std::min(1700, vel_+VEL_STEP);
	  //std::cout << "Controlling speed to " << DESIRED_RPM << " rpms." << std::endl;
	  vel_ = control_vel_(rpm_);
	  dirty = true;
	  break;
	case KEYCODE_D:
	  ROS_DEBUG("DOWN");
	  //vel_ = std::max(1300,vel_-VEL_STEP);
	  vel_ = 1500;
	  std::cout << "Stopping vehicle..." << std::endl;
	  dirty = true;
	  break;
      }

      um_acc::Pwm msg;
      if (dirty) {
	msg.vel_pwm = vel_;
	msg.steer_pwm = steer_;
	pwm_pub_.publish(msg);
      }

    //ros::spinOnce();
    //loop_rate.sleep();

    }

  return;
}
