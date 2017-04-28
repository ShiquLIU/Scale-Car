#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <um_acc/Pwm.h>
#include <algorithm>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

#define VEL_STEP 10
#define STEER_STEP 5

class TeleopCar
{
public:
  TeleopCar();
  void keyLoop();

private:
 
  ros::NodeHandle nh_;
  uint16_t steer_;
  uint16_t vel_;
  ros::Publisher pwm_pub_;
  
};

TeleopCar::TeleopCar(): steer_(1500), vel_(1500)
{
  pwm_pub_ = nh_.advertise<um_acc::Pwm>("drive_control", 1);
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


  for(;;) {

      if(read(kfd, &c, 1) < 0) {
	  perror("read():");
	  exit(-1);
      }

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
	  vel_ = std::min(1700, vel_+VEL_STEP);
	  dirty = true;
	  break;
	case KEYCODE_D:
	  ROS_DEBUG("DOWN");
	  vel_ = std::max(1300,vel_-VEL_STEP);
	  dirty = true;
	  break;
      }

      um_acc::Pwm msg;
      if (dirty) {
	msg.vel_pwm = vel_;
	msg.steer_pwm = steer_;
	pwm_pub_.publish(msg);
      }

    }

  return;
}
