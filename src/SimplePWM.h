#ifndef SIMPLEPWM_H_
#define SIMPLEPWM_H_

 /****************************************************************
 * Constants
 ****************************************************************/

#define SYSFS_PWM_DIR "/sys/class/pwm/pwmchip0"
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define MAX_BUF 64

int pwm_export(unsigned int pwm);
int pwm_unexport(unsigned int pwm);
int pwm_set_period(unsigned int pwm, unsigned int period);
int pwm_set_duty(unsigned int pwm, unsigned int duty);
int pwm_enable(unsigned int pwm);
int pwm_disable(unsigned int pwm);

#endif /* SIMPLEPWM_H_ */
