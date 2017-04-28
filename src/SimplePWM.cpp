#include "SimplePWM.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

/****************************************************************
 * pwm_export
 ****************************************************************/
int pwm_export(unsigned int pwm)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_PWM_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("pwmchip0/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", pwm);
	write(fd, buf, len);
	close(fd);

	return 0;
}

/****************************************************************
 * pwm_unexport
 ****************************************************************/
int pwm_unexport(unsigned int pwm)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_PWM_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		perror("pwmchip0/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", pwm);
	write(fd, buf, len);
	close(fd);
	return 0;
}

int pwm_set_period(unsigned int pwm, unsigned int period)
{
	int fd, len;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/period", pwm);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("pwm/period");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", period);
	write(fd, buf, len);

	close(fd);
	return 0;
}

int pwm_set_duty(unsigned int pwm, unsigned int duty)
{
	int fd, len;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/duty_cycle", pwm);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("pwm/duty_cycle");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", duty);
	write(fd, buf, len);

	close(fd);
	return 0;
}

int pwm_enable(unsigned int pwm)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/enable", pwm);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("pwm/enable");
		return fd;
	}

	write(fd, "1", 2);
	close(fd);
	return 0;
}

int pwm_disable(unsigned int pwm)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_PWM_DIR "/pwm%d/enable", pwm);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("pwm/enable");
		return fd;
	}

	write(fd, "0", 2);
	close(fd);
	return 0;
}
