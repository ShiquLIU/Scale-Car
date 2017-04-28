#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <um_acc/Pwm.h>
#include <um_acc/Sensor.h>
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include "rs232.h"
#include <pthread.h>

using namespace std;

int i=0,
    cport_nr=0,		/* /dev/ttyS0 (COM1 on windows) */
    bdrate=115200;	/* 115200 baud */

char mode[]={'8','N','1',0};

int main(int argc, char *argv[])
{

	unsigned char SW[3];
	SW[0] = 'M';
	SW[1] = 'O';
	SW[2] = 0x0d;

	unsigned char REV[10];
	
	unsigned char MO[5];
	MO[0] = 'M';
	MO[1] = 'O';
	MO[2] = '=';
	MO[3] = '1';
	MO[4] = 0x0d;

	unsigned char TC[5];
	TC[0] = 'T';
	TC[1] = 'C';
	TC[2] = '=';
	TC[3] = '1';
	TC[4] = 0x0d;

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
		for(int n = 0; n < 5; i++)
		{
			RS232_SendBuf(cport_nr, TC, 5);
			usleep(1000000);
		}
		RS232_SendBuf(cport_nr, TC, 0);
		break;

//		RS232_PollComport(cport_nr, REV, 10);
//		cout << REV << endl;
	}
}
