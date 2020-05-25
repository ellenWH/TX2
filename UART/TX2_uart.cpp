/**
 * UART communication of TX2
 * 
 *
 * Author:  ELLEN
 * Date:    2020-5-20
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <termios.h>
#include <sys/mman.h>
#include <stdbool.h>
#include "uart_api.h"

frame Frame_FDSP;
control Frame_Control;

int gDspfd;

int open_port(int com_port)
{
	int fd;
	/*
#if (COM_TYPE == GNR_COM)
	char *dev[] = {"/dev/ttyPS1", "/dev/ttyPS0", "/dev/ttyS2"};
#else
	char *dev[] = {"/dev/ttyPS1", "/dev/ttyPS0", "/dev/ttyUSB2"};
#endif
*/
	char *dev[] = {"/dev/ttyTHS1", "/dev/ttyTHS1", "/dev/ttyTHS1"};

	if ((com_port < 0) || (com_port > 5))
	{
		return -1;
	}
	
	fd = open(dev[com_port - 1], O_RDWR|O_NOCTTY|O_NDELAY);
	if (fd < 0)
	{
		perror("OPEN SERIAL PORT");
		return -1;
	}
	if (fcntl(fd, F_SETFL, 0) < 0)
	{
		perror("fcntl F_SETFL\n");
	}
	if (isatty(STDIN_FILENO) == 0)
	{
		perror("standard input is not a terminal device");
	}
	return fd;
}


// configuration of uart
int set_com_config(int fd, int baud_rate, int data_bits, char parity, int stop_bits)
{
	struct termios new_cfg, old_cfg;
	int speed;
	
	if (tcgetattr(fd, &old_cfg) != 0)
	{
		perror("tcgetattr");
		return 1;
	}
	
	new_cfg = old_cfg;
	cfmakeraw(&new_cfg);
	new_cfg.c_cflag &= ~CSIZE;
	
	switch(baud_rate)
	{
		case 2400:
			speed = B2400;
		break;
		
		case 4800:
			speed = B4800;
		break;
		
		case 9600:
			speed = B9600;
		break;
		
		case 19200:
			speed = B19200;
		break;
		
		case 38400:
			speed = B38400;
		break;
		
		default:
		case 115200:
			speed = B115200;
		break;
	}
	
	cfsetispeed(&new_cfg, speed);
	cfsetospeed(&new_cfg, speed);
	
	switch(data_bits)
	{
		case 7:
			new_cfg.c_cflag |= CS7;
		break;
		
		default:
		case 8:
			new_cfg.c_cflag |= CS8;
		break;
	}
	
	switch(parity)
	{
		default:
		case 'n':
		case 'N':
		{
			new_cfg.c_cflag &= ~PARENB;
			new_cfg.c_iflag &= ~INPCK;
		}
		break;
		
		case 'o':
		case 'O':
		{
			new_cfg.c_cflag &= (PARODD | PARENB);
			new_cfg.c_iflag &= INPCK;
			//new_cfg.c_iflag &= ~(ICRNL|IGNCR);
			//new_cfg.c_lflag &= ~(ICANON );	
		}
		break;
		
		case 'e':
		case 'E':
		{
			new_cfg.c_cflag |= PARENB;
			new_cfg.c_cflag &= ~PARODD;
			new_cfg.c_iflag |= INPCK;
		}
		break;

		case 's':
		case 'S':
		{
			new_cfg.c_cflag &= ~PARENB;
			new_cfg.c_cflag &= ~CSTOPB;
		}
		break;
	}
	
		switch(stop_bits)
	{
		default:
		case 1:
			new_cfg.c_cflag &= ~CSTOPB;
		break;
		
		case 2:
			new_cfg.c_cflag |= CSTOPB;
		break;
	}
	new_cfg.c_cc[VTIME] = 0;
	new_cfg.c_cc[VMIN] = 1;
	
	tcflush(fd, TCIFLUSH);
	if ((tcsetattr(fd, TCSANOW, &new_cfg)) != 0)
	{
		perror("tcsetattr");
		return -1;
	}
	return 0;
}

/* recieve information and analyse it */
int communication(int fd, char *buff)
{
	int i;
	int CRC = 0;
	char data[LEN];
	if((buff[0] != 0x7E) || (buff[1] != 0x01) || (buff[31] != 0xE7)){
		printf("Head/Tail error");
		return;
	}

	for (i=0;i<LEN-5;i++)
	{
		data[i] = buff[i + 2];
		CRC += data[i];
	}
#ifdef CRC_CHECK
	if (CRC != buff[LEN-2])
	{
		//return 1;
	}
#endif
	switch(buff[31]){
		case 0x03:
			Frame_Control.yaw = (buff[3] << 24) | (buff[4] << 16) | (buff[5] << 8) | buff[6];
			if(buff[16] == 0x03)
				Frame_Control.pitch = (buff[17] << 24) | (buff[18] << 16) | (buff[19] << 8) | buff[20];
			else
				printf("pitch data error!")
			
		break;
		
		case 0x04:
		break;
		
		default:
		break;
	
	}
	Frame_FDSP.status = data[0];

	printf("status: %d axis_x: %d axis_y: %d width: %d height: %d \n", Frame_FDSP.status, Frame_FDSP.axis_x, Frame_FDSP.axis_y, Frame_FDSP.width, Frame_FDSP.height);
	

	return 0;
}

int UartSend(){
	memset(buff_send, 0, LEN);
	
	switch(buff[2]){
		case 0x03:	// positioning
			buff[2] = 0x03;

		break;
			
		case 0x04:	// search
			buff[2] = 0x04;

			
		break;
		
		case 0x05:	// track
			buff[2] = 0x05;


		break;
		
		case 0x06:	// sector scan
			buff[2] = 0x06;

		break;
		
		case 0x07:	// circle
			buff[2] = 0x07;

		break;
		
		case 0x08:	// pre_setting point
			buff[2] = 0x08;
		break;
		
		case 0x09:
			buff[2] = 0x09;
		break;
		
		case 0x0A:
		break;
		
		case 0x0B:
		break;
		
		case 0x0C:
		break;
		
		default:
		break;	
	}
	write(gDspfd, buff_send, LEN);
	
}

void *_DspThreadProc(void *arg)
{
	int fd, nByte;
	char buff[BUFFER_SIZE];

	if((fd = open_port(PORT)) < 0){
		perror("open_port");
		return 1;
	}
	
	gDspfd = fd;
	if(set_com_config(fd, 115200, 8, 'N', 1) < 0){
		perror("set_com_config");
		return 1;
	}
	printf("UART PORT OPEN!\n");

    while(1){
		memset(buff, 0, BUFFER_SIZE);
		nByte = read(fd, buff, BUFFER_SIZE);
		printf("\r\n--- %d bytes recieved --- \n\r", nByte);
		communication(fd, buff);
	
	}
	close(fd);
	return NULL;
}

void DspSerialThread()
{
	pthread_t _thread;
	pthread_create(&_thread, NULL, _DspThreadProc, NULL);
    pthread_detach(_thread);

	return;
}

int main(int argc, char **argv){
	int cnt=0;
    printf("\r\n--- Entering main() --- \n\r");

	// DSP串口处理子线程
	DspSerialThread();
	
    while(1){
		if(cnt == 20){
			UartSend();
			cnt = 0;
		}
		cnt ++;
	}

    return 0;
}
