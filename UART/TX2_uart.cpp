int open_port(int com_port)
{
	int fd;
#if (COM_TYPE == GNR_COM)
	char *dev[] = {"/dev/ttyPS1", "/dev/ttyPS0", "/dev/ttyS2"};
#else
	char *dev[] = {"/dev/ttyPS1", "/dev/ttyPS0", "/dev/ttyUSB2"};
#endif

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


int communication(int fd, char *buff)
{
	int i;
	int CRC = 0;
	char data[LEN];
	if ((buff[0] == 0x4B) && (buff[1] == 0x5A)){
		for (i=0;i<LEN-5;i++)
		{
			data[i] = buff[i + 2];
			CRC += data[i];
		}
		if (CRC != buff[LEN-2])
		{
			//return 1;
		}
		
		Frame_FDSP.status = data[0];
	
		Frame_FDSP.axis_x = data[2];
		Frame_FDSP.axis_x = Frame_FDSP.axis_x << 8 | data[1];
		
		Frame_FDSP.axis_y = data[4];
		Frame_FDSP.axis_y = Frame_FDSP.axis_y << 8 | data[3];

		Frame_FDSP.width = data[6];
		Frame_FDSP.width = Frame_FDSP.width << 8 | data[5];
		
		Frame_FDSP.height = data[8];
		Frame_FDSP.height = Frame_FDSP.height << 8 | data[7];
		Frame_FDSP.height/=1.5;
		printf("status: %d axis_x: %d axis_y: %d width: %d height: %d \n", Frame_FDSP.status, Frame_FDSP.axis_x, Frame_FDSP.axis_y, Frame_FDSP.width, Frame_FDSP.height);
		
	}
	else
	{
		return 1;
	}
	return 0;
}



void *_DspThreadProc(void *arg)
{
	int fd, nByte;
	char buff[BUFFER_SIZE];

	if((fd = open_port(1)) < 0){
		perror("open_port");
		return 1;
	}
	
	gDspfd = fd;
	if(set_com_config(fd, 115200, 8, 'N', 1) < 0){
		perror("set_com_config");
		return 1;
	}
	printf("DSP PORT OPEN!\n");

    while(1){
		memset(buff, 0, BUFFER_SIZE);
		nByte = read(fd, buff, BUFFER_SIZE);
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
    printf("\r\n--- Entering main() --- \n\r");

	// DSP串口处理子线程
	DspSerialThread();
    while(1){}

    return 0;
}
