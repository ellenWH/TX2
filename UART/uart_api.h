/* uart_api.h */
#ifndef		UART_API_H
#define		UART_API_H

//#define		GNR_COM			0
//#define		USB_COM			1
//#define 	COM_TYPE		GNR_COM
//#define 	MAX_COM_NUM		3
//#define		HOST_COM_PORT		1
//#define		TARGET_COM_PORT		2
#define 	BUFFER_SIZE		16
//#define		TIME_DELAY		180
//#define		SEL_FILE_NUM		10
//#define		RECV_FILE_NAME		"recv.dat"
#define PORT 1
#define FALSE -1  
#define TRUE 0
#define LEN 14

extern	int open_port(int com_port);
extern	int set_com_config(int fd,int baud_rate, int data_bits, char parity, int stop_bits);
extern	void my_fun(char * p);
extern	int Max(int *fdt,int size);
void set_speed(int fd, int speed);
int set_Parity(int fd,int databits,int stopbits,int parity);  


typedef struct
{
	short status;
	short axis_x;
	short axis_y;
	short width;
	short height;

} frame;

frame Frame_FDSP;
//

#endif /* UART_API_H */
