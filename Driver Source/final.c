#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <iostream>
#include <linux/fb.h>

//for BLE
#include <termios.h>
#define __ARM__

//for pthread
#include <pthread.h>

//add opencv header
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#define FPGA_STEP_MOTOR_DEVICE "/dev/fpga_step_motor"
#define FPGA_SWITCH_DEVICE "/dev/fpga_push_switch"
#define FPGA_SONIC_DEVICE "/dev/sonic_name"
#define FPGA_BUZZER_DEVICE "/dev/fpga_buzzer"
#define FPGA_TEXT_LCD_DEVICE "/dev/fpga_text_lcd"
#define FPGA_FRAME_BUFFER_DEVICE "/dev/fb0"

#define MOTOR_ATTRIBUTE_ERROR_RANGE 4
#define MAX_BUTTON 9
#define MAX_BUFF 32
#define LINE_BUFF 16

using namespace cv;
using namespace std;
//for BLE
static int _blthandle;
struct termios oldtio, newtio;
//bluetooth variables
unsigned char recvdata[10];
unsigned char btString[10];

//for pthread data lock
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

//call function
int Capture(void);
int BmpFile_To_LCD(void);
double receive_time(void);
void user_signal1(int sig);
void set_motor_action(int num1, int num2, int num3);
int blt_Init();

//thread function
void* Camstreaming(void* Para);
void* Sonic(void* Para);
void* Blue_t(void* Para);

//for framebuffer 
typedef unsigned int U32;
typedef unsigned char U16;
typedef struct _user_fb {
	int xres;
	int yres;
	int bpps;
}user_fb;
unsigned int makepixel(U32 r, U32 g, U32 b) {
	return (U32)((b<<16)|(g<<8)|(r));
}

//for WebCam
CvCapture* capture;
Mat Cap_frame, Cap_frame2, Stream_frame, resize_frame;
char CaptureFileName[100];
struct fb_var_screeninfo fvs;
user_fb my_fb = { 1024, 640, 32 };
int check;
U32 Cap_pixel, Stream_pixel;
unsigned int* Cap_pfbdata; 
unsigned int* Stream_pfbdata;
int Cap_offset, Stream_offset;
unsigned int r,g,b,a, s_r, s_g, s_b, s_a;
int y=0, x=0, s_y=0, s_x=0;
//0 : auto 1 : Manuel
int Mode = 0;
int motor_left = 0;
int motor_right = 0;
int clear_pixel;
int posx1, posx2, posy1, posy2;
int repx, repy;

//ctrl + c -> quit
unsigned char quit = 0;

//motor direction flag
int cycle_state = 0;

//device
int dev_motor, dev_switch, dev_sonic, dev_buzzer, dev_lcd, dev_fb, fb_v4l;

//sonic-related variables
int sonic_data = 0;

int main(void)
{
	int BLE;

	BLE = blt_Init();
	printf("Success BLE! %d\n",BLE);

	int thr_stream, thr_sonic, thr_ble;
	pthread_t p_thread[6];
	void *status = NULL;
	
	capture = cvCaptureFromCAM(-1);
	if(capture)
		printf("Success to open webcam\n");
	memset(CaptureFileName, 0x00, sizeof(CaptureFileName));

	//time variable
	double current_time = 0;
	double last_time = 0;
	double detect_time = 0;
	double term_time = 0;

	double buzzer_start_time = 0;
	double buzzer_end_time = 0;

	double push_time_left = 0;
	double push_time_right = 0;
	double pull_time_left = 0;
	double pull_time_right = 0;

	int time_flag = 0;

	//switch-related varialbe
	unsigned char push_sw_buff[MAX_BUTTON];
	int buff_size;

	//buzzer-related variables
	unsigned char buzzer_retval;
	unsigned char buzzer_data;
	bool buzzer_flag = true;

	//LCD-related variables
	unsigned char string[32];
	int str_size;

	(void)signal(SIGINT, user_signal1);
	buff_size=sizeof(push_sw_buff);
	printf("Press <ctrl+c> to quit. \n");
	
	dev_switch = open(FPGA_SWITCH_DEVICE, O_RDWR);
	if(dev_switch < 0) {
		printf("Device Open Error\n");
		exit(1);
	}

	dev_lcd = open(FPGA_TEXT_LCD_DEVICE, O_WRONLY);
	if(dev_lcd < 0) {
		printf("Device Open Error : %s\n", FPGA_TEXT_LCD_DEVICE);
		exit(1);
	}

	dev_fb = open(FPGA_FRAME_BUFFER_DEVICE, O_RDWR);
	if(fb_v4l < 0) {
		printf("Unable to open %s\n", FPGA_FRAME_BUFFER_DEVICE);
		exit(1);
	} else
		printf("Success to open %s\n", FPGA_FRAME_BUFFER_DEVICE);


	//create thread
	thr_stream = pthread_create(&p_thread[0], NULL, Camstreaming, NULL);
	thr_sonic = pthread_create(&p_thread[1], NULL, Sonic, NULL);
	thr_ble = pthread_create(&p_thread[2], NULL, Blue_t, NULL);
	sleep(1);
	
	while(!quit) {
		
		memset(string,0,sizeof(string));
		unsigned char distance[4];
		sprintf((char *)distance, "%d", sonic_data);

		current_time = receive_time();
		buzzer_start_time = receive_time();
		//manual
		if(sonic_data <= 15 | Mode == 1){
			Mode = 1;	//streaming start
			if(time_flag == 0){
				detect_time = current_time;
				printf("------------------detect_time : %f\n",detect_time);
				term_time   = detect_time - last_time;
				printf("------------------term_time : %f\n",term_time);
				time_flag = 1;
				Capture();	
				BmpFile_To_LCD();
					
				dev_buzzer = open(FPGA_BUZZER_DEVICE, O_RDWR);
				if(dev_buzzer < 0) {
					printf("Device Open Error : %s\n",FPGA_BUZZER_DEVICE);
					exit(1);
				}

				if(buzzer_flag) {
					buzzer_end_time = buzzer_start_time + 5;
					buzzer_flag = false; 
					buzzer_data = 1;
					buzzer_retval = write(dev_buzzer, &buzzer_data, 1);
				}
			}
			if(buzzer_end_time - buzzer_start_time < 0) {
				buzzer_data = 0;
				buzzer_retval = write(dev_buzzer, &buzzer_data, 1);
				buzzer_flag = true;
			}
			set_motor_action(0,cycle_state, 0);

			strncat((char *)string, "Manual mode", 11);
			memset(string+11, ' ', LINE_BUFF-11);
			strncat((char *)string, "dist: ", 6);
			str_size = strlen((char *)distance);
			strncat((char *)string, (char *)distance, str_size);
			memset(string+LINE_BUFF+6+str_size, ' ', LINE_BUFF-(6+str_size));
			write(dev_lcd, string, MAX_BUFF);


			read(dev_switch, &push_sw_buff, buff_size);

			if(push_sw_buff[6] == 1) {
				Mode = 0;
				set_motor_action(1, cycle_state, 200);
			}
			else if(push_sw_buff[0] == 1 | motor_left == 1) {
				push_time_left = receive_time();
				set_motor_action(1,1,200);
				if(cycle_state == 0) {
					while(1) {
						read(dev_switch, &push_sw_buff, buff_size);
						current_time = receive_time();
						if(push_sw_buff[0] == 0 & motor_left == 0) {
							pull_time_left = receive_time();
							term_time = term_time - (pull_time_left - push_time_left);
							set_motor_action(0,0,0);
							break;
						}
						if(current_time - push_time_left >= term_time) {
							term_time = 0;
							set_motor_action(0,0,0);
							motor_left = 0;
							break;
						}
					}
				}
				else {
					while(1) {
						read(dev_switch, &push_sw_buff, buff_size);			
						current_time = receive_time();
						if(push_sw_buff[0] == 0 & motor_left == 0) {
							pull_time_left = receive_time();
							term_time = term_time + (pull_time_left - push_time_left);
							set_motor_action(0,0,0);
							break;
						}
						if(current_time - push_time_left>= 6.85000000 - term_time){
							term_time = 6.85000000;
							set_motor_action(0,0,0);
							motor_left = 0;
							break;
						}
					}
				}
			}
			else if(push_sw_buff[2] == 1 | motor_right == 1) {
				push_time_right = receive_time();
				set_motor_action(1,0,200);
				if(cycle_state == 0) {
					while(1) {
						read(dev_switch, &push_sw_buff, buff_size);	
						current_time = receive_time();
						if(push_sw_buff[2] == 0 & motor_right == 0) {
							pull_time_right = receive_time();
							term_time = term_time + (pull_time_right - push_time_right);
							set_motor_action(0,0,0);
							break;
						}
						if(current_time - push_time_right >= 6.850000 - term_time) {
							term_time = 6.850000;
							set_motor_action(0,0,0);
							motor_right = 0;
							break;
						}
					}
				}
				else {
					while(1) {
						read(dev_switch, &push_sw_buff, buff_size);	
						current_time = receive_time();
						if(push_sw_buff[2] == 0 & motor_right == 0) {
							pull_time_right = receive_time();
							term_time = term_time - (pull_time_right - push_time_right);
							set_motor_action(0,0,0);
							break;
						}
						if(current_time - push_time_right >= term_time) {
							term_time = 0;
							set_motor_action(0,0,0);
							motor_right = 0;
							break;
						}
					}
				}
			}
		}
		//auto
		else{
			strncat((char *)string, "Auto Mode", 9);
			memset(string+9, ' ', LINE_BUFF-9);
			strncat((char *)string, "dist: ", 6);
			str_size = strlen((char *)distance);
			strncat((char *)string,(char *)distance, str_size);
			memset(string+LINE_BUFF+6+str_size, ' ', LINE_BUFF-(6+str_size));
			write(dev_lcd, string, MAX_BUFF);
			set_motor_action(1, cycle_state, 200);

			if(time_flag == 1){
				last_time = current_time - term_time;
				printf("------------------current_time : %f\n", current_time);
				printf("------------------last_time: %f\n", last_time);
				term_time = 0;
				time_flag = 0;

				buzzer_data = 0;
				buzzer_retval = write(dev_buzzer, &buzzer_data, 1);
				buzzer_flag = true;
				close(dev_buzzer);
			}
						
			if(current_time - last_time >= 6.850000) {
				set_motor_action(0,0,0);
				cycle_state = (cycle_state != 0) ? 0 : 1;
				set_motor_action(1, cycle_state, 200);
				printf("-----------------current : %lf\n", current_time); 
				printf("-----------------last    : %lf\n", last_time);
				printf("%lf\n", current_time - last_time);
				last_time = current_time;
				printf("cycle_state : %d\n", cycle_state);
			}
		}
	}
	tcsetattr( _blthandle, TCSANOW, &oldtio);
	close(_blthandle);
	close(dev_switch);
	close(dev_lcd);
	close(dev_fb);
	printf("exit\n");
	thr_stream = pthread_join(p_thread[0], &status);
	thr_sonic = pthread_join(p_thread[1], &status);
	thr_ble = pthread_join(p_thread[2], &status);
}

int blt_Init(void) 
{
#ifdef __ARM__
	_blthandle = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY );
#else
	_blthandle = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY );
#endif
	if(_blthandle < 0) {
		printf("Serial Port Open Fail\n");
		return -1;
	}

	tcgetattr(_blthandle, &oldtio);
	memset(&newtio, 1, sizeof(newtio));
	newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR | ICRNL;
	newtio.c_oflag = OPOST | ONLCR;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(_blthandle, TCIFLUSH );
	tcsetattr(_blthandle, TCSANOW, &newtio);
	return 1;

}


int BmpFile_To_LCD(void)
{
	printf("file name : %s\n", CaptureFileName);

	clear_pixel = makepixel(0,0,0);
	posx1 = 20;
	posx2 = 340;
	posy1 = 20;
	posy2 = 260;

	for(repy=posy1; repy < posy2; repy++) {
		Cap_offset = repy * fvs.xres * (32/8) + posx1 * (32/8);
		if(lseek(dev_fb, Cap_offset, SEEK_SET) < 0) {
			perror("LSeek Error !");
			exit(1);
		}
		for(repx = posx1; repx <= posx2; repx++)
			write(dev_fb, &clear_pixel, (32/8));
	}

	Cap_pfbdata = (unsigned int *) mmap(0, fvs.xres*fvs.yres*(32/8), 
			PROT_READ | PROT_WRITE, MAP_SHARED, dev_fb, 0);
	if((unsigned)Cap_pfbdata == (unsigned)-1) {
		perror("Error Mapping!\n");
	}
	Cap_frame2 = imread(CaptureFileName, CV_LOAD_IMAGE_UNCHANGED);

	if(Cap_frame2.empty())
		return -1;

	resize(Cap_frame2, resize_frame, Size(Cap_frame2.cols/2,Cap_frame2.rows/2),0,0,CV_INTER_NN);

	for(y=0; y<resize_frame.rows; y++){
		Cap_offset = (y+20) * fvs.xres + 20;
		for(x=0; x<resize_frame.cols; x++) {
			r = resize_frame.at<Vec3b>(y,x)[0];
			g = resize_frame.at<Vec3b>(y,x)[1];
			b = resize_frame.at<Vec3b>(y,x)[2];
			Cap_pixel = makepixel(r,g,b);
			*(Cap_pfbdata+Cap_offset+x) = Cap_pixel;
		}
	}
	munmap(Cap_pfbdata, fvs.xres*fvs.yres*(24/8));
	
	return 0;
}

int Capture(void)
{
	Cap_frame = cvQueryFrame(capture);

	if(Cap_frame.empty()) {
		printf("webcam data frame error\n");
	}
	int temp_hour, temp_min, temp_sec, temp_time;
	temp_time = receive_time();
	temp_hour = temp_time / 3600;
	temp_min = temp_time % 3600 / 60;
	temp_sec = temp_time % 3600 % 60;
	sprintf(CaptureFileName, "/root/nfs/LCD_OPENCV/Capture/%02dh-%02dm-%02ds.bmp",temp_hour, temp_min, temp_sec);

	imwrite(CaptureFileName, Cap_frame);
	printf("Capture Success!\n");

	return 0;
}

//get the current time
double receive_time()
{
	double hour,min,sec;
	double usec;
	double return_time;
	struct timeval val;
	struct tm *ptm;

	gettimeofday(&val, NULL);
	ptm = localtime(&val.tv_sec);

	hour = ptm->tm_hour * 3600;
	min = ptm->tm_min * 60;
	sec = ptm->tm_sec;
	usec = val.tv_usec * 0.000001;

	return_time = hour + min + sec + usec;

	return return_time;
}

//set motor option
void set_motor_action(int num1, int num2, int num3)
{
	//motor-related variables
	int motor_action = 0;
	int motor_direction = 0;
	int motor_speed = 0;
	unsigned char motor_state[3];
	memset(motor_state,0,sizeof(motor_state));

	motor_action = num1;
	motor_direction = num2;
	motor_speed = num3;

	motor_state[0]=(unsigned char)motor_action;
	motor_state[1]=(unsigned char)motor_direction;
	motor_state[2]=(unsigned char)motor_speed;
	
	dev_motor = open(FPGA_STEP_MOTOR_DEVICE, O_WRONLY);
	if(dev_motor < 0) {
		printf("Device open error : %s\n", FPGA_STEP_MOTOR_DEVICE);
		exit(1);
	}
	write(dev_motor, motor_state, 3);
	close(dev_motor);
}


void user_signal1(int sig)
{
	quit = 1;
}

//streaming thread
void* Camstreaming(void* Para)
{	
	if((check=ioctl(dev_fb, FBIOGET_VSCREENINFO, &fvs)) < 0) {
		perror("Get Information Error - VSCREENINFO !");
		exit(1);
	}
	fvs.xres = my_fb.xres;
	fvs.yres = my_fb.yres;
	fvs.bits_per_pixel = my_fb.bpps;

	if(check=ioctl(dev_fb, FBIOPUT_VSCREENINFO, &fvs)) {
		perror("PUT Information Error - VSCREENINFO !");
		exit(1);
	}
							
	if(fvs.bits_per_pixel != 32) {
		perror("Unsupport Mode. 32bpp only.");
		exit(1);
	}

	if(lseek(dev_fb, 0, SEEK_SET) < 0 ){
		perror("LSeek Error.");
		exit(1);
	}

	Stream_pfbdata = (unsigned int*)mmap(0, fvs.xres*fvs.yres*(32/8), 
			PROT_READ | PROT_WRITE, MAP_SHARED, dev_fb, 0);
	if((unsigned)Stream_pfbdata == (unsigned)-1) {
		perror("Error Mapping!\n");
	}

	while(1) {
		if(Mode == 1){
			Stream_frame = cvQueryFrame(capture);
			if(Stream_frame.empty()) {
				printf("webcam data frame error\n");
			}
	
			for(s_y=0; s_y < Stream_frame.rows; s_y++){
				Stream_offset = (s_y+20) * fvs.xres + 360;
				for(s_x=0; s_x < Stream_frame.cols; s_x++) {
					s_r = Stream_frame.at<Vec3b>(s_y,s_x)[0];
					s_g = Stream_frame.at<Vec3b>(s_y,s_x)[1];
					s_b = Stream_frame.at<Vec3b>(s_y,s_x)[2];
					Stream_pixel = makepixel(s_r,s_g,s_b);
					*(Stream_pfbdata + Stream_offset + s_x) = Stream_pixel;
				}
			}
			if(quit == 1)
				break;
		}
		else{
			clear_pixel = makepixel(0,0,0);
			posx1 = 0;
			posx2 = 1024;
			posy1 = 0;
			posy2 = 640;

			for(repy=posy1; repy < posy2; repy++) {
				Cap_offset = repy * fvs.xres * (32/8) + posx1 * (32/8);
				if(lseek(dev_fb, Cap_offset, SEEK_SET) < 0) {
					perror("LSeek Error !");
					exit(1);
				}
				for(repx = posx1; repx <= posx2; repx++)
					write(dev_fb, &clear_pixel, (32/8));
				}
			if(quit == 1)
				break;
		}
	}
	munmap(Stream_pfbdata, fvs.xres*fvs.yres*(24/8));
}

//sonic thread
void* Sonic(void* Para)
{
	dev_sonic = open(FPGA_SONIC_DEVICE, O_RDWR);
	if(dev_sonic < 0) {
		printf("Device Open Error : %s\n",FPGA_SONIC_DEVICE);
		exit(1);
	}
	while(1){	
		read(dev_sonic, &sonic_data, 1);
		if(quit == 1) {
			break;
		}
	}
	close(dev_sonic);
}

void* Blue_t(void* Para)
{
	while(1) {
		memset(btString, 0x00, sizeof(btString));
		memset(recvdata, 0x00, sizeof(recvdata));

		while(read(_blthandle, recvdata, sizeof(recvdata))) {
			strcat((char*)btString,(char*)recvdata);
			usleep(10000);
		}

		if(!strcmp((char*)btString, "auto")) {
			pthread_mutex_lock(&mutex);
			Mode = 0;
			printf("btString : %s\n", btString);
			pthread_mutex_unlock(&mutex);
		} else if(!strcmp((char*)btString, "manu")) {
			pthread_mutex_lock(&mutex);
			Mode = 1;
			printf("btString : %s\n", btString);
			pthread_mutex_unlock(&mutex);
		} else if(!strcmp((char*)btString, "left")) {
			pthread_mutex_lock(&mutex);
			motor_left = 1;
			printf("btString : %s\n", btString);
			pthread_mutex_unlock(&mutex);
		} else if(!strcmp((char*)btString, "right")) {
			pthread_mutex_lock(&mutex);
			motor_right = 1;
			printf("btString : %s\n", btString);
			pthread_mutex_unlock(&mutex);
		}
		if(quit == 1) 
			break;
	}
}

