#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/unistd.h>
#include <sys/time.h>
#include <getopt.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stdint.h>
#include "globalvar.h"
#include "main.h"

#define true 1
#define false 0

/* == define the task here!!
   This file must define the "control()" function call
*/

#include "radio_comm.h"
//#include "ltv_perching_task.h"
//#include "elevator_b2t.h"
//#include "elevator_t2b.h"

/*private functions*/
int open_radio_port();

lcm_t * lcm;



// ==== The actual control task thread ====
int radio_comm()
{

  	float elapsed;
    double timestamp;
	static struct timeval td_start,td_end, timestamp_st;
        double servo;
        static int init = true;

  /*write array buffer*/
  unsigned char ch[5];

  /*staus variable for device port pins*/
  int status=0;

  /*RS232 serial port file descriptor*/
  int rs232;
  
  /*time value structure*/
  struct timeval curr_time;
   
  /*open and configure serial port*/
  rs232=open_radio_port();
  
  if (rs232 == -1)
      
  {    
      /*Port could not be opened.*/
      
      perror("open_port: Unable to open " MODEMDEVICE);
      
  }
  
    ch[0] = 150;
    ch[1] = 150;
    ch[2] = 150;
    ch[3] = 150;
  write(rs232, ch, 4);

   int check=0;

  //usleep(500000);
  
  // ENTER THE CONTROL LOOP HERE
  while(1){
  	  
      /*sleep until CTS pin changes*/

	if(check==0)

	{
		printf("waiting for connection...\n");	
	}
	
      ioctl(rs232, TIOCMIWAIT , &status);  

	if(check==0)

	{
		printf("connection complete!\n");	
		check=1;
	}
		
	if(init)
	{

	gettimeofday(&td_start,NULL);

	init = false;
	}
       
	gettimeofday(&td_end,NULL);

       	elapsed = 1000000.0 * (td_end.tv_sec -td_start.tv_sec);
       	elapsed += (td_end.tv_usec - td_start.tv_usec);

       	elapsed=elapsed/1000000.0;

	lock();
	
	ch[0]=(uint8_t)lcm_hotrod_u.aileron;
	ch[1]=(uint8_t)lcm_hotrod_u.elevator;
	ch[2]=(uint8_t)lcm_hotrod_u.throttle;
	ch[3]=(uint8_t)lcm_hotrod_u.rudder;

	unlock();

	//printf("%d\n",ch);

      //ch=((256.0/2.0)*sin(2*3.14159*(.5)*elapsed)+(256.0/2.0));

	//printf("%d\n",ch);


      /*log the control sent time*/
     // gettimeofday(&curr_time,NULL);
      
      /*write command to servo (within 1.25 ms for PCbuddy*/
      write(rs232, ch, 4);
      gettimeofday(&timestamp_st,NULL);
      
      


      timestamp = (double)timestamp_st.tv_sec + (double)timestamp_st.tv_usec/(double)1000000.0;
      
      //printf("t=%f\n", timestamp);
      /*
      lcm_publish_u_actual(timestamp, ch[0], ch[1], ch[2], ch[3],
        lcm_hotrod_u.kp_height, lcm_hotrod_u.kd_height,
        lcm_hotrod_u.kp_yaw, lcm_hotrod_u.kd_yaw,
        lcm_hotrod_u.kp_pitch, lcm_hotrod_u.kd_pitch,
        lcm_hotrod_u.kp_roll, lcm_hotrod_u.kd_roll );
	    */
	  
	  lcmt_hotrod_u msg;
	  
	  //msg.timestamp= timestamp;
	  
      msg.timestamp = (timestamp_st.tv_sec * 1000.0) + (float)timestamp_st.tv_usec/1000.0 + 0.5;
	  
      msg.aileron = ch[0];
	  msg.elevator = ch[1];
      msg.throttle = ch[2];
      msg.rudder = ch[3];

      msg.kp_height = lcm_hotrod_u.kp_height;
      msg.kd_height = lcm_hotrod_u.kd_height;

      msg.kp_yaw = lcm_hotrod_u.kp_yaw;
      msg.kd_yaw = lcm_hotrod_u.kd_yaw;

      msg.kp_pitch = lcm_hotrod_u.kp_pitch;
      msg.kd_pitch = lcm_hotrod_u.kd_pitch;

      msg.kp_roll = lcm_hotrod_u.kp_roll;
      msg.kd_roll = lcm_hotrod_u.kd_roll;
      
      msg.kp_x = lcm_hotrod_u.kp_x;
      msg.kd_x = lcm_hotrod_u.kd_x;
      
      msg.kp_z = lcm_hotrod_u.kp_z;
      msg.kd_z = lcm_hotrod_u.kd_z;
      
      lcmt_hotrod_u_publish (lcm, "hotrod_u_actual", &msg);
	            
	            //printf("%d\n", ch[2]);
      /*sleep for 5ms to keep from over-looping*/
      usleep(5000);
      
      /*flush the input and output buffer*/
      tcflush(rs232, TCIOFLUSH);
      
  }
  
  /*close the file descriptor*/
  close( rs232 );

	return 0;

}



/*==================================================
 * Function Name: open_port
 * Description :  configures and opens rs232 serial port
 * Inputs:	 None
 * Outputs: None
 * =====================================================*/
int open_radio_port() {
    /* local file descriptor for the port */
    int fd;
    
    /*open the serial port
     *
     * flags:  O_RDWR   : read write mode
     * O_NOCTTY : keep program from being the controlling terminal
     * O_NDELAY : don't care about state of DCD signal
     */
    
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    
    /*If the file descriptor returns a -1*/
    if (fd == -1)
        
    {
        
        /*Port could not be opened.*/
        
        perror("open_port: Unable to open " MODEMDEVICE);
        
    }
    
    else
        
        /*restore normal blocking behavior port read command*/
        fcntl(fd, F_SETFL, 0);
    
    /*create the options structure*/
    struct termios options;
    
    /*get the port options*/
    tcgetattr(fd, &options);
    
    /*zero out all options*/
    bzero(&options, sizeof(options));
    
    /*set the control flag options
     *
     * flags:  BAUDRATE   : Port baudrate
     * CS8 	   : Set 8 data bits
     * CLOCAL	   : Local line - do not change "owner" of the port
     * CREAD 	   : Enable receiver
     * CRTSCTS    : Enable flow control
     */
    
    options.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD ;//| CRTSCTS;
    
    /*Set input options flag to ignore parity bit*/
    options.c_iflag = IGNPAR;
    
    /*Do not set an output flags*/
    options.c_oflag = 0;
    
    /*Do not set any local options*/
    options.c_lflag = 0;

	   		options.c_cc[VTIME]    = 0;     /* inter-character timer unused */
        	options.c_cc[VMIN]     = 8;     /* blocking read until 1 character arrives */
    
    /*flush port buffer of input data*/
    tcflush(fd, TCIFLUSH);
    
    /*set all port options*/
    tcsetattr(fd, TCSANOW, &options);
    
    /*return file descriptor*/
    return (fd);
    
    
}
