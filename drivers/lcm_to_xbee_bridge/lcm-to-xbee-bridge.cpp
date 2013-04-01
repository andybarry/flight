/*
 * Bridges LCM messages to a MAVLINK protocol and sends over an Xbee to the groundstation.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */

#include <iostream>


using namespace std;


#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../mavlink-rlg/csailrlg/mavlink.h"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include "mav_pose_t.h" // from Fixie

#include "mavconn.h" // from mavconn
    


#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <inttypes.h>
#include <fstream>

#define BAUD_RATE 57600

#define DOWNSAMPLE_AMOUNT 10


lcm_t * lcm;

int origin_init = 0;
BotGPSLinearize gpsLinearize;
double elev_origin;

char *xbeeDevice = NULL;

mav_pose_t_subscription_t *pose_sub;

uint8_t systemID = getSystemID();

uint8_t serialBuffer[MAVLINK_MAX_PACKET_LEN];

int serialPort_fd;

int downsampleCounter = 0;

int open_port(char *port);
bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
void close_port(int fd);


static void usage(void)
{
        fprintf(stderr, "usage: lcm-to-xbee-bridge xbee-device state-estimator-pose-channel-name\n");
        fprintf(stderr, "    xbee-device: Location of the Xbee (often /dev/ttyUSB0)\n");
        fprintf(stderr, "    state-estimator-pose-channel-name: LCM channel to listen for state estimates on\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./lcm-to-xbee-bridge /dev/ttyUSB0 STATE_ESTIMATOR_POSE\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    mav_pose_t_unsubscribe(lcm, pose_sub);
    lcm_destroy (lcm);
    
    close_port(serialPort_fd);

    printf("done.\n");
    
    exit(0);
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}


void pose_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user)
{
    if (downsampleCounter >= DOWNSAMPLE_AMOUNT)
    {
        // translates a local LCM message for servos into a message for the airplane over
        
        downsampleCounter = 0;
        
        // create the mavlink message
        mavlink_message_t mavmsg;
        
        
        mavlink_msg_state_estimator_pose_pack(
            systemID,   
            201,
            &mavmsg,
		    (int32_t) getTimestampNow(),
		    msg->pos[0], msg->pos[1], msg->pos[2],
		    msg->vel[0], msg->vel[1], msg->vel[2],
		    msg->orientation[0], msg->orientation[1], msg->orientation[2], msg->orientation[3],
		    msg->rotation_rate[0], msg->rotation_rate[1], msg->rotation_rate[2],
		    msg->accel[0], msg->accel[1], msg->accel[2]
	    );
        
        
        int messageLength = mavlink_msg_to_send_buffer(serialBuffer, &mavmsg);
        int written = write(serialPort_fd, (char*)serialBuffer, messageLength);
        
        if (written != messageLength)
        {
	        fprintf(stderr, "\nERROR: Unable to send pose message over serial port.\n");
        }
    } else {
        downsampleCounter ++;
    }
    
}


int main(int argc,char** argv)
{
    char *channelPose = NULL;

    if (argc!=3) {
        usage();
        exit(0);
    }

    xbeeDevice = argv[1];
    channelPose = argv[2];
    
    // SETUP SERIAL PORT

	printf("Xbee | LCM bridge started\n");

	// Exit if opening port failed
	// Open the serial port.
	printf("Trying to connect to %s.. ", xbeeDevice);
	serialPort_fd = open_port(xbeeDevice);
	if (serialPort_fd == -1)
	{
		printf("failure, could not open port.\n");
		exit(1);
	}
	else
	{
		printf("success.\n");
	}
	
	printf("Trying to configure %s.. ", xbeeDevice);
	bool setup = setup_port(serialPort_fd, BAUD_RATE, 8, 1, false, false);
	if (!setup)
	{
		printf("failure, could not configure port.\n");
		exit(1);
	}
	else
	{
		printf("success.\n");
	}
	
	

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    pose_sub =  mav_pose_t_subscribe (lcm, channelPose, &pose_handler, NULL);

    signal(SIGINT,sighandler);
    
    printf("Receiving:\n\tPose LCM: %s\nPublishing to Xbee: %s\n", channelPose, xbeeDevice);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}


// --------- code copied from mavconn-bridge-serial.cc ----------------------
/**
*
*
* Returns the file descriptor on success or -1 on error.
*/

int open_port(char *port)
{
	int fd; /* File descriptor for the port */

	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		/* Could not open the port. */
		return(-1);
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}

	return (fd);
}

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	//struct termios options;

	struct termios  config;
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %s is NOT a serial port\n", xbeeDevice);
		return false;
	}
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of port %s\n", xbeeDevice);
		return false;
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	config.c_oflag = 0;
	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	config.c_cflag |= CLOCAL;
	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	//tcgetattr(fd, &options);

	switch (baud)
	{
	case 1200:
		if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;
	case 1800:
		cfsetispeed(&config, B1800);
		cfsetospeed(&config, B1800);
		break;
	case 9600:
		cfsetispeed(&config, B9600);
		cfsetospeed(&config, B9600);
		break;
	case 19200:
		cfsetispeed(&config, B19200);
		cfsetospeed(&config, B19200);
		break;
	case 38400:
		if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;
	case 57600:
		if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;
	case 115200:
		if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;
	case 460800:
		if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;
	case 921600:
		if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
		{
			fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
			return false;
		}
		break;
	default:
		fprintf(stderr, "ERROR: Desired baud rate %d could not be set, falling back to 115200 8N1 default rate.\n", baud);
		cfsetispeed(&config, B115200);
		cfsetospeed(&config, B115200);

		break;
	}

	/*

	//
	// Enable the receiver and set local mode...
	//

	options.c_cflag |= (CLOCAL | CREAD);

	// Setup 8N1
	if (!parity)
	{
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
	}

	if (data_bits == 8)
	{
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
	}

	if (!hardware_control)
	{
		// Disable hardware flow control
		//#ifdef _LINUX
		options.c_cflag &= ~CRTSCTS;
		//#endif
	}

	// Choose raw input
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// Set one second timeout
	options.c_cc[VMIN]  = 1;
	options.c_cc[VTIME] = 10;

	// Set the new options for the port...

	tcsetattr(fd, TCSANOW, &options);
	*/

	//
	// Finally, apply the configuration
	//
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of port %s\n", xbeeDevice);
		return false;
	}
	return true;
}

void close_port(int fd)
{
	close(fd);
}
// ------------------ end copied code ------------------------
