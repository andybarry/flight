/*
 * Publishes LCM messages to over an Xbee to the groundstation.
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
#include <stdexcept>      // std::invalid_argument
#include <map>

#include "../../mavlink-rlg/csailrlg/mavlink.h"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include "mav_pose_t.h" // from Fixie

#include "mavconn.h" // from mavconn

#include "../xbee_to_lcm_bridge2/LcmTransportPart.hpp" // for message size defines
    
#include <string>

#include <stdio.h>   /* Standard input/output definitions */
//#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <inttypes.h>
#include <fstream>
#include <mutex>

#define BAUD_RATE 57600

#define MAX_CHANNELS 255

#define MAX_HOLDING_MESSAGES 255

map<string, int> downsampleAmounts;
map<string, int> downsampleCounters;

lcm_subscription_t* lcm_sub_array[MAX_CHANNELS];

int globalId = 0;


lcm_t * lcm;

int origin_init = 0;
BotGPSLinearize gpsLinearize;
double elev_origin;

char *xbeeDevice = NULL;

mav_pose_t_subscription_t *pose_sub;

uint8_t systemID = getSystemID();

uint8_t serialBuffer[MAVLINK_MAX_PACKET_LEN];

int serialPort_fd;

int open_port(char *port);
bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
void close_port(int fd);
void* serial_wait(void* serial_ptr);

LcmTransportPart globalHoldingArray[MAX_HOLDING_MESSAGES];
int globalNextMessageSlot = 0;

static void usage(void)
{
        fprintf(stderr, "usage: lcm-to-xbee-bridge2 xbee-device channel1 downsample1 [channel2 downsample2] [channel3 downsample3...]\n");
        fprintf(stderr, "    xbee-device: Location of the Xbee (often /dev/ttyUSB0)\n");
        fprintf(stderr, "    channels: LCM channel to transfer over Xbee\n");
        fprintf(stderr, "    downsample: Messages to skip, aka if 2, then send a message, skip 2, send another\n");
        fprintf(stderr, "    \tset to 0 to send all messages\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./lcm-to-xbee-bridge2 /dev/ttyUSB0 STATE_ESTIMATOR_POSE 15 TIMESYNC 0\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    for (int i=0; i < (int)downsampleAmounts.size(); i++)
    {
        lcm_unsubscribe (lcm, lcm_sub_array[i]);
    }

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

int GetMessageIndex(int id)
{
    for (int i=0;i<MAX_HOLDING_MESSAGES; i++)
    {
        if (id == globalHoldingArray[i].id)
        {
            return i;
        }
    }
    return -1;
}


void message_handler(const lcm_recv_buf_t *rbuf, const char* channel, void *userdata)
{
    // check to see if we should send this message or if it should be downsampled
    if (downsampleAmounts.at(channel) != 0 && downsampleAmounts.at(channel) > downsampleCounters.at(channel))
    {
        // downsample this message
        downsampleCounters.at(channel) ++;
        return;
    } else {
        downsampleCounters.at(channel) = 0;
    }
    
    
    // send the message via Xbee
    
    // first, break up the LCM message into a few different parts (if needed)
    int numMessagesNeeded = rbuf->data_size / MAVLINK_LCM_PAYLOAD_SIZE + 1;
    
    char payload[MAVLINK_LCM_PAYLOAD_SIZE];
    int payloadSize;
    
    for (int i=0; i < numMessagesNeeded; i++)
    {
        // send a message with this part of the message
        
        if (i == numMessagesNeeded - 1)
        {
            // this is the last message
            payloadSize = rbuf->data_size % MAVLINK_LCM_PAYLOAD_SIZE;
        } else {
            // this isn't the last message, so fill the payload completely
            payloadSize = MAVLINK_LCM_PAYLOAD_SIZE;
        }
        
        // we cast the void* to a char so that we can do arithmetic on it
        char *buffer = (char*) rbuf->data;
        
        // copy in the relavent payload
        memcpy(&payload, buffer + i * MAVLINK_LCM_PAYLOAD_SIZE, payloadSize);
        
        // build mavlink message
        mavlink_message_t mavmsg;
        
        mavlink_msg_lcm_transport_pack(
            systemID,   
            201,
            &mavmsg,
		    (int32_t) getTimestampNow(),    // timestamp
            channel,                        // channel name
            globalId,                       // ID for this message
		    i,                              // which message this is
            numMessagesNeeded,              // total messages required
            payloadSize,                     // size of this payload
            payload);                       // payload data
            
            
        int messageLength = mavlink_msg_to_send_buffer(serialBuffer, &mavmsg);
        
        int written = write(serialPort_fd, (char*)serialBuffer, messageLength);
        
        if (written != messageLength)
        {
	        fprintf(stderr, "\nERROR: Unable to send message over serial port.\n");
        }
            
    }
    
    globalId++;
    if (globalId > 65535)
    {
        globalId = 0;
    }
    
}


void sendLcmMessage(mavlink_message_t *message)
{
    // send this mavlink message as an LCM message
    switch(message->msgid)
    {
        // process messages here
        case MAVLINK_MSG_ID_LCM_TRANSPORT:
            
            // rebuild the LCM message
            mavlink_lcm_transport_t transportIn;
            
            mavlink_msg_lcm_transport_decode(message, &transportIn);
            
            // check to see if we know about this message
            int index;
            index = GetMessageIndex(transportIn.msg_id);
            if (index >= 0)
            {
                globalHoldingArray[index].AddMessage(transportIn);
                if (globalHoldingArray[index].IsComplete())
                {
                    // send the message
                    
                    // send the lcm message
                    lcm_publish(lcm, globalHoldingArray[index].channelName.c_str(), globalHoldingArray[index].data,     
                        globalHoldingArray[index].totalDataSizeSoFar);                    
                }
            } else {
                // we don't know about this message yet
                globalHoldingArray[globalNextMessageSlot].AddMessage(transportIn);
                globalNextMessageSlot ++;
                if (globalNextMessageSlot >= MAX_HOLDING_MESSAGES)
                {
                    globalNextMessageSlot = 0;
                }
            }
            
            break;            
            
            
        default:
            printf("Error: got unknown MAVLINK message: %d\n", message->msgid);
    }
}


// threaded mavlink reading
void* MavlinkReadingThread(void *fd_ptr)
{  
    while (true)
    {
        serial_wait((void*)fd_ptr);
    }
    return NULL;
}

// threaded lcm reading
void* LcmReadingThread(void *nothing)
{  
    while (true)
    {
        lcm_handle(lcm);
    }
    return NULL;
}


int main(int argc,char** argv)
{
    if (argc != 2 && argc < 4) {
        usage();
        exit(0);
    }

    xbeeDevice = argv[1];
    
    int numChannels = (argc - 2)/2;
    if (numChannels > MAX_CHANNELS)
    {
        printf("Error: channels exceeds maximum supported number.\n");
        exit(0);
    }
    
    // check to make sure there is a downsample argument with each channel
    if ((argc - 2) % 2 != 0)
    {
        printf("Error: Each channel must have a downsample amount.  If you don't want downsampling, set it to 0.\n");
        exit(0);
    }
    
    // SETUP SERIAL PORT

	printf("LCM / Xbee bridge started\n");

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
    
    printf("Publishing to Xbee: %s\nSending:\n", xbeeDevice);

    if (numChannels == 0)
    {
        printf("\t(no channels, only receiving)\n");
    }

    // subscribe to all of the channels we need
    for (int i=0; i < numChannels; i++)
    {
        lcm_sub_array[i] = lcm_subscribe(lcm, argv[2+i*2], &message_handler, NULL);
        
        // ensure that the next argument is a number
        int thisDownsampleAmount;
        try
        {
            thisDownsampleAmount = std::stoi(argv[3+i*2]);
        } catch (const std::invalid_argument &ia) {
            printf("\nError: invalid downsample factor of \"%s\" for channel: %s\n\n", argv[3+2*i], argv[2+2*i]);
            exit(0);
        }
        
        downsampleAmounts.insert(pair<string, int>(argv[2+i*2], thisDownsampleAmount));
        downsampleCounters.insert(pair<string, int>(argv[2+i*2], 0));

        
        printf("\t%s | downsample: %d\n", argv[2+i*2], thisDownsampleAmount);
    }

    signal(SIGINT,sighandler);
    
    pthread_t lcmReadThread, mavlinkReadThread;
    
    int* fd_ptr = &serialPort_fd;
    
    // fire off the two threads that wait for serial messages
    // and wait for LCM messages
    pthread_create( &lcmReadThread, NULL, LcmReadingThread, NULL);
    pthread_create( &mavlinkReadThread, NULL, MavlinkReadingThread, fd_ptr);
    
    // wait for the threads to rejoin (which they never will, this is just a blocking call)
    pthread_join( lcmReadThread, NULL);
    

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
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
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


// -------------- slightly modified copied code --------------


/**
* @brief Serial function
*
* This function blocks waiting for serial data in it's own thread
* and forwards the data once received.
*/
void* serial_wait(void* serial_ptr)
{
    bool verbose = false;
    bool debug = false;
    bool silent = false;
    
	int fd = *((int*) serial_ptr);

	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;

	//if (debug) printf("Checking for new data on serial port\n");
	// Block until data is available, read only one byte to be able to continue immediately
	//char buf[MAVLINK_MAX_PACKET_LEN];
	uint8_t cp;
	mavlink_message_t message;
	mavlink_status_t status;
	uint8_t msgReceived = false;
	//tcflush(fd, TCIFLUSH);
    
    // before we do a read we need to grab the serial port mutex
	if (read(fd, &cp, 1) > 0)
	{
    
		// Check if a message could be decoded, return the message in case yes
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
		if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)
		{
			if (verbose || debug) printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			if (debug)
			{
				unsigned char v=cp;
				fprintf(stderr,"%02x ", v);
			}
		}
		lastStatus = status;
	}
	else
	{
		if (!silent) fprintf(stderr, "ERROR: Could not read from port %s\n", xbeeDevice);
	}

	// If a message could be decoded, handle it
	if(msgReceived)
	{
		if (verbose || debug) std::cout << std::dec << "Received and forwarded serial port message with id " << static_cast<unsigned int>(message.msgid) << " from system " << static_cast<int>(message.sysid) << std::endl;

		// Do not send images over serial port

		// DEBUG output
		if (debug)
		{
			//fprintf(stderr,"Forwarding SERIAL -> LCM: ");
			unsigned int i;
			uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
			unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
			if (messageLength > MAVLINK_MAX_PACKET_LEN)
			{
				fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
			}
			else
			{
				for (i=0; i<messageLength; i++)
				{
					unsigned char v=buffer[i];
					fprintf(stderr,"%02x ", v);
				}
				fprintf(stderr,"\n");
			}
		}

		// Send out packets to LCM
		// Send over LCM
        /*
		if (pc2serial)
		{
			sendMAVLinkMessage(lcm, &message, MAVCONN_LINK_TYPE_UART_VICON);
		}
		else
		{
			sendMAVLinkMessage(lcm, &message, MAVCONN_LINK_TYPE_UART);
		}
		*/
		sendLcmMessage(&message); 
	}
	return NULL;
				}

// -----------------------------------------------------------

