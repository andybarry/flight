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

// labjack header
#include "u3.h"
#include <unistd.h>

#define true 1
#define false 0

/* == define the task here!!
   This file must define the "control()" function call
*/

#include "radio_labjack_ch2.h"
//#include "ltv_perching_task.h"
//#include "elevator_b2t.h"
//#include "elevator_t2b.h"

/*
#define LJ_ioPUT_CONFIG 1000
#define LJ_chTDAC_SCL_PIN_NUM 5119
#define LJ_ioTDAC_COMMUNICATION 506
#define LJ_chTDAC_UPDATE_DACA 5125
*/

#define FIO4 0
#define FIO5 1
#define FIO6 2
#define FIO7 3

// these map FIOx to channel on the transmitter

#define CAR_THROTTLE 1
#define CAR_STEERING 0

//#define AILERON 0
//#define ELEVATOR 1
//#define THROTTLE 2
//#define RUDDER 3

// function that converts 0-255 to voltage
float ServoToVoltageThrottle(int servo);
float ServoToVoltageSteering(int servo);

int SetVoltage(float v, int port, HANDLE hDevice, u3CalibrationInfo caliInfo);

lcm_t * lcm;

int checkI2CErrorcode(uint8 errorcode);


int checkI2CErrorcode(uint8 errorcode)
{
    if(errorcode != 0)
    {
        printf("I2C error : received errorcode %d in response\n", errorcode);
        return -1;
    }
    return 0;
}

// ==== The actual control task thread ====
int radio_labjack()
{

  	float elapsed;
  	double currentTimeDouble, messageTimeDouble;
    struct timeval thisTime;
	static struct timeval td_start,td_end, timestamp_st;
    double servo;
    static int init = true;

    // init here
    
    HANDLE hDevice;
    u3CalibrationInfo caliInfo;
    int localID;
    long DAC1Enable, error;

    //Open first found U3 over USB
    localID = -1;
    if( (hDevice = openUSBConnection(localID)) == NULL)
    {
        if(error > 0)
        {
            printf("Failed to connect to labjack over USB.\n", error);
        }
        
        return 0;
    }

    //Get calibration information from UE9
    if(getCalibrationInfo(hDevice, &caliInfo) < 0)
    {
        if(error > 0)
        {
            printf("Received an error code of %ld\n", error);
        }
        
        closeUSBConnection(hDevice);
        return 0;
    }


    /* Note: The eAIN, eDAC, eDI, and eDO "easy" functions have the ConfigIO parameter.
       If calling, for example, eAIN to read AIN3 in a loop, set the ConfigIO paramter
       to 1 (True) on the first iteration so that the ConfigIO low-level function is
       called to ensure that channel 3 is set to an analog input.  For the rest of the
       iterations in the loop, set the ConfigIO parameter to 0 (False) since the channel
       is already set as analog. */


    //Set DAC0 to 2.1 volts.
    //printf("Calling eDAC to set DAC0 to 2.1 V\n");
    //SetVoltage(2.1, STEERING, hDevice, caliInfo);
    
  
    gettimeofday(&td_start,NULL);

  
    printf("connected.\n");
    
    // ENTER THE CONTROL LOOP HERE
    float elevatorV, aileronV, rudderV, throttleV;
    long int elevatorE, aileronE, rudderE, throttleE;
    
    float temp;
    
    while(1)
    {
      	
      	messageTimeDouble = lcm_hotrod_u.timestamp;
      	
      	
      	//aileronV = ServoToVoltage((uint8_t)lcm_hotrod_u.aileron);
      	//elevatorV = ServoToVoltage((uint8_t)lcm_hotrod_u.elevator);
      	throttleV = ServoToVoltageThrottle((uint8_t)lcm_hotrod_u.throttle);
      	rudderV = ServoToVoltageSteering((uint8_t)lcm_hotrod_u.rudder);
      	
      	// check for a long time since the last message that might
        // indicate that the controller has crashed (and we should turn the throttle off!)
        currentTimeDouble = (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;
        if (currentTimeDouble > messageTimeDouble + 100)
        {
            // we haven't received a control signal in the last 100ms -- ESTOP!!
            //throttleV = ServoToVoltage(0);
            //printf("WARNING: Throttle set to zero because no control signal in the last %f ms.\n", currentTimeDouble - messageTimeDouble);
        }
      	
      	
      	
      	
      	gettimeofday(&td_end,NULL);
        
        /*
        
        // enable for sin(t) testing
       	
       	elapsed = 1000000.0 * (td_end.tv_sec -td_start.tv_sec);
       	elapsed += (td_end.tv_usec - td_start.tv_usec);
       	elapsed=elapsed/1000000.0;
  
        temp=sin(2*3.14159*1*elapsed) + 1;
        //temp *= 255;
        temp *= 50;
        //temp = 128;
        
                
        printf("temp: %f", temp);
      	
      	aileronV = ServoToVoltage(temp);
      	elevatorV = ServoToVoltage(temp);
      	throttleV = ServoToVoltage(temp);
      	rudderV = ServoToVoltage(temp);
      	
        */
      
      	
        lock();

        //printf("a: %f e: %f t: %f r: %f\n", aileronV, elevatorV, throttleV, rudderV);

//        aileronE = SetVoltage(aileronV, AILERON, hDevice, caliInfo1, caliInfo2);
//        elevatorE = SetVoltage(elevatorV, ELEVATOR, hDevice, caliInfo1, caliInfo2);


        throttleE = SetVoltage(throttleV, CAR_THROTTLE, hDevice, caliInfo);
        rudderE = SetVoltage(rudderV, CAR_STEERING, hDevice, caliInfo);
printf("throttle: %f\n", throttleV);
printf("rudder: %f\n", rudderV);
        aileronE = 0;
        elevatorE = 0;
        

        unlock();

        if(elevatorE != 0 || aileronE != 0 || throttleE != 0 || rudderE != 0)
        {
            printf("Error: Codes are: elevatorError: %ld aileronError: %ld thurstError: %ld rudderError: %ld\n", elevatorE, aileronE, throttleE, rudderE);
        
            closeUSBConnection(hDevice);
            return 0;
        }
        
        lcmt_hotrod_u msg;

        gettimeofday(&thisTime, NULL);
        
        msg.timestamp = (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;

        msg.aileron = -1;
        msg.elevator = -1;
        msg.throttle = (uint8_t)lcm_hotrod_u.throttle;
        msg.rudder = (uint8_t)lcm_hotrod_u.rudder;

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
            
    }
  
	return 0;

}

float ServoToVoltageThrottle(int servo)
{
    // 3.2 = max low (NOTE: 0 throttle = 3.2v)
    // 2.5 = middle
    // 1.8 = max high
    float volts = 0.00588235 * servo + 1.5;
    if (volts < 0)
    {
        volts = 0;
    }
    if (volts > 3.0)
    {
        volts = 3.0;
    }
    return volts;
}

float ServoToVoltageSteering(int servo)
{
    // 3.2 = max low (NOTE: 0 throttle = 3.2v)
    // 2.5 = middle
    // 1.8 = max high
    float volts = 0.00784314 * servo + 0.6;
    if (volts < 0)
    {
        volts = 0;
    }
    if (volts > 3.0)
    {
        volts = 3.0;
    }
    return volts;
}


int SetVoltage(float v, int port, HANDLE hDevice, u3CalibrationInfo caliInfo)
{
    /*
        long eDAC( HANDLE Handle,
                   u3CalibrationInfo *CalibrationInfo,
                   long ConfigIO,
                   long Channel,
                   double Voltage,
                   long Binary,
                   long Reserved1,
                   long Reserved2);
    */
//    v = 2;
//    port = 1;
    return eDAC(hDevice, &caliInfo, 0, port, v, 0, 0, 0);
}



