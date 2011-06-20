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

#include "radio_labjack.h"
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
#define AILERON 0
#define ELEVATOR 1
#define THROTTLE 2
#define RUDDER 3

// these map to analog inputs on the labjack
#define ELEVATOR_FEEDBACK 0
#define RUDDER_FEEDBACK 2
#define THROTTLE_FEEDBACK 3
#define AILERON_FEEDBACK 1

// function that converts 0-255 to voltage
float ServoToVoltage(int servo);

int SetVoltage(float v, int port, HANDLE hDevice, u3TdacCalibrationInfo caliInfo1, u3TdacCalibrationInfo caliInfo2);

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
        printf("No device?");
        return 0;
    }
    
    // configure IO ports FIO4 and FIO5 to be digital
    // note that you should make sure to do this before getting
    // calibration info
    configIO_example(hDevice);
    
    u3TdacCalibrationInfo caliInfo1, caliInfo2;
    

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
    
    // Getting calibration information from LJTDAC
    if(getTdacCalibrationInfo(hDevice, &caliInfo1, 4) < 0)
    {
        if(error > 0)
        {
            printf("Received an error code of %ld\n", error);
        }
        
        closeUSBConnection(hDevice);
        return 0;
    }
    
    if(getTdacCalibrationInfo(hDevice, &caliInfo2, 6) < 0)
    {
        if(error > 0)
        {
            printf("Received an error code of %ld\n", error);
        }
        
        closeUSBConnection(hDevice);
        return 0;
    }
    
  
    gettimeofday(&td_start,NULL);

  
    printf("connected.\n");
    
    float elevatorV, aileronV, rudderV, throttleV;
    long int elevatorE, aileronE, rudderE, throttleE;
    
    float temp;
    
    float biasElevator = 0, biasRudder = 0, biasThrottle = 0, biasAileron = 0;
    
    printf("Estimating voltage bias...\n");
    
    aileronV = ServoToVoltage(128);
  	elevatorV = ServoToVoltage(128);
  	throttleV = ServoToVoltage(128);
  	rudderV = ServoToVoltage(128);
  	
    aileronE = SetVoltage(aileronV + biasAileron, AILERON, hDevice, caliInfo1, caliInfo2);
    elevatorE = SetVoltage(elevatorV + biasElevator, ELEVATOR, hDevice, caliInfo1, caliInfo2);
    throttleE = SetVoltage(throttleV + biasThrottle, THROTTLE, hDevice, caliInfo1, caliInfo2);
    rudderE = SetVoltage(rudderV + biasRudder, RUDDER, hDevice, caliInfo1, caliInfo2);
    
    double dblVoltage, biasSumElevator, biasSumAileron, biasSumThrottle, biasSumRudder;
    
    int q;
    int biasNum = 1000;
    
    biasSumElevator = 0;
    biasSumAileron = 0;
    biasSumThrottle = 0;
    biasSumRudder = 0;

    for (q=0;q<biasNum;q++)
    {
        if((error = eAIN(hDevice, &caliInfo, 1, &DAC1Enable, ELEVATOR_FEEDBACK, 31, &dblVoltage, 0, 0, 0, 0, 0, 0)) != 0)
        {
            printf("Received an error code of %ld\n", error);
            closeUSBConnection(hDevice);
            return 0;
        }
        biasSumElevator += dblVoltage;
        
        if((error = eAIN(hDevice, &caliInfo, 1, &DAC1Enable, RUDDER_FEEDBACK, 31, &dblVoltage, 0, 0, 0, 0, 0, 0)) != 0)
        {
            printf("Received an error code of %ld\n", error);
            closeUSBConnection(hDevice);
            return 0;
        }
        biasSumRudder += dblVoltage;
        
        if((error = eAIN(hDevice, &caliInfo, 1, &DAC1Enable, THROTTLE_FEEDBACK, 31, &dblVoltage, 0, 0, 0, 0, 0, 0)) != 0)
        {
            printf("Received an error code of %ld\n", error);
            closeUSBConnection(hDevice);
            return 0;
        }
        biasSumThrottle += dblVoltage;
        
        if((error = eAIN(hDevice, &caliInfo, 1, &DAC1Enable, AILERON_FEEDBACK, 31, &dblVoltage, 0, 0, 0, 0, 0, 0)) != 0)
        {
            printf("Received an error code of %ld\n", error);
            closeUSBConnection(hDevice);
            return 0;
        }
        biasSumAileron += dblVoltage;
        
        sleep(0.1);
    }
    
    biasElevator = biasSumElevator / (float)biasNum - elevatorV;
    biasRudder = biasSumRudder / (float)biasNum - rudderV;
    biasThrottle = biasSumThrottle / (float)biasNum - throttleV;
    biasAileron = biasSumAileron / (float)biasNum - aileronV;
    printf("Elevator bias:\t%.3f\n", biasElevator);
    printf("Rudder bias:\t%.3f\n", biasRudder);
    printf("Throttle bias:\t%.3f\n", biasThrottle);
    printf("Aileron bias:\t%.3f\n\n", biasAileron);

    // ENTER THE CONTROL LOOP HERE
    while(1)
    {
    
      	messageTimeDouble = lcm_hotrod_u.timestamp;
      	
      	
      	aileronV = ServoToVoltage((uint8_t)lcm_hotrod_u.aileron);
      	elevatorV = ServoToVoltage((uint8_t)lcm_hotrod_u.elevator);
      	throttleV = ServoToVoltage((uint8_t)lcm_hotrod_u.throttle);
      	rudderV = ServoToVoltage((uint8_t)lcm_hotrod_u.rudder);
      	
      	// check for a long time since the last message that might
        // indicate that the controller has crashed (and we should turn the throttle off!)
        currentTimeDouble = (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;
        if (currentTimeDouble > messageTimeDouble + 100)
        {
            // we haven't received a control signal in the last 100ms -- ESTOP!!
            throttleV = ServoToVoltage(0);
            printf("WARNING: Throttle set to zero because no control signal in the last %f ms.\n", currentTimeDouble - messageTimeDouble);
        }
      	
      	
      	
      	
      	gettimeofday(&td_end,NULL);
        
        /*
        // enable fpr sin(t) testing
       	elapsed = 1000000.0 * (td_end.tv_sec -td_start.tv_sec);
       	elapsed += (td_end.tv_usec - td_start.tv_usec);
       	elapsed=elapsed/1000000.0;
  
        temp=sin(2*3.14159*1*elapsed) + 1;
        temp *= 255;
        temp = 128;
                
        //printf("temp: %f\n", temp);
      	
      	aileronV = ServoToVoltage(temp);
      	elevatorV = ServoToVoltage(temp);
      	throttleV = ServoToVoltage(0);
      	rudderV = ServoToVoltage(temp);
      	*/
      	
        lock();

        //printf("a: %f e: %f t: %f r: %f\n", aileronV, elevatorV, throttleV, rudderV);

        aileronE = SetVoltage(aileronV - biasAileron, AILERON, hDevice, caliInfo1, caliInfo2);
        elevatorE = SetVoltage(elevatorV - biasElevator, ELEVATOR, hDevice, caliInfo1, caliInfo2);
        throttleE = SetVoltage(throttleV - biasThrottle, THROTTLE, hDevice, caliInfo1, caliInfo2);
        rudderE = SetVoltage(rudderV - biasRudder, RUDDER, hDevice, caliInfo1, caliInfo2);

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

        msg.aileron = (uint8_t)lcm_hotrod_u.aileron;
        msg.elevator = (uint8_t)lcm_hotrod_u.elevator;
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

float ServoToVoltage(int servo)
{
    // 3.2 = max low (NOTE: 0 throttle = 3.2v)
    // 2.5 = middle
    // 1.8 = max high
    float volts = -0.00549 * servo + 3.2;
    if (volts < 1.8)
    {
        volts = 1.8;
    }
    if (volts > 3.2)
    {
        volts = 3.2;
    }
    return volts;
}

int SetVoltage(float v, int port, HANDLE hDevice, u3TdacCalibrationInfo caliInfo1, u3TdacCalibrationInfo caliInfo2)
{
    u3TdacCalibrationInfo caliInfo;
    uint8 ljdacCommandByte;
    
    int err;
    uint8 options, speedAdjust, sdaPinNum, sclPinNum, address, numBytesToSend, numBytesToReceive, errorcode;
    uint16 binaryVoltage;
    uint8 bytesCommand[5];
    uint8 bytesResponse[64];
    uint8 ackArray[4];
    int i;

    err = 0;
    
    if (port == FIO4)
    {
        ljdacCommandByte = 0x30;
        sdaPinNum = 5;           //SDAPinNum : FIO5 connected to pin DIOB
        sclPinNum = 4;           //SCLPinNum : FIO4 connected to pin DIOA
        caliInfo = caliInfo1;
    } else if (port == FIO5)
    {
        ljdacCommandByte = 0x31;
        sdaPinNum = 5;           //SDAPinNum : FIO5 connected to pin DIOB
        sclPinNum = 4;           //SCLPinNum : FIO4 connected to pin DIOA
        caliInfo = caliInfo1;
    } else if (port == FIO6)
    {
        ljdacCommandByte = 0x30;
        sdaPinNum = 7;           //SDAPinNum : FIO7 connected to pin DIOB
        sclPinNum = 6;           //SCLPinNum : FIO6 connected to pin DIOA
        
        caliInfo = caliInfo2;
    } else if (port == FIO7)
    {
        ljdacCommandByte = 0x31;
        sdaPinNum = 7;           //SDAPinNum : FIO7 connected to pin DIOB
        sclPinNum = 6;           //SCLPinNum : FIO6 connected to pin DIOA
        
        caliInfo = caliInfo2;
    }
    
    

    //Setting up parts I2C command that will remain the same throughout this example
    options = 0;             //I2COptions : 0
    speedAdjust = 0;         //SpeedAdjust : 0 (for max communication speed of about 130 kHz)


    /* Set DACA to 1.2 volts. */

    //Setting up I2C command
    //Make note that the I2C command can only update 1 DAC channel at a time.
    address = (uint8)(0x24);  //Address : h0x24 is the address for DAC
    numBytesToSend = 3;       //NumI2CByteToSend : 3 bytes to specify DACA and the value
    numBytesToReceive = 0;    //NumI2CBytesToReceive : 0 since we are only setting the value of the DAC
    bytesCommand[0] = (uint8)(ljdacCommandByte);  //LJTDAC command byte : h0x30 (DACA)
    getTdacBinVoltCalibrated(&caliInfo, 0, v, &binaryVoltage);
    bytesCommand[1] = (uint8)(binaryVoltage/256);          //value (high)
    bytesCommand[2] = (uint8)(binaryVoltage & (0x00FF));   //value (low)

    //Performing I2C low-level call
    err = I2C(hDevice, options, speedAdjust, sdaPinNum, sclPinNum, address, numBytesToSend, numBytesToReceive, bytesCommand, &errorcode, ackArray, bytesResponse);

    if(checkI2CErrorcode(errorcode) == -1 || err == -1)
    {
        return -1;
    }
    return 0;
}




//Sends a ConfigIO low-level command that configures the FIO4 and FIO5 lines to digital.
int configIO_example(HANDLE hDevice)
{
    uint8 sendBuff[12], recBuff[12];
    uint16 checksumTotal;
    int sendChars, recChars;

    sendBuff[1] = (uint8)(0xF8);  //Command byte
    sendBuff[2] = (uint8)(0x03);  //Number of data words
    sendBuff[3] = (uint8)(0x0B);  //Extended command number

    sendBuff[6] = 7;  //Writemask : Setting writemask for TimerCounterConfig (bit 0),
                    //            DAC1Enable (bit 1) and FIOAnalog (bit 2)

    sendBuff[7] = 0;  //Reserved
    sendBuff[8] = 64; //TimerCounterConfig : disable timers and counters. set
                    //                     TimerCounterPinOffset to 4 (bits 4-7)
    sendBuff[9] = 0;  //DAC1 enable : disabling
    sendBuff[10] = 0; //FIOAnalog : setting FIO channels to digital
    sendBuff[11] = 0; //EIOAnalog : Not setting anything
    extendedChecksum(sendBuff, 12);

    //Sending command to U3
    if( (sendChars = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuff, 12)) < 12)
    {
        if(sendChars == 0)
            printf("ConfigIO error : write failed\n");
        else
            printf("ConfigIO error : did not write all of the buffer\n");
        return -1;
    }

    //Reading response from U3
    if( (recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP2_IN, recBuff, 12)) < 12)
    {
        if(recChars == 0)
            printf("ConfigIO error : read failed\n");
        else
            printf("ConfigIO error : did not read all of the buffer\n");
        return -1;
    }

    checksumTotal = extendedChecksum16(recBuff, 12);
    if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5])
    {
        printf("ConfigIO error : read buffer has bad checksum16(MSB)\n");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4])
    {
        printf("ConfigIO error : read buffer has bad checksum16(LBS)\n");
        return -1;
    }

    if( extendedChecksum8(recBuff) != recBuff[0])
    {
        printf("ConfigIO error : read buffer has bad checksum8\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x03) || recBuff[3] != (uint8)(0x0B) )
    {
        printf("ConfigIO error : read buffer has wrong command bytes\n");
        return -1;
    }

    if( recBuff[6] != 0)
    {
        printf("ConfigIO error : read buffer received errorcode %d\n", recBuff[6]);
        return -1;
    }

    if( recBuff[8] != 64)
    {
        printf("ConfigIO error : TimerCounterConfig did not get set correctly\n");
        return -1;
    }

    if( recBuff[10] != 0 && recBuff[10] != (uint8)(0x0F))
    {
        printf("ConfigIO error : FIOAnalog did not set correctly\n");
        return -1;
    }

    return 0;
}

