//Author: LabJack
//June 25, 2009
//Header for U3 example helper functions.
//
//History
//-added easy functions
//-added I2C function and LJTDAC functions and structure (09/04/2007)
//-fixed memory leak in ehFeedback and I2C functions (09/27/2007)
//-fixed some bugs in "easy" functions (09/27/2007)
//-added U3-LV/HV support (04/07/2008)
//-fixed bug in eAIN for positive channel 30 - temp sensor (04/25/2008)
//-Modified calibration constants structs.  Modified the names and code of the
// functions that apply the calibration constants. (06/25/2009)

#ifndef _U3_H
#define _U3_H

#include <sys/time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "labjackusb.h"


typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

//Structure for storing calibration constants
struct U3_CALIBRATION_INFORMATION {
    uint8 prodID;
    double hardwareVersion; //helps to determine which calibration calculations
                            //to use
    int highVoltage;        //indicates if the device is U3-HV
    double ccConstants[20];
    /*
    Calibration constants order
    0 - LV AIN SE Slope
    1 - LV AIN SE Offset
    2 - LV AIN Diff Slope
    3 - LV AIN Diff Offset
    4 - DAC0 Slope
    5 - DAC0 Offset
    6 - DAC1 Slope
    7 - DAC1 Offset
    8 - Temp Slope
    9 - Vref @Cal
    10 - Vref*1.5 @Cal
    11 - Vreg @Cal
    12 - HV AIN0 Slope
    13 - HV AIN1 Slope
    14 - HV AIN2 Slope
    15 - HV AIN3 Slope
    16 - HV AIN0 Offset
    17 - HV AIN1 Offset
    18 - HV AIN2 Offset
    19 - HV AIN3 Offset
    */
};

typedef struct U3_CALIBRATION_INFORMATION u3CalibrationInfo;

//Structure for storing LJTDAC calibration constants
struct U3_TDAC_CALIBRATION_INFORMATION{
    uint8 prodID;
    double ccConstants[4];
    /*
    DAC Calibration constants order
    0 - SlopeA;
    1 - OffsetA;
    2 - SlopeB;
    3 - OffsetB;
    */
};

typedef struct U3_TDAC_CALIBRATION_INFORMATION u3TdacCalibrationInfo;


/* Functions */

void normalChecksum( uint8 *b,
                     int n);
//Adds checksum to a data packet for normal command format.
//b = data packet for normal command
//n = size of data packet

void extendedChecksum( uint8 *b,
                       int n);
//Adds checksum to a data packet for extended command format.
//b = data packet for extended command
//n = size of data packet

uint8 normalChecksum8( uint8 *b,
                       int n);
//Returns the Checksum8 for a normal command data packet.
//b = data packet for normal command
//n = size of data packet

uint16 extendedChecksum16( uint8 *b,
                           int n);
//Returns the Checksum16 for a extended command data packet.
//b = data packet for extended command
//n = size of data packet

uint8 extendedChecksum8( uint8 *b);
//Returns the Checksum8 for a extended command data packet.
//b = data packet for extended command

HANDLE openUSBConnection( int localID);
//Opens a U3 with the given localID.  Returns NULL on failure, or a HANDLE on
//success.

void closeUSBConnection( HANDLE hDevice);
//Closes a HANDLE to a U3 device.

long getTickCount();
//Returns the number of milliseconds that has elasped since the system was
//started.

long getCalibrationInfo( HANDLE hDevice,
                         u3CalibrationInfo *caliInfo);
//Gets calibration information from memory blocks 0-4 of a U3.  Returns the
//calibration information in a calibrationInfo structure.
//hDevice = handle to a U3 device
//caliInfo = structure where calibrarion information will be stored

long getTdacCalibrationInfo( HANDLE hDevice,
                             u3TdacCalibrationInfo *caliInfo,
                             uint8 DIOAPinNum);
//Gets calibration information from the EEPROM of a LJTick-DAC (LJTDAC).
//Returns the calibration information in a u3TdacCalibrationInfo structure.
//hDevice = handle to a U3 device
//caliInfo = structure where LJTDAC calibration information will be stored
//DIOAPinNum = The U3 digital IO line where the LJTDAC DIOA pin is connected.
//             The DIOB pin is assumed to be the next digital IO line.


double FPuint8ArrayToFPDouble( uint8 *buffer,
                               int startIndex);
//Converts a fixed point byte array (starting a startIndex) to a floating point
//double value.  This function is used primarily by getCalibrationInfo.

long isCalibrationInfoValid(u3CalibrationInfo *caliInfo);
//Performs a simple check to determine if the caliInfo struct was set up by
//getCalibrationInfo.  Returns 0 if caliInfo is not valid, or 1 if it is.
//caliInfo = structure where calibrarion information is stored

long isTdacCalibrationInfoValid(u3TdacCalibrationInfo *caliInfo);
//Performs a simple check to determine if the caliInfo struct was set up by
//getLJTDACCalibrationInfo.  Returns 0 if caliInfo is not valid, or 1 if it is.
//caliInfo = structure where LJTDAC calibration information is stored

long getAinVoltCalibrated( u3CalibrationInfo *caliInfo,
                           int dac1Enabled,
                           uint8 negChannel,
                           uint16 bytesVolt,
                           double *analogVolt);
//Translates the binary AIN reading from the U3, to a voltage value
//(calibrated) in Volts.  Call getCalibrationInfo first to set up caliInfo.
//Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20 and 1.21.  Function will also work
//for hardware version 1.30 U3-LV, but not U3-HV.
//caliInfo = structure where calibrarion information is stored
//dac1Enabled = If this is nonzero (True), then it is indicated that DAC1 is
//             enabled and analog voltage will be calculated with Vreg.  If
//             this is 0 (False), then it is indicated that DAC1 is disabled
//             and the analog voltage will be calculated with the AIN slopes
//             and offsets.
//negChannel = the negative channel of the differential analog reading
//bytesVolt = the 2 byte voltage that will be converted
//analogVolt = the converted analog voltage

long getAinVoltCalibrated_hw130( u3CalibrationInfo *caliInfo,
                                 uint8 positiveChannel,
                                 uint8 negChannel,
                                 uint16 bytesVolt,
                                 double *analogVolt);
//Translates the binary AIN reading from the U3, to a voltage value
//(calibrated) in Volts.  Call getCalibrationInfo first to set up caliInfo.
//Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.30 (U3-LV/HV).
//caliInfo = structure where calibrarion information is stored
//positiveChannel = the positive channel of the differential analog reading
//negChannel = the negative channel of the differential analog reading
//bytesVolt = the 2 byte voltage that will be converted
//analogVolt = the converted analog voltage

long getDacBinVoltCalibrated( u3CalibrationInfo *caliInfo,
                              int dacNumber,
                              double analogVolt,
                              uint8 *bytesVolt);
//Translates a analog output voltage value (Volts) to a binary 8 bit value
//(calibrated) that can be sent to a U3.  Call getCalibrationInfo first to set
//up caliInfo.  Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20, 1.21 and 1.30, and does the
//same thing as the analogToCalibratedBinary8BitVoltage function.
//caliInfo = structure where calibrarion information is stored
//DACNumber - channel number of the DAC
//analogVolt = the analog voltage that will be converted
//bytesVolt = the converted binary 8 bit value

long getDacBinVoltCalibrated8Bit( u3CalibrationInfo *caliInfo,
                                  int dacNumber,
                                  double analogVolt,
                                  uint8 *bytesVolt8);
//Translates a analog output voltage value (Volts) to a binary 8 bit value
//(calibrated) that can be sent to a U3.  Call getCalibrationInfo first to set
//up caliInfo.  Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20, 1.21 and 1.30.
//caliInfo = structure where calibrarion information is stored
//dacNumber - channel number of the DAC
//analogVolt = the analog voltage that will be converted
//bytesVolt8 = the converted binary 8 bit value

long getDacBinVoltCalibrated16Bit( u3CalibrationInfo *caliInfo,
                                   int dacNumber,
                                   double analogVolt,
                                   uint16 *bytesVolt16);
//Translates a analog output voltage value (Volts) to a binary 16 bit value
//(calibrated) that can be sent to a U3. Call getCalibrationInfo first to set
//up caliInfo.  Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.30 (U3-LV/HV).
//caliInfo = structure where calibrarion information is stored
//dacNumber - channel number of the DAC
//analogVolt = the analog voltage that will be converted
//bytesVolt16 = the converted binary 16 bit value

long getTdacBinVoltCalibrated( u3TdacCalibrationInfo *caliInfo,
                               int dacNumber,
                               double analogVolt,
                               uint16 *bytesVolt);
//Translates a voltage value (Volts) to binary analog input bytes (calibrated)
//that can be sent to a LJTick-DAC (LJTDAC).  Call getTdacCalibrationInfo
//first to set up caliInfo.  Returns -1 on error, 0 on success.
//caliInfo = structure where LJTDAC calibrarion information is stored
//dacNumber - channel number of the DAC (0 = DACA, 1 = DACB)
//analogVolt = the analog voltage that will be converted
//bytesVolt = the converted 2 byte voltage

long getTempKCalibrated( u3CalibrationInfo *caliInfo,
                         uint32 bytesTemp,
                         double *kelvinTemp);
//Translates the binary reading from the U3, to a temperature value
//(calibrated) in Kelvins.  Call getCalibrationInfo first to set up caliInfo.
//Returns -1 on error, 0 on success.
//caliInfo = structure where calibrarion information is stored
//bytesTemp = the 2 byte binary temperature that will be converted
//kelvinTemp = the converted Kelvin temperature


long getAinVoltUncalibrated( int dac1Enabled,
                             uint8 negChannel,
                             uint16 bytesVolt,
                             double *analogVolt);
//Translates the binary AIN reading from the U3, to a voltage value
//(uncalibrated) in Volts. Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20 and 1.21.
//dac1Enabled = If this is nonzero (True), then it is indicated that DAC1 is
//              enabled and analog voltage will be calculated with Vreg.  If
//              this is 0 (False), then it is indicated that DAC1 is disabled
//              and the analog voltage will be calculated with the AIN slopes
//              and offsets.
//negChannel = the negative channel of the differential analog reading
//bytesVolt = the 2 byte voltage that will be converted
//analogVolt = the converted analog voltage

long getAinVoltUncalibrated_hw130( int highVoltage,
                                   uint8 positiveChannel,
                                   uint8 negChannel,
                                   uint16 bytesVolt,
                                   double *analogVolt);
//Translates the binary AIN reading from the U3, to a voltage value
//(uncalibrated) in Volts. Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.30 (U3-LV/HV).
//highVoltage = Set to 1 to indicate that U3-HV calculations should be used
//              for the binary to voltage conversion, otherwise U3-LV voltage
//              calculations will be used.
//positiveChannel = the positive channel of the differential analog reading
//negChannel = the negative channel of the differential analog reading
//bytesVolt = the 2 byte voltage that will be converted
//analogVolt = the converted analog voltage

long getDacBinVoltUncalibrated( int dacNumber,
                                double analogVolt,
                                uint8 *bytesVolt);
//Translates a DAC voltage value (Volts) to a binary 8 bit value (uncalibrated)
//that can be sent to a U3.  Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20 and 1.21, and does the same
//thing as the analogToUncalibratedBinary8BitVoltage function.
//dacNumber - channel number of the DAC
//analogVolt = the analog voltage that will be converted
//bytesVolt = the converted binary 8 bit value

long getDacBinVoltUncalibrated8Bit( int dacNumber,
                                    double analogVolt,
                                    uint8 *bytesVolt8);
//Translates a DAC voltage value (Volts) to a binary 8 bit value (uncalibrated)
//that can be sent to a U3.  Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20 and 1.21.
//dacNumber - channel number of the DAC
//analogVoltage = the analog voltage that will be converted
//bytesVoltage = the converted binary 8 bit value

long getDacBinVoltUncalibrated16Bit( int dacNumber,
                                     double analogVolt,
                                     uint16 *bytesVolt16);
//Translates a DAC voltage value (Volts) to a binary 16 bit value
//(uncalibrated) that can be sent to a U3-LV/HV.  Returns -1 on error, 0 on
//success.
//This function is for U3 hardware versions 1.30 (U3-LV/HV).
//dacNumber - channel number of the DAC
//analogVoltage = the analog voltage that will be converted
//bytesVoltage = the converted binary 16 bit value

long getTempKUncalibrated( uint16 bytesTemp,
                           double *kelvinTemp);
//Translates the binary analog bytes read from the U3, to a temperature value
//(uncalibrated) in Kelvins.  Call getCalibrationInfo first to set up caliInfo.
//Returns -1 on error, 0 on success.
//bytesTemp = the 2 byte binary temperature that will be converted
//kelvinTemp = the converted Kelvin temperature

long I2C( HANDLE hDevice,
          uint8 I2COptions,
          uint8 SpeedAdjust,
          uint8 SDAPinNum,
          uint8 SCLPinNum,
          uint8 Address,
          uint8 NumI2CBytesToSend,
          uint8 NumI2CBytesToReceive,
          uint8 *I2CBytesCommand,
          uint8 *Errorcode,
          uint8 *AckArray,
          uint8 *I2CBytesResponse);
//This function will perform the I2C low-level function call.  Please refer to
//section 5.3.19 of the U3 User's Guide for parameter documentation.  Returns
//-1 on error, 0 on success.
//hDevice = handle to a U3 device
//I2COptions = byte 6 of the command
//SpeedAdjust = byte 7 of the command
//SDAPinNum = byte 8 of the command
//SCLPinNum = byte 9 of the command
//Address = byte 10 of the command
//NumI2CBytesToSend = byte 12 of the command
//NumI2CBytesToReceive = byte 13 of the command
//*I2CBytesCommand = Array that holds bytes 14 and above of the command.  Needs
//                   to be at least NumI2CBytesToSend elements in size.
//*Errorcode = returns byte 6 of the response
//*AckArray = Array that returns bytes 8 - 11 of the response.  Needs to be at
//            least 4 elements in size.
//*I2CBytesResponse = Array that returns bytes 12 and above of the response.
//                    Needs to be at least NumI2CBytesToReceive elements in
//                    size.


/* Easy Functions (Similar to the easy functions in the UD driver for Windows) */

long eAIN( HANDLE Handle,
           u3CalibrationInfo *CalibrationInfo,
           long ConfigIO,
           long *DAC1Enable,
           long ChannelP,
           long ChannelN,
           double *Voltage,
           long Range,
           long Resolution,
           long Settling,
           long Binary,
           long Reserved1,
           long Reserved2);
//An "easy" function that returns a reading from one analog input.  This
//function does not automatically configure the specified channels as analog
//input, unless ConfigIO is set as True.  Returns 0 for no error, or -1 or >0
//value (low-level errorcode) on error.
//Call getCalibrationInfo first to set up CalibrationInfo.
//Handle = Handle to a U3 device.
//CalibrationInfo = Structure where calibration information is stored.
//ConfigIO = If this is nonzero (True), then 1 or 2 ConfigIO low-level function
//           calls will be made in addition to the 1 Feedback call to set the
//           specified Channels to analog inputs.  If this is 0 (False), then
//           only a Feedback low-level call will be made, and an error will be
//           returned if the specified Channels are not already set to analog
//           inputs.
//DAC1Enable = This parameter helps to determine the appropriate equation to
//             use when calculating Voltage.  If the long variable that is
//             being pointed to is nonzero (True), then it is indicated that
//             DAC1 is enabled.  If it is 0 (False), then it is indicated that
//             DAC1 is disabled.  For both case, if ConfigIO is set to True,
//             then input value will be ignored and the output value will be
//             set to the current DAC1Enable value in the ConfigIO low-level
//             response.
//ChannelP = The positive AIN channel to acquire.
//ChannelN = The negative AIN channel to acquire.  For single-ended channels on
//           the U3, this parameter should be 31 (see Section 2.6.1).
//Voltage = Returns the analog input reading, which is generally a voltage.
//Range = Ignored on the U3.
//Resolution = Pass a nonzero value to enable QuickSample.
//Settling = Pass a nonzero value to enable LongSettling.
//Binary = If this is nonzero (True), the Voltage parameter will return the raw
//         binary value.
//Reserved (1&2) = Pass 0.


long eDAC( HANDLE Handle,
           u3CalibrationInfo *CalibrationInfo,
           long ConfigIO,
           long Channel,
           double Voltage,
           long Binary,
           long Reserved1,
           long Reserved2);
//An "easy" function that writes a value to one analog output.  This function
//does not automatically enable the specified analog output, unless ConfigIO is
//set as True.  Returns 0 for no error, or -1 or >0 value (low-level errorcode)
//on error.
//Call getCalibrationInfo first to set up CalibrationInfo.
//Handle = Handle to a U3 device.
//CalibrationInfo = structure where calibrarion information is stored
//ConfigIO = If this is nonzero (True) and Channel is 1, then 1 ConfigIO
//           low-level function call will be made in addition to the 1 Feedback
//           call to enable DAC1.  If this is 0 (False), then only a Feedback
//           low-level call will be made, and an error will be returned if DAC1
//           is not already enabled.
//Channel = The analog output channel to write to.
//Voltage = The voltage to write to the analog output.
//Binary = If this is nonzero (True), the value passed for Voltage should be
//         binary.
//Reserved (1&2) = Pass 0.

long eDI( HANDLE Handle,
          long ConfigIO,
          long Channel,
          long *State);
//An "easy" function that reads the state of one digital input.  This function
//does not automatically configure the specified channel as digital, unless
//ConfigIO is set as True.  Returns 0 for no error, or -1 or >0 value
//(low-level errorcode) on error.
//Handle = Handle to a U3 device.
//ConfigIO = If this is nonzero (True), then 2 ConfigIO low-level functions
//           calls will be made in addition to the 1 Feedback call to set
//           Channel as digital.  If this is 0 (False), then only a Feedback
//           low-level call will be made, and an error will be returned if
//           Channel is not already set as digital.
//Channel = The channel to read.  0-19 corresponds to FIO0-CIO3.
//          For U3 hardware versions 1.30, HV model, Channel needs to be 4-19,
//State = Returns the state of the digital input.  0=False=Low and 1=True=High.

long eDO( HANDLE Handle,
          long ConfigIO,
          long Channel,
          long State);
//An "easy" function that writes the state of one digital output.  This
//function does not automatically configure the specified channel as digital,
//unless ConfigIO is set as True.  Returns 0 for no error, or -1 or >0 value
//(low-level errorcode) on error.
//Handle = Handle to a U3 device.
//ConfigIO = If this is nonzero (True), then 2 ConfigIO low-level functions
//           calls will be made in addition to the 1 Feedback call to set
//           Channel as digital. If this is 0 (False), then only a Feedback
//           low-level call will be made, and an error will be returned if
//           Channel is not already set as digital.
//Channel = The channel to write to.  0-19 corresponds to FIO0-CIO3.
//          For U3 hardware versions 1.30, HV model, Channel needs to be 4-19,
//State = The state to write to the digital output.  0=False=Low and
//        1=True=High.

long eTCConfig( HANDLE Handle,
                long *aEnableTimers,
                long *aEnableCounters,
                long TCPinOffset,
                long TimerClockBaseIndex,
                long TimerClockDivisor,
                long *aTimerModes,
                double *aTimerValues,
                long Reserved1,
                long Reserved2);
//An "easy" function that configures and initializes all the timers and
//counters.  When needed, this function automatically configures the needed
//lines as digital.  Returns 0 for no error, or -1 or >0 value (low-level
//errorcode) on error.
//Handle = Handle to a U3 device.
//aEnableTimers = An array where each element specifies whether that timer is
//                enabled.  Timers must be enabled in order starting from 0, so
//                for instance, Timer1 cannot be enabled without enabling Timer
//                0 also.  A nonzero for an array element specifies to enable
//                that timer.  For the U3, this array must always have at least
//                2 elements.
//aEnableCounters = An array where each element specifies whether that counter
//                  is enabled.  Counters do not have to be enabled in order
//                  starting from 0, so Counter1 can be enabled when Counter0
//                  is disabled.  A nonzero value for an array element
//                  specifies to enable that counter.  For the U3, this array
//                  must always have at least 2 elements.
//TCPinOffset = Value from 0-8 specifies where to start assigning timers and
//              counters.
//              For U3 hardware versions 1.30, HV model, value needs to be 4-8.
//TimerClockBaseIndex = Pass a constant to set the timer base clock.  The
//                      default is LJ_tc48MHZ.
//TimerClockDivisor = Pass a divisor from 0-255 where 0 is a divisor of 256.
//aTimerModes = An array where each element is a constant specifying the mode
//              for that timer.  For the U3, this array must always have at
//              least 2 elements.
//aTimerValues = An array where each element specifies the initial value for
//               that timer.  For the U3, this array must always have at least
//               2 elements.
//Reserved (1&2) =  Pass 0.

long eTCValues( HANDLE Handle,
                long *aReadTimers,
                long *aUpdateResetTimers,
                long *aReadCounters,
                long *aResetCounters,
                double *aTimerValues,
                double *aCounterValues,
                long Reserved1,
                long Reserved2);
//An "easy" function that updates and reads all the timers and counters.
//Returns 0 for no error, or -1 or >0 value (low-level errorcode) on error.
//Handle = Handle to a U3 device.
//aReadTimers = An array where each element specifies whether to read that
//              timer.  A nonzero value for an array element specifies to read
//              that timer.  For the U3, this array must always have at least 2
//              elements.
//aUpdateResetTimers = An array where each element specifies whether to
//                     update/reset that timers.  A nonzero value for an array
//                     element specifies to update/reset that timer.  For the
//                     U3, this array must always have at least 2 elements.
//aReadCounters = An array where each element specifies whether to read that
//                counter.  A nonzero value for an array element specifies to
//                read that counter.  For the U3, this array must always have
//                at least 2 elements.
//aResetCounters = An array where each element specifies whether to reset that
//                 counter.  A nonzero value for an array element specifies to
//                 reset that counter.  For the U3, this array must always have
//                 at least 2 elements.
//aTimerValues = Input: An array where each element is the new value for that
//               timer.  Each value is only updated if the appropriate element
//               is set in the aUpdateResetTimers array.
//               Output: An array where each element is the value read from
//               that timer if the appropriate element is set in the
//               aReadTimers array.  If the timer mode set for the timer is
//               Quadrature Input, the value needs to be converted from an
//               unsigned 32-bit integer to a signed 32-bit integer (2’s
//               complement).  For the U3, this array must always have at least
//               2 elements.
//aCounterValues = An array where each element is the value read from that
//                 counter if the appropriate element is set in the aReadTimers
//                 array.  For the U3, this array must always have at least 2
//                 elements.
//Reserved (1&2) = Pass 0.


/* Easy Function Helpers */

long ehConfigIO( HANDLE hDevice,
                 uint8 inWriteMask,
                 uint8 inTimerCounterConfig,
                 uint8 inDAC1Enable,
                 uint8 inFIOAnalog,
                 uint8 inEIOAnalog,
                 uint8 *outTimerCounterConfig,
                 uint8 *outDAC1Enable,
                 uint8 *outFIOAnalog,
                 uint8 *outEIOAnalog);
//Used by the eAIN, eDAC, eDI, eDO and eTCConfig easy functions.  This function
//takes the ConfigIO low-level command and response bytes (not including
//checksum and command bytes) as its parameter and performs a ConfigIO call
//with the U3.  Returns -1 or errorcode (>1 value) on error, 0 on success.

long ehConfigTimerClock( HANDLE hDevice,
                         uint8 inTimerClockConfig,
                         uint8 inTimerClockDivisor,
                         uint8 *outTimerClockConfig,
                         uint8 *outTimerClockDivisor);
//Used by the eTCConfig easy function.  This function takes the
//ConfigTimerClock low-level command and response bytes (not including checksum
//and command bytes) as its parameter and performs a ConfigTimerClock call with
//the U3.  Returns -1 or errorcode (>1 value) on error, 0 on success.

long ehFeedback( HANDLE hDevice,
                 uint8 *inIOTypesDataBuff,
                 long inIOTypesDataSize,
                 uint8 *outErrorcode,
                 uint8 *outErrorFrame,
                 uint8 *outDataBuff,
                 long outDataSize);
//Used by the all of the easy functions.  This function takes the Feedback
//low-level command and response bytes (not including checksum and command
//bytes) as its parameter and performs a Feedback call with the U3.  Returns -1
//or errorcode (>1 value) on error, 0 on success.


/* Easy function constants */


/* Timer clocks for Hardware Version 1.20 or lower */

// 2 MHz
#define LJ_tc2MHZ 10

// 6 MHz
#define LJ_tc6MHZ 11

// 24 MHz
#define LJ_tc24MHZ 12

// 500/Divisor KHz
#define LJ_tc500KHZ_DIV 13

// 2/Divisor MHz
#define LJ_tc2MHZ_DIV 14

// 6/Divisor MHz
#define LJ_tc6MHZ_DIV 15

// 24/Divisor MHz
#define LJ_tc24MHZ_DIV 16


/* Timer clocks for Hardware Version 1.21 or higher */

// 4 MHz
#define LJ_tc4MHZ 20

// 12 MHz
#define LJ_tc12MHZ 21

// 48 MHz
#define LJ_tc48MHZ 22

// 1/Divisor MHz
#define LJ_tc1MHZ_DIV 23

// 4/Divisor MHz
#define LJ_tc4MHZ_DIV 24

// 12/Divisor MHz
#define LJ_tc12MHZ_DIV 25

// 48/Divisor MHz
#define LJ_tc48MHZ_DIV 26


/* Timer modes */

// 16 bit PWM
#define LJ_tmPWM16 0

// 8 bit PWM
#define LJ_tmPWM8 1

// 32-bit rising to rising edge measurement
#define LJ_tmRISINGEDGES32 2

// 32-bit falling to falling edge measurement
#define LJ_tmFALLINGEDGES32 3

// duty cycle measurement
#define LJ_tmDUTYCYCLE 4

// firmware based rising edge counter
#define LJ_tmFIRMCOUNTER 5

// firmware counter with debounce
#define LJ_tmFIRMCOUNTERDEBOUNCE 6

// frequency output
#define LJ_tmFREQOUT 7

// Quadrature
#define LJ_tmQUAD 8

// stops another timer after n pulses
#define LJ_tmTIMERSTOP 9

// read lower 32-bits of system timer
#define LJ_tmSYSTIMERLOW 10

// read upper 32-bits of system timer
#define LJ_tmSYSTIMERHIGH 11

// 16-bit rising to rising edge measurement
#define LJ_tmRISINGEDGES16 12

// 16-bit falling to falling edge measurement
#define LJ_tmFALLINGEDGES16 13

#endif
