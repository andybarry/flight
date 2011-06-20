#ifndef RADIO_COMM_H_
#define RADIO_COMM_H_


/*device name*/
//#define MODEMDEVICE "/dev/ttyUSB0"
#define MODEMDEVICE "/dev/ttyF1"

/*baudrate: must be 19200 for PC Buddy*/
#define BAUDRATE B19200 

int radio_comm();

#endif /* RADIO_COMM_H_ */
