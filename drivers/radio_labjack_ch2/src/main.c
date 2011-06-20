#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <aio.h> 
#include <signal.h>
#include <pthread.h>
#include <termios.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "globalvar.h"
#include "lcm_i.h"

void *lcm_thread_handler();
pthread_t lcm_thread;

int main()
{

global_init();
lcm_publish_init();

printf("connecting to labjack ...\n");

pthread_mutex_init(&mutex,NULL);

pthread_create( &lcm_thread, NULL, lcm_thread_handler,NULL);
	
radio_labjack();

     	pthread_join( lcm_thread, NULL);

	pthread_mutex_destroy(&mutex);

     	exit(0);

	return 0;

}

void *lcm_thread_handler()
{

	lcm_get_servo();

	return 0;

}

void lock(void) 
{
	pthread_mutex_lock(&mutex);
}

void unlock(void) 
{
	pthread_mutex_unlock(&mutex);
}
