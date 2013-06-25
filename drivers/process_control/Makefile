# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

#MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a

#MAVLIB=../../../mav/mavconn/build/lib
#MAVLIB=/home/abarry/mav/mavconn/build/lib

CFLAGS=-c -Wall -g -O0 -std=c++0x `pkg-config --cflags lcm bot2-core bot2-param-client glib-2.0` -I/$(LCMDIR) -I../../mavlink-rlg -I../../../Fixie/build/include/lcmtypes -I../../../mav/mavlink/build/include/v1.0/ -I../../../mav/mavconn/src/

LIBS=`pkg-config --libs lcm bot2-core bot2-param-client` -lutil $(LCMLIB)

all: process-control

process-control: ProcessControlProc.o process-control.o 
	$(CC) process-control.o ProcessControlProc.o -o process-control $(LIBS)

process-control.o: process-control.cpp
	$(CC) $(CFLAGS) process-control.cpp

ProcessControlProc.o: ProcessControlProc.cpp
	$(CC) $(CFLAGS) ProcessControlProc.cpp

clean:
	rm -rf *o process-control



