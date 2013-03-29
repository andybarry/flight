# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a

MAVLIB=/home/linaro/mav/mavconn/build/lib

CFLAGS=-c -Wall -O3 -I../../mavlink-rlg `pkg-config --cflags lcm bot2-core bot2-param-client` -I/$(LCMDIR) -I../../../Fixie/build/include/lcmtypes

LIBS=`pkg-config --libs lcm bot2-core bot2-param-client` $(LCMLIB) $(MAVLCMLIB)




all: xbee-to-lcm-bridge

xbee-to-lcm-bridge: xbee-to-lcm-bridge.o
	$(CC) xbee-to-lcm-bridge.o -o xbee-to-lcm-bridge $(LIBS)

xbee-to-lcm-bridge.o: xbee-to-lcm-bridge.cpp
	$(CC) $(CFLAGS) xbee-to-lcm-bridge.cpp

clean:
	rm -rf *o xbee-to-lcm-bridge



