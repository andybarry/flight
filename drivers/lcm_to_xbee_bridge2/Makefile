# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a

#MAVLIB=/home/linaro/mav/mavconn/build/lib
MAVLIB=/home/abarry/mav/mavconn/build/lib

CFLAGS=-c -Wall -O3 -std=c++0x `pkg-config --cflags lcm bot2-core bot2-param-client glib-2.0` -I/$(LCMDIR) -I../../mavlink-rlg -I../../../Fixie/build/include/lcmtypes -I../../../mav/mavlink/build/include/v1.0/ -I../../../mav/mavconn/src/

LIBS=`pkg-config --libs lcm bot2-core bot2-param-client` $(LCMLIB) $(MAVLCMLIB) $(MAVLIB)/libmavconn_lcm.so -Wl,-rpath -Wl,$(MAVLIB)

all: lcm-to-xbee-bridge2

lcm-to-xbee-bridge2: lcm-to-xbee-bridge2.o LcmTransportPart.o
	$(CC) lcm-to-xbee-bridge2.o LcmTransportPart.o -o lcm-to-xbee-bridge2 $(LIBS)

lcm-to-xbee-bridge2.o: lcm-to-xbee-bridge2.cpp
	$(CC) $(CFLAGS) lcm-to-xbee-bridge2.cpp

LcmTrasportPart.o: LcmTransportPart.cpp
	$(CC) $(CFLAGS) LcmTransportPart.cpp

clean:
	rm -rf *o lcm-to-xbee-bridge2



