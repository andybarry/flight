# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a

CFLAGS=-c -Wall -O3 -std=c++0x `pkg-config --cflags lcm bot2-core bot2-param-client` -I/$(LCMDIR) -I../../mavlink-generated -I../../../Fixie/build/include/lcmtypes -I../../../mav/mavlink/build/include/v1.0/ -I../../../mav/mavconn/src/

LIBS=`pkg-config --libs lcm bot2-core bot2-param-client` $(LCMLIB) $(MAVLCMLIB)




all: ground-sound

ground-sound: ground-sound.o
	$(CC) ground-sound.o -o ground-sound $(LIBS)

ground-sound.o: ground-sound.cpp
	$(CC) $(CFLAGS) ground-sound.cpp

clean:
	rm -rf *o ground-sound



