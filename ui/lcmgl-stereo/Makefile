# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a

MAVLIB=../../../mav/mavconn/build/lib

CFLAGS=-c -Wall -O3 `pkg-config --cflags lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames` -I/$(LCMDIR)

LIBS=`pkg-config --libs lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames` $(LCMLIB)




all: stereo-lcmgl

stereo-lcmgl: stereo-lcmgl.o
	$(CC) stereo-lcmgl.o -o stereo-lcmgl $(LIBS)

stereo-lcmgl.o: stereo-lcmgl.cpp
	$(CC) $(CFLAGS) stereo-lcmgl.cpp

clean:
	rm -rf *o stereo-lcmgl



