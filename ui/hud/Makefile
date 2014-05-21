# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a

MAVLIB=../../../mav/mavconn/build/lib

CFLAGS=-c -Wall -O3 -std=c++0x `pkg-config --cflags lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames opencv` -I/$(LCMDIR) -I/usr/local/include/firefly-mv-utils -I/usr/local/include/dc1394 -I/usr/include/dc1394

LIBS=`pkg-config --libs lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames opencv` -ldc1394 -L /usr/local/lib/libdc1394.so -lgthread-2.0 ../../LCM/lib/libtypes.a /usr/local/lib/firefly-mv-utils/libutil.so -lboost_system -lboost_filesystem -ljpeg $(LCMLIB)

all: hud-main

hud-main: hud-main.o hud.o jpeg-utils.o opencv-stereo-util.o
	$(CC) hud-main.o hud.o jpeg-utils.o opencv-stereo-util.o -o hud-main $(LIBS)

hud-main.o: hud-main.cpp
	$(CC) $(CFLAGS) hud-main.cpp
	
hud.o: ../../ui/hud/hud.cpp ../../ui/hud/hud.hpp
	$(CC) $(CFLAGS) ../../ui/hud/hud.cpp
	
opencv-stereo-util.o: ../../sensors/stereo/opencv-stereo-util.cpp ../../sensors/stereo/opencv-stereo-util.hpp 
	$(CC) $(CFLAGS) ../../sensors/stereo/opencv-stereo-util.cpp
	
jpeg-utils.o: ../../externals/jpeg-utils/jpeg-utils.c ../../externals/jpeg-utils/jpeg-utils.h
	$(CC) $(CFLAGS) ../../externals/jpeg-utils/jpeg-utils.c

clean:
	rm -rf *o hud-main



