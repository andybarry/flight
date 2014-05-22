# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a
OCTOMAPLIB=../../../Fixie/build/lib/liboctomap.a ../../../Fixie/build/lib/liboctomath.a ../../../Fixie/build/lib/liblcmtypes_octomap-utils.a

MAVLIB=../../../mav/mavconn/build/lib

CFLAGS=-c -Wall -O3 -std=c++0x -I/usr/local/include/opencv2 `pkg-config --cflags opencv lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames  eigen3` -I/$(LCMDIR) -I/usr/local/include/firefly-mv-utils -I/usr/local/include/dc1394 -I/usr/include/dc1394 -I../../../Fixie/build/include

LIBS=`pkg-config --libs opencv lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames eigen3` -ldc1394 -L /usr/local/lib/libdc1394.so -lgthread-2.0 ../../LCM/lib/libtypes.a /usr/local/lib/firefly-mv-utils/libutil.so -lboost_system -lboost_filesystem -ljpeg $(LCMLIB) $(MAVLCMLIB) $(OCTOMAPLIB)

all: hud-main

hud-main: hud-main.o hud.o jpeg-utils.o opencv-stereo-util.o
	$(CC) hud-main.o hud.o jpeg-utils.o opencv-stereo-util.o -o hud-main $(LIBS)

hud-main.o: hud-main.cpp hud-main.hpp
	$(CC) $(CFLAGS) hud-main.cpp
	
hud.o: hud.cpp hud.hpp
	$(CC) $(CFLAGS) ../../ui/hud/hud.cpp
	
opencv-stereo-util.o: ../../sensors/stereo/opencv-stereo-util.cpp ../../sensors/stereo/opencv-stereo-util.hpp 
	$(CC) $(CFLAGS) ../../sensors/stereo/opencv-stereo-util.cpp
	
jpeg-utils.o: ../../externals/jpeg-utils/jpeg-utils.c ../../externals/jpeg-utils/jpeg-utils.h
	$(CC) $(CFLAGS) ../../externals/jpeg-utils/jpeg-utils.c

clean:
	rm -rf *o hud-main



