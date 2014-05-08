# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a

MAVLIB=../../../mav/mavconn/build/lib

CFLAGS=-c -Wall -O3 -std=c++0x `pkg-config --cflags lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames opencv` -I/$(LCMDIR)

LIBS=`pkg-config --libs lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames opencv` -ljpeg $(LCMLIB)




all: plane-visualizer

plane-visualizer: plane-visualizer.o hud.o jpeg-utils.o
	$(CC) plane-visualizer.o hud.o jpeg-utils.o -o plane-visualizer $(LIBS)

plane-visualizer.o: plane-visualizer.cpp
	$(CC) $(CFLAGS) plane-visualizer.cpp
	
hud.o: ../../ui/hud/hud.cpp ../../ui/hud/hud.hpp
	$(CC) $(CFLAGS) ../../ui/hud/hud.cpp
	
jpeg-utils.o: ../../externals/jpeg-utils/jpeg-utils.c ../../externals/jpeg-utils/jpeg-utils.h
	$(CC) $(CFLAGS) ../../externals/jpeg-utils/jpeg-utils.c

clean:
	rm -rf *o plane-visualizer



