# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

CFLAGS=-c -Wall -O3 -mfpu=neon -std=c++0x `pkg-config --cflags lcm eigen3` -I/$(LCMDIR) -I../../mavlink-generated

LIBS=`pkg-config --libs lcm` $(LCMLIB)




all: stereo-imu-obstacles

stereo-imu-obstacles: stereo-imu-obstacles.o
	$(CC) stereo-imu-obstacles.o -o stereo-imu-obstacles $(LIBS)

stereo-imu-obstacles.o: stereo-imu-obstacles.cpp
	$(CC) $(CFLAGS) stereo-imu-obstacles.cpp

clean:
	rm -rf *o stereo-imu-obstacles



