# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

TRAJPATH=../../controllers/cpp_stereo_obstacles

CFLAGS=-c -Wall -g --std=c++0x `pkg-config --cflags lcm bot2-core bot2-param-client bot2-frames bot2-lcmgl-client opencv` -I/$(LCMDIR) -I../../mavlink-generated -I../../../Fixie/build/include/lcmtypes

LIBS=`pkg-config --libs lcm bot2-core bot2-param-client bot2-frames bot2-lcmgl-client opencv` $(LCMLIB)




all: stereo-type-converter

stereo-type-converter: stereo-type-converter.o Trajectory.o
	$(CC) stereo-type-converter.o Trajectory.o -o stereo-type-converter $(LIBS)

stereo-type-converter.o: stereo-type-converter.cpp
	$(CC) $(CFLAGS) stereo-type-converter.cpp

Trajectory.o: $(TRAJPATH)/Trajectory.cpp
	$(CC) $(CFLAGS) $(TRAJPATH)/Trajectory.cpp


clean:
	rm -rf *o stereo-type-converter



