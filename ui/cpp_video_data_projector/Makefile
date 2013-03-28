# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

TRAJPATH=../../controllers/cpp_stereo_obstacles

CFLAGS=-c -Wall -O3 --std=c++0x `pkg-config --cflags lcm bot2-core bot2-param-client bot2-frames bot2-lcmgl-client opencv` -I/$(LCMDIR) -I../../mavlink-generated -I../../../Fixie/build/include/lcmtypes

LIBS=`pkg-config --libs lcm bot2-core bot2-param-client bot2-frames bot2-lcmgl-client opencv` $(LCMLIB)




all: video-data-projector

video-data-projector: video-data-projector.o Trajectory.o
	$(CC) video-data-projector.o Trajectory.o -o video-data-projector $(LIBS)

video-data-projector.o: video-data-projector.cpp
	$(CC) $(CFLAGS) video-data-projector.cpp

Trajectory.o: $(TRAJPATH)/Trajectory.cpp
	$(CC) $(CFLAGS) $(TRAJPATH)/Trajectory.cpp


clean:
	rm -rf *o video-data-projector



