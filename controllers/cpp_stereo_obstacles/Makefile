# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a
OCTOMAPLIB=../../../Fixie/build/lib/liboctomap.a ../../../Fixie/build/lib/liboctomath.a ../../../Fixie/build/lib/liblcmtypes_octomap-utils.a
 

CFLAGS=-c -Wall -O3 -std=c++0x `pkg-config --cflags lcm eigen3 opencv bot2-core bot2-frames bot2-param-client` -I/$(LCMDIR) -I../../mavlink-generated -I../../../Fixie/build/include

LIBS=`pkg-config --libs lcm opencv bot2-core bot2-frames bot2-param-client` $(LCMLIB) -pthread -Wl,-rpath -Wl,/usr/local/lib `pkg-config --libs bot2-lcmgl-client` $(MAVLCMLIB) $(OCTOMAPLIB)




all: stereo-imu-obstacles

stereo-imu-obstacles: stereo-imu-obstacles.o TrajectoryLibrary.o Trajectory.o StereoOctomap.o StereoFilter.o
	$(CC) stereo-imu-obstacles.o TrajectoryLibrary.o  Trajectory.o StereoOctomap.o StereoFilter.o -o stereo-imu-obstacles $(LIBS)

stereo-imu-obstacles.o: stereo-imu-obstacles.cpp stereo-imu-obstacles.hpp
	$(CC) $(CFLAGS) stereo-imu-obstacles.cpp
	
TrajectoryLibrary.o: TrajectoryLibrary.cpp TrajectoryLibrary.hpp
	$(CC) $(CFLAGS) TrajectoryLibrary.cpp

Trajectory.o: Trajectory.cpp Trajectory.hpp
	$(CC) $(CFLAGS) Trajectory.cpp
	
StereoOctomap.o: ../../estimators/StereoOctomap/StereoOctomap.cpp ../../estimators/StereoOctomap/StereoOctomap.hpp
	$(CC) $(CFLAGS) ../../estimators/StereoOctomap/StereoOctomap.cpp
	
StereoFilter.o: StereoFilter.cpp StereoFilter.hpp
	$(CC) $(CFLAGS) StereoFilter.cpp
	
clean:
	rm -rf *o stereo-imu-obstacles



