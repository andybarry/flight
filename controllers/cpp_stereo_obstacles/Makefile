TARGET = stereo-imu-obstacles
SOURCES = stereo-imu-obstacles.cpp ../../sensors/stereo/opencv-stereo-util.cpp TrajectoryLibrary.cpp ../../estimators/StereoOctomap/StereoOctomap.cpp StereoFilter.cpp Trajectory.cpp ../../externals/jpeg-utils/jpeg-utils.c

LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a
OCTOMAPLIB=../../../Fixie/build/lib/liboctomap.a ../../../Fixie/build/lib/liboctomath.a ../../../Fixie/build/lib/liblcmtypes_octomap-utils.a

MAVLIB=../../../mav/mavconn/build/lib

CXXFLAGS=-std=c++0x

CPPFLAGS=-c -Wall -O3 -I/usr/local/include/opencv2 `pkg-config --cflags opencv lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames  eigen3` -I/$(LCMDIR) -I/usr/local/include/firefly-mv-utils -I/usr/local/include/dc1394 -I/usr/include/dc1394 -I../../../Fixie/build/include

LDPOSTFLAGS = `pkg-config --libs opencv lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames eigen3` -ldc1394 -L /usr/local/lib/libdc1394.so -lgthread-2.0 ../../LCM/lib/libtypes.a /usr/local/lib/firefly-mv-utils/libutil.so -lboost_system -lboost_filesystem -ljpeg $(LCMLIB) $(MAVLCMLIB) $(OCTOMAPLIB)


include ../../externals/edam.mk
