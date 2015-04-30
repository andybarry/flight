TARGET = opencv-cam-calib-test
SOURCES = opencv-cam-calib-test.cpp opencv-stereo-util.cpp ../../externals/jpeg-utils/jpeg-utils.c RecordingManager.cpp

LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a
OCTOMAPLIB=../../../Fixie/build/lib/liboctomap.a ../../../Fixie/build/lib/liboctomath.a ../../../Fixie/build/lib/liblcmtypes_octomap-utils.a

MAVLIB=../../../mav/mavconn/build/lib

CXXFLAGS=-std=c++0x

CPPFLAGS=-Wall -I/usr/local/include/opencv2 `pkg-config --cflags opencv lcm bot2-core bot2-param-client bot2-lcmgl-client bot2-frames  eigen3` -I/$(LCMDIR) -I/usr/local/include/firefly-mv-utils -I/usr/local/include/dc1394 -I/usr/include/dc1394 -I../../../Fixie/build/include

LDPOSTFLAGS=-fopenmp `pkg-config --libs opencv` -L/usr/local/lib -ldc1394 -lglib-2.0 -L /usr/local/lib -L /usr/local/lib/libdc1394.so -L /usr/local/lib/liblcm.so -lgthread-2.0 -lrt -lglib-2.0 ../../LCM/lib/libtypes.a /usr/local/lib/firefly-mv-utils/libutil.so -pthread -lboost_system -lboost_filesystem -Wl,-rpath -Wl,/usr/local/lib/firefly-mv-utils `pkg-config --libs bot2-lcmgl-client bot2-core libavcodec` -ljpeg


include ../../externals/edam.mk
