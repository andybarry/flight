# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS=-c -Wall -O3 -std=c++0x -fopenmp `pkg-config --cflags opencv bot2-lcmgl-client bot2-core` -I/usr/local/include   -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include   -pthread -I/usr/local/include -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include -I/usr/local/include/firefly-mv-utils -I/usr/local/include/dc1394 -I/usr/include/dc1394

LIBS=-fopenmp `pkg-config --libs opencv` -L/usr/local/lib -ldc1394   -lglib-2.0 -L /usr/local/lib -L /usr/local/lib/libdc1394.so -L /usr/local/lib/liblcm.so -lgthread-2.0 -lrt -lglib-2.0 ../../LCM/lib/libtypes.a /usr/local/lib/firefly-mv-utils/libutil.so -pthread -lboost_system -lboost_filesystem -Wl,-rpath -Wl,/usr/local/lib/firefly-mv-utils `pkg-config --libs bot2-lcmgl-client bot2-core libavcodec` -ljpeg

all: opencv-stereo opencv-calibrate opencv-cam-calib-test

opencv-stereo: opencv-stereo.o barrymoore.o opencv-stereo-util.o hud.o RecordingManager.o jpeg-utils.o
	$(CC) opencv-stereo.o barrymoore.o opencv-stereo-util.o hud.o RecordingManager.o jpeg-utils.o -o opencv-stereo $(LIBS)
	
barrymoore.o: barrymoore.cpp barrymoore.hpp
	$(CC) $(CFLAGS) barrymoore.cpp
	
opencv-stereo-util.o: opencv-stereo-util.cpp opencv-stereo-util.hpp 
	$(CC) $(CFLAGS) opencv-stereo-util.cpp
	
RecordingManager.o: RecordingManager.cpp RecordingManager.hpp opencv-stereo-util.o
	$(CC) $(CFLAGS) RecordingManager.cpp
	
hud.o: hud.cpp hud.hpp
	$(CC) $(CFLAGS) hud.cpp

opencv-stereo.o: opencv-stereo.hpp opencv-stereo.cpp opencv-stereo-util.hpp barrymoore.hpp RecordingManager.hpp
	$(CC) $(CFLAGS) opencv-stereo.cpp
	
opencv-calibrate: opencv-calibrate.o opencv-stereo-util.o jpeg-utils.o
	$(CC) opencv-calibrate.o opencv-stereo-util.o jpeg-utils.o -o opencv-calibrate $(LIBS)
    
opencv-calibrate.o: opencv-calibrate.cpp
	$(CC) $(CFLAGS) opencv-calibrate.cpp
	
opencv-cam-calib-test: opencv-cam-calib-test.o opencv-stereo-util.o jpeg-utils.o
	$(CC) opencv-cam-calib-test.o opencv-stereo-util.o -o opencv-cam-calib-test jpeg-utils.o $(LIBS)
    
jpeg-utils.o: ../../externals/jpeg-utils/jpeg-utils.c ../../externals/jpeg-utils/jpeg-utils.h
	$(CC) $(CFLAGS) ../../externals/jpeg-utils/jpeg-utils.c
	
	
opencv-cam-calib-test.o: opencv-cam-calib-test.cpp
	$(CC) $(CFLAGS) opencv-cam-calib-test.cpp
    
clean:
	rm -rf *o opencv-stereo opencv-calibrate


