# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS=-c -Wall -O3 -mfpu=neon -fopenmp `pkg-config --cflags opencv` -I/usr/local/include   -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include   -pthread -I/usr/local/include -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include -I/usr/local/include/firefly-mv-utils -I/usr/local/include/dc1394 

LIBS=-fopenmp `pkg-config --libs opencv` -L/usr/local/lib -ldc1394   -lglib-2.0 -L /usr/local/lib -L /usr/local/lib/libdc1394.so -L /usr/local/lib/liblcm.so -lgthread-2.0 -lrt -lglib-2.0 ../../LCM/lib/libtypes.a /usr/local/lib/firefly-mv-utils/libutil.so /usr/local/lib/firefly-mv-utils/libopencvutil.so -pthread -Wl,-rpath -Wl,/usr/local/lib/firefly-mv-utils `pkg-config --libs bot2-lcmgl-client`

all: opencv-stereo opencv-calibrate

opencv-stereo: opencv-stereo.o barrymoore.o
	$(CC) opencv-stereo.o barrymoore.o -o opencv-stereo $(LIBS)
	
barrymoore.o: barrymoore.cpp
	$(CC) $(CFLAGS) barrymoore.cpp

opencv-stereo.o: opencv-stereo.cpp
	$(CC) $(CFLAGS) opencv-stereo.cpp
	
opencv-calibrate: opencv-calibrate.o
	$(CC) opencv-calibrate.o -o opencv-calibrate $(LIBS)
    
opencv-calibrate.o:
	$(CC) $(CFLAGS) opencv-calibrate.cpp
    

clean:
	rm -rf *o opencv-stereo opencv-calibrate


