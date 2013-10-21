# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a

MAVLIB=../../../mav/mavconn/build/lib

CFLAGS=-c -Wall -O3 `pkg-config --cflags lcm bot2-core bot2-param-client bot2-lcmgl-client` -I/$(LCMDIR)

LIBS=`pkg-config --libs lcm bot2-core bot2-param-client bot2-lcmgl-client` $(LCMLIB)




all: obstacles-lcmgl

obstacles-lcmgl: obstacles-lcmgl.o
	$(CC) obstacles-lcmgl.o -o obstacles-lcmgl $(LIBS)

obstacles-lcmgl.o: obstacles-lcmgl.cpp
	$(CC) $(CFLAGS) obstacles-lcmgl.cpp

clean:
	rm -rf *o obstacles-lcmgl



