# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++

# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../Fixie/build/lib/liblcmtypes_mav-lcmtypes.a

CFLAGS=-c -Wall -O3 -std=c++0x `pkg-config --cflags lcm` -I/$(LCMDIR) -I../../mavlink-generated -I../../../Fixie/build/include/lcmtypes

LIBS=`pkg-config --libs lcm` $(LCMLIB) $(MAVLCMLIB)




all: wind-estimator

wind-estimator: wind-estimator.o
	$(CC) wind-estimator.o -o wind-estimator $(LIBS)

wind-estimator.o: wind-estimator.cpp
	$(CC) $(CFLAGS) wind-estimator.cpp

clean:
	rm -rf *o wind-estimator



