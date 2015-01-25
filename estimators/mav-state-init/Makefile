TARGET = send-mav-state-init
SOURCES = send-mav-state-init.cpp

LCMDIR=../../LCM/
LCMLIB=../../LCM/lib/libtypes.a

MAVLCMLIB=../../../pronto-distro/build/lib/liblcmtypes_mav-lcmtypes.a


CXXFLAGS=-std=c++0x

CPPFLAGS=-Wall -I/$(LCMDIR) -I../../../pronto-distro/build/include `pkg-config --cflags lcm bot2-core`

LDPOSTFLAGS=-fopenmp -L/usr/local/lib -L /usr/local/lib `pkg-config --libs lcm bot2-core` ../../LCM/lib/libtypes.a


include ../../externals/edam.mk
