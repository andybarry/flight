TARGET = opencv-calibrate
SOURCES = opencv-calibrate.cpp opencv-stereo-util.cpp ../../externals/jpeg-utils/jpeg-utils.c ../../utils/utils/RealtimeUtils.cpp

# include a standard makefile that uses these variables and builds everything
include ../../utils/make/flight.mk
