TARGET = test

SOURCES = tvlqr-controller.cpp tests.cpp TvlqrControl.cpp ../TrajectoryLibrary/TrajectoryLibrary.cpp ../TrajectoryLibrary/Trajectory.cpp ../../externals/csvparser/csvparser.c ../../utils/utils/RealtimeUtils.cpp  ../../utils/ServoConverter/ServoConverter.cpp ../../estimators/StereoOctomap/StereoOctomap.cpp ../../estimators/StereoFilter/StereoFilter.cpp


include ../../utils/make/flight.mk
