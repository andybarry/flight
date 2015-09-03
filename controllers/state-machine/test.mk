TARGET = test

SM_SOURCES = AircraftStateMachine.sm

SOURCES = $(SM_SOURCES:.sm=_sm.cpp) StateMachineControl.cpp ../tvlqr/TvlqrControl.cpp ../TrajectoryLibrary/TrajectoryLibrary.cpp ../TrajectoryLibrary/Trajectory.cpp ../../externals/csvparser/csvparser.c ../../utils/utils/RealtimeUtils.cpp ../../utils/ServoConverter/ServoConverter.cpp ../../estimators/StereoOctomap/StereoOctomap.cpp StateMachineTests.cpp ../../estimators/SpacialStereoFilter/SpacialStereoFilter.cpp

SMC = java -jar ../../externals/smc/bin/Smc.jar

# Uncomment to turn off IOStreams for debug.
# NO_STREAMS=     -nostreams

# Uncomment to see state machine debug output.
# FSM_DEBUG=      -DFSM_DEBUG

# Uncomment to turn on debug message generation.
TRACE=          -g $(NO_STREAMS)

# Uncomment to turn off try/catch/rethrow generation.
# NO_CATCH=       -nocatch

# Uncomment to turn off exception throws.
# NO_EXCEPT=      -noex

# Uncomment to turn on CRTP code and -crtp SMC flag.
# CRTP=           -crtp
# CRTP_FLAG=      -DCRTP

SMC_FLAGS = -c++ $(TRACE) $(NO_CATCH) $(NO_EXCEPT) $(CRTP) $(TRACE)

%_sm.h %_sm.cpp : %.sm
		$(SMC) $(SMC_FLAGS) $<

%_sm.dot :      %.sm
		$(SMC) -graph -glevel 1 $<

%_sm.png :      %_sm.dot
		dot -T png -o $@ $<

%_sm.pdf :      %_sm.dot
		dot -T pdf -o $@ $<

graph :         $(SM_SOURCES:%.sm=%_sm.dot)

png :           $(SM_SOURCES:%.sm=%_sm.png)

pdf :           $(SM_SOURCES:%.sm=%_sm.pdf)

clean : smcclean

include ../../utils/make/flight.mk

RM_F=           rm -f

smcclean :
		-$(RM_F) *_sm.h
		-$(RM_F) *_sm.cpp
		-$(RM_F) *_sm.dot
		-$(RM_F) *_sm.png
		-$(RM_F) *_sm.pdf
		-$(RM_F) *_sm.html
