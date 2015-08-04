#ifndef STATE_MACHINE_CONTROL_HPP
#define STATE_MACHINE_CONTROL_HPP

/*
 * Implements a state machine controller for the aircraft
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include <iostream>
#include "../../LCM/mav/pose_t.hpp"
#include "AircraftStateMachine_sm.h"

class StateMachineControl {

    public:
        StateMachineControl();

        void DoControl(const mav::pose_t &msg) { std::cout << "DoControl(" << msg.utime << ")" << std::endl; }

        bool EvalTakeoff(const mav::pose_t &msg) { return false; }
        bool AtCrusingAltitude(const mav::pose_t &msg) { return true; }
        bool ReadyForThrottle(const mav::pose_t &msg) { return true; }

        void ProcessImuMsg(const mav::pose_t &msg);

    private:

        void MavPoseHandler();
        AircraftStateMachineContext fsm_;



};

#endif
