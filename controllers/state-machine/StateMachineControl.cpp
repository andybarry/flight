#include "StateMachineControl.hpp"

StateMachineControl::StateMachineControl() : fsm_(*this) {


}

void StateMachineControl::ProcessImuMsg(const mav::pose_t &msg) {
    fsm_.ImuUpdate(msg);
}



int main(int argc,char** argv) {

    StateMachineControl fsm_control;

    mav::pose_t msg;

    msg.utime = 0;

    fsm_control.ProcessImuMsg(msg);

    msg.utime = 1;

    fsm_control.ProcessImuMsg(msg);


    return 0;
}
