

#include <iostream>


using namespace std;

#include <lcm/lcm-cpp.hpp>
#include "../trajectory_t.hpp"

int main(int argc,char** argv) {

    lcm::LCM* lcm = new lcm::LCM();

    trajectory_t msg;


    lcm->publish("test", &msg);



    return 0;
}
