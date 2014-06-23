/*
 * Sends an LCM message to init the state estimator.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2014
 *
 */

#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/mav/filter_state_t.hpp" // from libbot for images over LCM

#include "../../externals/ConciseArgs.hpp"
#include "lcmtypes/mav/ins_t.hpp"

#include <bot_core/bot_core.h>

#include <sys/time.h>

using namespace std;

mav::ins_t *ins_msg = NULL;

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}

class InsHandler 
{
    public:
        ~InsHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const mav::ins_t* msg) {
                    
            ins_msg = new mav::ins_t(*msg);
        }
};


void ComputeQuatFromAttitude(double *accel, double *quat) {
    cout << accel[0] << endl;
    cout << accel[1] << endl;
    cout << accel[2] << endl;
    
    double my_accel[3];
    my_accel[0] = 0;//accel[0] / 9.81;
    my_accel[1] = 0;//accel[1] / 9.81;
    my_accel[2] = -1.1;//accel[2] / 9.81;
    
    double g_vec[3] = { 0.0, 0.0, -1 };
    
    bot_vector_normalize_3d(my_accel);
    bot_vector_normalize_3d(g_vec);
    
    double theta = bot_vector_dot_3d(my_accel, g_vec);
    double axis[3];
    
    bot_vector_cross_3d(my_accel, g_vec, axis);
    
    bot_vector_normalize_3d(axis);
    
    // convert to quat
    
    bot_angle_axis_to_quat(theta, axis, quat);
    
}

int main(int argc,char** argv) {
    
    bool origin = false;
    
    ConciseArgs parser(argc, argv);
    parser.add(origin, "o", "init-origin", "Don't attempt to init based on acclerometer data, just use the origin.");
    parser.parse();
    
    lcm::LCM lcm;
    
    if (!lcm.good()) {
        cerr << "LCM init failed. " << endl;
        return 1;
    }
    
    mav::filter_state_t msg;
    
    
    if (origin == false) {
        // wait for a "attitude" data packet
        InsHandler handler;
        lcm.subscribe("attitude", &InsHandler::handleMessage, &handler);
        
        cout << "Waiting for attitude message..." << endl;
        while (ins_msg == NULL) {
            lcm.handle();
        }
        
        cout << "Got attitude message." << endl;
        
        // compute quat from acceleration vector assuming we are not moving but may
        // be on a slope
        
        ComputeQuatFromAttitude(ins_msg->accel, msg.quat);
        
        
    } else {
        msg.quat[0] = 1;
        msg.quat[1] = 0;
        msg.quat[2] = 0;
        msg.quat[3] = 0;
    }
    
    
    
    
    msg.utime = 0;//getTimestampNow();

    
    
    msg.num_states = 21;
    msg.state.resize(msg.num_states);
    
    for (int i = 0; i < msg.num_states; i++) {
        msg.state[i] = 0;
    }
    
    msg.num_cov_elements = 441;
    msg.cov.resize(msg.num_cov_elements);
    
    for (int i = 0; i < msg.num_cov_elements; i++) {
        msg.cov[i] = 0;
    }
    
    msg.cov[66] = 0.0225;
    msg.cov[88] = 0.0225;
    msg.cov[110] = 0.025;
    msg.cov[132] = 0.00274155677;
    msg.cov[154] = 0.00274155677;
    msg.cov[176] = 0.00274155677;
    msg.cov[198] = 0.25;
    msg.cov[220] = 0.25;
    
    lcm.publish("MAV_STATE_EST_INITIALIZER", &msg);
    
    cout << "Init message sent." << endl;

    return 0;
}

