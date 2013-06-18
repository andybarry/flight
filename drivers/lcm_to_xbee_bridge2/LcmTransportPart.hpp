#ifndef LCM_TRANSPORT_PART_H
#define LCM_TRANSPORT_PART_H


#include <string>
#include <iostream>
#include "../../mavlink-rlg/csailrlg/mavlink.h"


using namespace std;

#define MAX_MESSAGE_PARTS 255
#define MAVLINK_LCM_PAYLOAD_SIZE 62

class LcmTransportPart {
    public:
        std::string channelName;
        int id;
        int numTotal;
        bool recievedSoFar[MAX_MESSAGE_PARTS];
        char data[MAX_MESSAGE_PARTS*MAVLINK_LCM_PAYLOAD_SIZE];
        int totalDataSizeSoFar;
        
        LcmTransportPart() { };
        LcmTransportPart(mavlink_lcm_transport_t firstMessage);
        
        void AddMessage(mavlink_lcm_transport_t mavmsg);
        
        bool IsComplete();
};

#endif
