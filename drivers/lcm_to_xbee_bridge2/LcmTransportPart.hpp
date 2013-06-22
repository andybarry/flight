#ifndef LCM_TRANSPORT_PART_H
#define LCM_TRANSPORT_PART_H


#include <string>
#include <iostream>
#include <vector>
#include "../../mavlink-rlg/csailrlg/mavlink.h"


using namespace std;

#define MAX_MESSAGE_PARTS 255
#define MAVLINK_LCM_PAYLOAD_SIZE 124

class LcmTransportPart {
    public: 
        LcmTransportPart();
        LcmTransportPart(mavlink_lcm_transport_t firstMessage);
        ~LcmTransportPart();
    
        std::string channelName;
        int id;
        int numTotal;
        
        int receivedSoFar;
        char data[MAX_MESSAGE_PARTS*MAVLINK_LCM_PAYLOAD_SIZE];
        
        int totalDataSizeSoFar;
        
        
        
        void AddMessage(mavlink_lcm_transport_t mavmsg);
        
        bool DoComplete();
        
    private:
        int channelNameLen;
        char* dataArray[MAX_MESSAGE_PARTS];
        int sizeArray[MAX_MESSAGE_PARTS];
};

#endif
