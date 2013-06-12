#include "LcmTransportPart.hpp"

LcmTransportPart::LcmTransportPart(mavlink_lcm_transport_t firstMessage)
{
    AddMessage(firstMessage);
}

void LcmTransportPart::AddMessage(mavlink_lcm_transport_t mavmsg)
{
    channelName = mavmsg.channel_name;
    id = mavmsg.msg_id;
    numTotal = mavmsg.message_part_total;
    
    recievedSoFar[mavmsg.message_part_counter] = true;
    // copy the data in from this message
    memcpy(data + mavmsg.message_part_counter*MAVLINK_LCM_PAYLOAD_SIZE, mavmsg.payload, mavmsg.payload_size);
    
    totalDataSizeSoFar += mavmsg.payload_size;
}

bool LcmTransportPart::IsComplete()
{
    // check to see if we have received all of the parts of the message
    for (int i=0;i<numTotal;i++)
    {
        if (recievedSoFar[i] == false)
        {
            return false;
        }
    }
    return true;
}

