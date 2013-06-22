#include "LcmTransportPart.hpp"

LcmTransportPart::LcmTransportPart()
{
    channelNameLen = -1;
    receivedSoFar = 0;
}

LcmTransportPart::LcmTransportPart(mavlink_lcm_transport_t firstMessage)
{
    channelNameLen = -1;
    receivedSoFar = 0;
    
    AddMessage(firstMessage);
}



void LcmTransportPart::AddMessage(mavlink_lcm_transport_t mavmsg)
{
    id = mavmsg.msg_id;
    numTotal = mavmsg.message_part_total;
    
    receivedSoFar ++;
    
    // copy the data in from this message
    
    
    char *thisData = new char[MAVLINK_LCM_PAYLOAD_SIZE];
    
    int nameOffset = 0;
    
    // if this is the first message, it will contain the channel name
    if (mavmsg.message_part_counter == 0)
    {
        // since the first data is the string, we can just read a string off which will terminate
        // at the \0 and then we can ask the string how long it is
        channelName = mavmsg.payload;
        
        channelNameLen = channelName.length() + 1; // + 1 for the \0 at the end of the string
        channelName += "-recv";
        
        nameOffset = channelNameLen;
    }
    
    memcpy(thisData, mavmsg.payload + nameOffset, mavmsg.payload_size);
    
    dataArray[mavmsg.message_part_counter] = thisData;
    sizeArray[mavmsg.message_part_counter] = mavmsg.payload_size;
        
    totalDataSizeSoFar += mavmsg.payload_size;
    
    
}

bool LcmTransportPart::DoComplete()
{
    //cout << "do complete (" << receivedSoFar << "/" << numTotal << ")" << endl;
    
    // check to see if we have received all of the parts of the message
    if (receivedSoFar != numTotal)
    {
        return false;
    }
    
    int dataLoc = 0;
    
    // we have received everything, so build the data array
    for (int i=0;i<numTotal;i++)
    {
        memcpy(data + dataLoc, dataArray[i], sizeArray[i]);
        dataLoc += sizeArray[i];
    }

    return true;
}

LcmTransportPart::~LcmTransportPart()
{
    // zap all the char arrays
    for (int i=0; i<numTotal; i++)
    {
        delete dataArray[i];
    }
}
