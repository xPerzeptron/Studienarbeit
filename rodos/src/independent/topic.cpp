/**
 * @file topic.cc
 * @date 2008/09/01 7:07
 * @author Sergio Montenegro, Lutz Dittrich
 *
 *
 * @brief topic for middleware
 *
 */

#include "application.h"
#include "listelement.h"
#include "misc-rodos-funcs.h"
#include "reserved_application_ids.h"
#include "reserved_topic_ids.h"
#include "rodos-debug.h"
#include "subscriber.h"
#include "topic.h"


namespace RODOS {

/*************************************************/

// List TopicInterface::topicList = 0; This shall be here, but moved to main

static Application applicationName("Topics & Middleware", APID_MIDDLEWARE);

TopicInterface::TopicInterface(int64_t id, size_t len, const char* name, bool _onlyLocal) : ListElement(topicList, name)  {
    mySubscribers = 0;
    msgLen        = len;
    onlyLocal     = _onlyLocal;
    topicFilter   = 0;
    receiverNodesBitMap = 0; // see receiverNode+receiverNodesBitMap.txt
    if(id == -1) {
        topicId = hash(name) ;
        if(topicId < FIRST_USER_TOPIC_ID) { // reserved topic ids
            topicId +=  FIRST_USER_TOPIC_ID;
        }
    } else {
        topicId       = static_cast<uint32_t>(id & 0xFFFFFFFF);
    }
    if(topicId < ALL_TOPICS_BELOW_THIS_ARE_BROADCAST) receiverNodesBitMap = ~0u;

    /** Check for replications - except for udp async topic**/
    static const uint32_t udpAsyncTopicID = 22582; 
    ITERATE_LIST(TopicInterface, topicList) {
        if(iter->topicId == udpAsyncTopicID) continue;  // udp async topic can exist more than once, due to multiple link interfaces
        RODOS_ASSERT( (iter->topicId != topicId) || (iter == this) ); // Duplicated topicId
    }
} 


void TopicInterface::setTopicFilter(TopicFilter* filter) {
    if(topicFilter != 0) {
        RODOS_ERROR("More than one topicFilter for topic");
        PRINTF(" topic %s has more than one filter\n", name);
    }
    topicFilter = filter;
}

TopicInterface*  TopicInterface::findTopicId(uint32_t wantedTopicId) {
    ITERATE_LIST(TopicInterface, topicList) {
        if(iter->topicId == wantedTopicId)  return iter;
    }
    return 0;
}

// The value for receiverNode :  See receiverNode+receiverNodesBitMap.txt
int32_t TopicInterface::receiverNodesBitMap2Index() {

    if(topicId < ALL_TOPICS_BELOW_THIS_ARE_BROADCAST) { //!! Warnung! Side effect
        receiverNodesBitMap = ~0u; // all bits -> all channels -> all nodes. Redundant with constructor! Shall be!
        return -1;
    }
    uint32_t bitmap = receiverNodesBitMap;
    int32_t  indexFound = -2; // -2 -> no receiver found
    for(int index = 0; bitmap != 0; index++) {
        if(bitmap & 0x01) {
              if(indexFound != -2) return -1; // more than one -> broadcast
              indexFound = index;
        }
        bitmap >>= 1; 
    }
    return indexFound; // -2 if not found, else index of single 1
}


/**********************/

uint32_t TopicInterface::publish(void* data, bool shallSendToNetwork, NetMsgInfo* netMsgInfo) {
    return publishMsgPart(data,msgLen,shallSendToNetwork,netMsgInfo);

}

uint32_t TopicInterface::publishMsgPart(void* data, size_t lenToSend, bool shallSendToNetwork, NetMsgInfo* netMsgInfo) {
    uint32_t cnt = 0; // number of receivers a message is sent to
    NetMsgInfo localmsgInfo;

    if(!netMsgInfo) {
        localmsgInfo.init();
        netMsgInfo= & localmsgInfo;
    }


    /** If a filter is installed, it may modify the msg bevor the subscriver tet it **/
   if(topicFilter != 0)  {
       topicFilter->prolog (topicId, lenToSend, data, *netMsgInfo);
    }

    /** Distribute to all (and only) my subscribers **/
    ITERATE_LIST(Subscriber, mySubscribers) {
        if(iter->isEnabled) cnt += iter->put(topicId, lenToSend, data, *netMsgInfo);
    }

   if(topicFilter != 0)  {
       topicFilter->epilog(topicId, lenToSend, data, *netMsgInfo);
   }

   if(onlyLocal)           { return cnt; }
   if(!shallSendToNetwork) { return cnt; }

    //______________________________________________ Now distribute message to all gateways
    netMsgInfo->receiverNode        = receiverNodesBitMap2Index(); // first this due to side-effect
    netMsgInfo->receiverNodesBitMap = this->receiverNodesBitMap;
    
    ITERATE_LIST(Subscriber, defaultGatewayTopic.mySubscribers) {
        cnt += iter->put(topicId, lenToSend, data, *netMsgInfo);
    }
    return cnt;
}

void TopicInterface::publishFromInterrupt(void *any, size_t len) {
    ITERATE_LIST(Subscriber, mySubscribers) {
        iter->putFromInterrupt(topicId, any, len);
    }
}



// Basic topic used for TBA
Topic<GenericMsgRef> defaultGatewayTopic(0, "gatewayTopic");
Topic<NetworkMessage> defaultRouterTopic(-1, "routerTopic");

Topic<void*> interruptTimer(-1,   "TimerInterrupt",  true); // all these are only local
Topic<void*> interruptUart(-1,    "UartInterrupt",   true);
Topic<void*> interruptSigterm(-1, "SigTermInterrupt",true);
Topic<GenericMsgRef> charInput(-1, "CharInput",      true);

}

