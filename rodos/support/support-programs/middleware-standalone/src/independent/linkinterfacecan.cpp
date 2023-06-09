



#include"gateway/linkinterfacecan.h"
#include "gateway/gateway.h"

#include "stream-bytesex.h"

#include "hal/hal_can.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

LinkinterfaceCAN:: LinkinterfaceCAN(HAL_CAN* _can) : Linkinterface(-1),can(*_can){
	currentReceiveCANId=0;
	bufferedCANMsgs_start=0;
	bufferedCANMsgs_end=0;
}

void LinkinterfaceCAN::init() {
    isBroadcastLink=true;

    for(int i=0;i<CAN_MSG_BUFFER_SIZE;i++){
    	emptyBufferedCANMessages[i] = &BufferedCANMessage_memory[i];
    }
    emptyBufferedCANMessagesPos=CAN_MSG_BUFFER_SIZE-1;

    can.addIncomingFilter(
		CAN_LINK_ID,
		uint32_t(CAN_LINK_NODE_BITS+CAN_LINK_TOPIC_BITS),
		true);
    can.setIoEventReceiver(this);
}

/************* Connection to the network, network and protocol dependent ***/

bool LinkinterfaceCAN::sendNetworkMsg(NetworkMessage &outMsg)	{


	char buffer[8];

	uint16_t dataLength = outMsg.get_len();
	uint32_t senderNode = outMsg.get_senderNode() & uint32_t(CAN_LINK_NODE_BITS);
	uint32_t topicId= outMsg.get_topicId() & uint32_t(CAN_LINK_TOPIC_BITS);

	uint32_t canID = CAN_LINK_ID |(topicId << CAN_LINK_NODE_BITS ) | senderNode; //Construct CAN ID of fixed ID+topicID+nodeID


	uint8_t* messageData = outMsg.userDataC;


	uint8_t sequenceCounter=0;
	int count;

	int16_tToBigEndian(&buffer[1],dataLength);
	count=3;

	while(dataLength>0){

		buffer[0]=sequenceCounter;
		while(dataLength>0 && count < 8){
			buffer[count]=*messageData;
			count++;
			messageData++;
			dataLength--;
		}
		while(can.write(buffer,count,canID,true) == -1){
			//can.suspendUntilWriteFinished(NOW() + CAN_TX_TIMEOUT);
			int error;
			if((error=can.status(CAN_STATUS_TX_ERROR)) > 10){
				//Don't block if there are transmit errors
				return false;
			}
		}
		sequenceCounter++;
		count=1;

	}

    return true;
}



bool LinkinterfaceCAN::getNetworkMsg(NetworkMessage &inMsg,int32_t &numberOfReceivedBytes) {


	currentMsg = &inMsg;
	BufferedCANMessage* msg;
	numberOfReceivedBytes = -1;

	if(currentReceiveCANId==0){
		//No message is in receive at the moment, so look at the buffer

		BufferedCANMessage* bufElement = bufferedCANMsgs_start;
		BufferedCANMessage* prevBufElement =0;

		while(bufElement !=0){
			if(bufElement->canID==currentReceiveCANId || currentReceiveCANId==0){

				//Remove from buffer
				if(prevBufElement==0){ //first element
					bufferedCANMsgs_start=bufElement->next;
				}else{
					prevBufElement->next=bufElement->next;
				}
				if(bufElement->next==0){ //was last
					bufferedCANMsgs_end=prevBufElement;
				}



				if(appendCANMsgToCurrentNetMsg(bufElement)){
					return true;
				}
			}else{
				//If we removed the current element from the buffer prev elemts stays the same of next round
				prevBufElement=bufElement;
			}
			bufElement=bufElement->next;
		}
	}



	while(1){

		msg=receiveCANMessage();

		if(msg==0){
			return false;
		}


		if(msg->canID==currentReceiveCANId || currentReceiveCANId==0){
			 if(appendCANMsgToCurrentNetMsg(msg)){
				 return true;
			 }
		}else{
			//put in buffer
			msg->next=0;
			if(bufferedCANMsgs_end==0){
				bufferedCANMsgs_start=msg;
			}else{
				bufferedCANMsgs_end->next=msg;
			}
			bufferedCANMsgs_end=msg;
		}



	}
}



BufferedCANMessage* LinkinterfaceCAN::receiveCANMessage(){

	BufferedCANMessage* msg,*previous=0;

	if(emptyBufferedCANMessagesPos < 0){
		//There are no entries left in the buffer
		// so steal the last entry of the buffer to keep it running


		//We assume here that linked list bufferedCANMsgs has at least two entries, but that should be the case when there are no free buffer elements
		msg=bufferedCANMsgs_start;
		while(msg->next!=0){
			previous=msg;
			msg=msg->next;
		}
		previous->next=0;
		bufferedCANMsgs_end=previous;

	}else{
		msg=emptyBufferedCANMessages[emptyBufferedCANMessagesPos];
	}


	if((msg->len=can.read((char*)msg->data,&msg->canID)) >= 0){

		if(!(emptyBufferedCANMessagesPos < 0)){
			emptyBufferedCANMessagesPos--;
		}
		return msg;

	}


	if(emptyBufferedCANMessagesPos < 0){ // put msg back to empty msgs if it was taken out of the buffer
		emptyBufferedCANMessagesPos++;
		if(emptyBufferedCANMessagesPos < 0 || emptyBufferedCANMessagesPos >= CAN_MSG_BUFFER_SIZE){
			ERROR("ASSERT: emptyBufferedCANMessagesPos out of range");
		}
		emptyBufferedCANMessages[emptyBufferedCANMessagesPos]=msg;
	}

	return 0;
}

bool LinkinterfaceCAN::appendCANMsgToCurrentNetMsg(BufferedCANMessage* canMsg){

	static int64_t fakeTime=0;

	emptyBufferedCANMessagesPos++;
	if(emptyBufferedCANMessagesPos < 0 || emptyBufferedCANMessagesPos >= CAN_MSG_BUFFER_SIZE) {
		ERROR("ASSERT: emptyBufferedCANMessagesPos out of range");
	}
	emptyBufferedCANMessages[emptyBufferedCANMessagesPos]=canMsg;


	//*error=false;
	int userDataBegin=1;


	if(canMsg->data[0] == 0){
		// New msg
		uint16_t len = bigEndianToInt16_t(&canMsg->data[1]);
		currentReceiveCANId=canMsg->canID;

		currentMsg->put_maxStepsToForward(5);
		currentMsg->put_senderNode(static_cast<int32_t>(currentReceiveCANId & uint32_t(CAN_LINK_NODE_BITS)));
		currentMsg->put_senderThreadId(0);
		currentMsg->put_sentTime(fakeTime++);
		currentMsg->put_topicId((currentReceiveCANId >> CAN_LINK_NODE_BITS) & uint32_t(CAN_LINK_TOPIC_BITS));
		currentMsg->put_len(len);
		currentDataPointer = currentMsg->userDataC;
		bytesReceived=0;
		sequenceCounter=0;
		userDataBegin=3;
	}

	if(currentReceiveCANId==0){
		if(canMsg->data[0] >0){
			//*error=true;
			return false;
		}
	}else{
		if(canMsg->canID != currentReceiveCANId || canMsg->data[0] != sequenceCounter){
			//*error=true;
			return false;

		}
	}


	for(int i=userDataBegin;i< canMsg->len;i++){
		*currentDataPointer = canMsg->data[i];
		currentDataPointer++;
	}
	bytesReceived +=canMsg->len-userDataBegin;


	if(bytesReceived >= currentMsg->get_len()){

		currentMsg->setCheckSum();
		currentReceiveCANId=0;
		return true;
	}

	sequenceCounter++;
	return false;

}


void LinkinterfaceCAN::onWriteFinished() {
    //if(threadToResume) threadToResume->resume();
}

void LinkinterfaceCAN::onDataReady() {
    //if(threadToResume) threadToResume->resume();
}


#ifndef NO_RODOS_NAMESPACE
}
#endif
