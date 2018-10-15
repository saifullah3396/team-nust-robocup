/**
 * @file CommModule/CommModule.h
 *
 * This file implements the class CommModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "Utils/include/ConfigMacros.h"
#include "CommModule/include/CommModule.h"
#include "CommModule/include/CommRequest.h"
#include "CommModule/include/TeamComm.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include <opencv2/opencv.hpp>

CommModule::CommModule(void* teamNUSTSPL) :
  BaseModule(teamNUSTSPL, (unsigned) TNSPLModules::COMM, "CommModule"),
    DebugBase("CommModule", this)
{
  GET_CONFIG("CommModule",
    (int, dataPort, dataPort),
    (int, imagePort, imagePort),
    (bool, memoryDataSwitch, memoryDataSwitch),
  )
}

void
CommModule::setThreadPeriod()
{
  setPeriodMinMS(IVAR(int, CommModule::commThreadPeriod));
}

void
CommModule::initMemoryConn()
{
  ASSERT_MSG(sharedMemory, "Shared Memory not found.");
  genericInputConnector = 
    new InputConnector(this, getModuleName() + "InputConnector");
  genericOutputConnector = 
    new OutputConnector(this, getModuleName() + "OutputConnector");
  genericInputConnector->initConnector();
  genericOutputConnector->initConnector();
}

void
CommModule::init()
{
  initDebugBase();
  initTeamComm();
  //!make server module on port 20010.
  tcpConnection = new TcpConnection(dataPort, imagePort, 10000, 10000);
  dataRead = new char[TEMP_DATA_BUFFER_SIZE];
  lastTeamCommSendTime = 0.f;  
}

void
CommModule::handleRequests()
{
  //auto cmsg = CommMessage("black lotus is my sister", CommMsgTypes::LOG_TEXT);
  //SendMsgRequestPtr smReq = boost::make_shared<SendMsgRequest>(cmsg);
  //BaseModule::publishModuleRequest(smReq);
  //inRequests.pushToQueue(smReq);
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <CommRequest>(request)) {
    auto reqId = request->getId();
    if (reqId == (unsigned)CommRequestIds::SEND_MSG_REQUEST) {
      auto smr = boost::static_pointer_cast<SendMsgRequest>(request);
      if (smr->cMsg.getType() > CommMsgTypes::REQUEST_MSGS) {
        addCommMessageToQueue(smr->cMsg, BASIC, -1);
      }
    } else if (reqId == (unsigned)CommRequestIds::SEND_IMAGE_REQUEST) {
      auto smr = boost::static_pointer_cast<SendImageRequest>(request);
      addImageToQueue(smr->image);
    }
  }
  inRequests.popQueue();
}

void
CommModule::mainRoutine()
{
  // We can only send 5 messages in one second
  //if (getModuleTime() - lastTeamCommSendTime >= 0.2f) {
  //  teamComm->send();
  //  lastTeamCommSendTime = getModuleTime();
  //}
  //teamComm->receive();
  //teamComm->processReceivedMsgs();
  //teamComm->updateTeamBallInfo();
  manageClients();
  if (tcpConnection->serviceNewConnections(clients)) {
    for (size_t i = 0; i < clients.size(); ++i) {
      clientStartupComm(clients[i]);
    }
  }
  if (clients.empty()) {
    return;
  }
  if (memoryDataSwitch) {
    sendHeartBeat();
    sendMemoryData();
    //sendThreadSelectLines();
  }
  sendReceive();
  updateIncomingMessageQueue();
  OVAR(int, CommModule::heartBeat)++;
}

void
CommModule::initTeamComm()
{
  string bcastAddr = UdpComm::getWifiBroadcastAddress();
  teamComm = boost::shared_ptr < TeamComm > (new TeamComm(this));
  teamComm->startUp(IVAR(unsigned, CommModule::teamPort), bcastAddr.c_str());
  teamComm->initTBTracker();
}

void
CommModule::clientStartupComm(TcpClient* client)
{
  if (!client->getStartup()) {
    if (client->getType() == BASIC) 
      sendMemoryHeader();
    client->setStartup(true);
  }
}

void
CommModule::manageClients()
{
  OVAR(vector<ClientInfo>, CommModule::clientsInfo).clear();
  for (size_t i = 0; i < clients.size(); ++i) {
    if (clients[i]->getDeleteClient()) {
      TcpClient* p = clients[i];
      clients.erase(clients.begin() + i);
      delete p;
    } else {
      OVAR(vector<ClientInfo>, CommModule::clientsInfo).push_back(clients[i]->getClientInfo());
    }
  }
}

void
CommModule::sendMemoryHeader()
{
  //! Send memory header
  string memoryHeader;
  getLocalSharedMemory()->getStringHeader(memoryHeader);
  CommMessage cMsg(memoryHeader, CommMsgTypes::MEMORY_HEADER);
  addCommMessageToQueue(cMsg, BASIC, -1);  
}

void
CommModule::sendHeartBeat()
{
  //! Send heart beat signal
  CommMessage cMsg("", CommMsgTypes::HEART_BEAT);
  addCommMessageToQueue(cMsg, BASIC, -1);  
}

void
CommModule::sendMemoryData()
{
  //! Send memory data
  string memoryData;
  getLocalSharedMemory()->getString(memoryData);
  CommMessage cMsg(memoryData, CommMsgTypes::MEMORY_DATA);
  addCommMessageToQueue(cMsg, BASIC, -1);
}

void
CommModule::sendThreadSelectLines()
{
  //! Send multiplexers select lines.
  string msgStr = "";
  for (size_t i = 0; i < (unsigned)TNSPLModules::COUNT; ++i) {
    BaseModule* bm = BaseModule::getModule(i);
    bm->getInputMuxSelectLineString(msgStr);
    msgStr += "%";
  }
  CommMessage cMsg(msgStr, CommMsgTypes::THREADS_SELECT_LINES);
  addCommMessageToQueue(cMsg, BASIC, -1);
}

void 
CommModule::addCommMessageToQueue(
  const CommMessage& cMsg,
  const TcpClientType& targetClientType, 
  const int& relatedClientId)
{
  auto msg = TcpMessage(
    cMsg.getMessage(), 
    cMsg.getType(),
    targetClientType,
    relatedClientId
  );
  sendDataQueue.push(msg);
}

void
CommModule::addImageToQueue(const Mat& image)
{
  vector <uchar> bytesData;
  if (imencode(".jpg", image, bytesData)) {
    const unsigned char* ptr = &bytesData[0];
    string data = DataUtils::bytesToHexString(ptr, bytesData.size());
    auto msg = CommMessage(data, CommMsgTypes::IMAGE);
    addCommMessageToQueue(msg, IMAGE, -1);
  }
}

void
CommModule::sendReceive()
{
  //!Data sending
  while (!sendDataQueue.empty()) {
    auto tcpMsg = sendDataQueue.front();
    string dataToSend = 
      DataUtils::varToString((unsigned)tcpMsg.getType()) + "@" + tcpMsg.getMessage() + "\n";
    for (size_t i = 0; i < clients.size(); ++i)
    {
      if (clients[i]->connected() &&
          tcpMsg.targetClientType == clients[i]->getType() &&
          tcpMsg.relatedClientId == -1 || 
          tcpMsg.relatedClientId == clients[i]->getId()) 
      {
        if (!clients[i]->send(dataToSend.c_str(), dataToSend.length())) {
          ERROR("Failed to send data to client " + DataUtils::varToString(i))
        }
      }
    }
    sendDataQueue.pop();
  }
  //!Data recieval
  for (size_t i = 0; i < clients.size(); ++i) {
    if (clients[i]->connected()) {
      int bytesRead = 0;
      do {
        bytesRead = receive(i);
        if (bytesRead < 0) continue;
      } while (bytesRead > 0);
    } else {
      clients[i]->setDeleteClient(true);
    }
  }
}

int
CommModule::receive(const int& clientIndex)
{
  if (!dataRead) {
    //ParentThread->DisplayToLog(1, "ASSERT:"  "receive:Error creating recv buffer");
    return 0;
  }
  int sizeReceived = 0;
  bool noMoreData = false;
  bool success = clients[clientIndex]->receive(
    recvDataQueue,
    dataRead,
    sizeReceived,
    noMoreData,
    TEMP_DATA_BUFFER_SIZE);
  if (!noMoreData) {
    if (!success) {
      return -1;
    } else {
      return sizeReceived;
    }
  } else {
    return 0;
  }
}

void
CommModule::processIncomingMessage(const TcpMessage& tcpMsg)
{
  string reply = DebugBase::processDebugMsg(tcpMsg.getMessage());
  auto msg = CommMessage(reply, CommMsgTypes::LOG_TEXT);
  addCommMessageToQueue(msg, BASIC, tcpMsg.relatedClientId);
}

void
CommModule::updateIncomingMessageQueue()
{
  while (!recvDataQueue.empty()) {
    processIncomingMessage(recvDataQueue.front());
    recvDataQueue.pop();
  }
}
