/**
 * @file CommModule/CommModule.h
 *
 * This file declares the class CommModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <type_traits>
#include "CommModule/include/CommMsgTypes.h"
#include "CommModule/include/TcpConnection.h"
#include "TNRSBase/include/DebugBase.h"
#include "TNRSBase/include/BaseIncludes.h"
#include "Utils/include/BallInfo.h"
#include "Utils/include/TeamBallInfo.h"
#include "Utils/include/TeamRobot.h"
#include "Utils/include/ThreadSafeQueue.h"

class TeamComm;
typedef boost::shared_ptr<TeamComm> TeamCommPtr;

#define TEMP_DATA_BUFFER_SIZE 1000

/**
 * @class CommModule
 * @brief This class defines all the functions and algorithms for data
 *   that are necessary for communication between the robots and the game
 *   controller
 */
class CommModule : public BaseModule, public DebugBase
{
CREATE_INPUT_CONNECTOR(CommInput,
  (int, commThreadPeriod),
  (int, playerNumber),
  (int, teamNumber),
  (int, teamPort),
  (bool, robotFallen),
  (RobotPose2D<float>, robotPose2D),
  (RobotPose2D<float>, moveTarget),
  (Vector2f, kickTarget),
  //(BehaviorAccepted, lastMBehaviorAccepted),
  (BallInfo, ballInfo),
  (int, robotIntention),
  (bool, robotLocalized),
  (int, positionConfidence),
  (int, sideConfidence),
)

CREATE_OUTPUT_CONNECTOR(CommOutput,
  (int, heartBeat),
  (vector<ClientInfo>, clientsInfo),
  (vector<TeamRobot>, teamRobots),
  (TeamBallInfo, teamBallInfo),
)

INIT_DEBUG_BASE_(
  (unsigned, sendLogs, 0),
)

public:
  /**
   * Initializes the communication module
   *
   * @param teamNUSTSPL pointer to base class which is in this case
   *   the TeamNUSTSPL class
   */
  CommModule(void* teamNUSTSPL);

  /**
   * Destructor
   */
  ~CommModule()
  {
    for (size_t i = 0; i < clients.size(); ++i)
      delete clients[i];
      delete dataRead;
    delete genericInputConnector;
    delete genericOutputConnector;
  }

  /**
   * Derived from BaseModule
   */
  void
  init();

  /**
   * Derived from BaseModule
   */
  void
  mainRoutine();
  
  /**
   * Derived from BaseModule
   */
  void
  handleRequests();

  /**
   * Derived from BaseModule
   */
  void
  initMemoryConn();

  /**
   * Derived from BaseModule
   */
  void
  setThreadPeriod();

private:
  /**
   * Defines send/receive data handling for communication between
   * the robot and the debugging GUI interface
   *
   * @return void
   */
  void
  sendReceive();

  /**
   * Initializes the team communication
   *
   * @return void
   */
  void
  initTeamComm();

  /**
   * Client startup communication management (GUI communication channel)
   *
   * @param client the TcpClient to which the robot is connected
   * @return void
   */
  void
  clientStartupComm(TcpClient* client);

  /**
   * Handles incoming messages (such as user commands)
   *
   * @return void
   */
  void
  updateIncomingMessageQueue();

  /**
   * Sets the data modules for which the data is to be sent over the network.
   *
   * @return void
   */
  void
  setDataModules(const vector<boost::shared_ptr<BaseModule> >& dataModules)
  {
    this->dataModules = dataModules;
  }

  /**
   * Recieves the incoming data into the receive buffer
   *
   * @param clientIndex connected client to read data from
   * @return int
   */
  int
  receive(const int& clientIndex);

  /**
   * Manages clients
   */
  void
  manageClients();

  /**
   * Sends a heart beat signal for connection verification
   *
   * @return void
   */
  void
  sendHeartBeat();

  /**
   * Sends the shared memory variable headers to the data client
   *
   * @return void
   */
  void
  sendMemoryHeader();

  /**
   * Sends the shared memory variable data to the data client
   *
   * @return void
   */
  void
  sendMemoryData();

  /**
   * Sends thread select lines to the data client for data handling in
   * GUI
   *
   * @return void
   */
  void
  sendThreadSelectLines();

  /**
   * Defines and sends the TcpMessage for the robot heartbeat
   *
   * @param msg message to be sent
   * @param targetClientId target client id
   * @return void
   */
  void
  sendHeartbeat(const string& msg, const long& targetClientId);

  /**
   * Converts an image into a byte string and adds it to the send queue
   * 
   * @param image: Image matrix
   *
   * @return void
   */
  void
  addImageToQueue(const Mat& image);
  
  /**
   * Adds a comm message to the send queue
   * 
   * @param cMsg: The comm msg
   * @param targetClientType: Type of client
   * @param relatedClientId: The client for which the message is for
   *
   * @return void
   */
  void
  addCommMessageToQueue(
    const CommMessage& cMsg,
    const TcpClientType& targetClientType, 
    const int& relatedClientId);
  
  /**
   * Processes an incoming communication message
   *
   * @param tcpMsg TcpMessage to be processed
   */
  void
  processIncomingMessage(const TcpMessage& tcpMsg);
  
  //! Tcp connection variable for initializing a connection with
  //! the GUI interface
  TcpConnection* tcpConnection;

  //! Pointer to TeamComm class
  TeamCommPtr teamComm;

  //! Vector of all the connected clients
  vector<TcpClient*> clients;

  //! If true, all memory data is sent over the network
  //! Updated from configuration
  bool memoryDataSwitch;

  //! Port for streaming of normal data
  int dataPort;
  
  //! Port for streaming of image data
  int imagePort;

  //! The data receival buffer
  char* dataRead;

  //! The queue for sending tcp messages
  queue<TcpMessage> sendDataQueue;

  //! The queue for receiving tcp messages
  queue<TcpMessage> recvDataQueue;

  //! Last team communication update
  float lastTeamCommSendTime;

  //! Vector of pointers to all running threads for which data is to be sent.
  vector<boost::shared_ptr<BaseModule> > dataModules;
};
