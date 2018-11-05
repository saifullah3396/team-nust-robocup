/**
 * @file TeamComm/TeamComm.h
 *
 * This file declares the class TeamComm
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Jan 2017
 */

#pragma once

#include "CommModule/include/CommModule.h"
#include "CommModule/include/UdpComm.h"
#include "CommModule/include/TeamBallTracker.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/SPLStandardMessage.h"
#include "Utils/include/TeamBallInfo.h"
#include "Utils/include/ThreadSafeQueue.h"
#include <boost/make_shared.hpp>

/**
 * @class TeamComm
 * @brief This class deals with inter-robot communication using udp
 *   broadcast and messsage receival
 */
class TeamComm : public MemoryBase
{
public:
  /**
   * Constructor
   *
   * @param commModule pointer to parent CommModule
   */
  TeamComm(CommModule* commModule) :
    MemoryBase(commModule), commModule(commModule), port(0)
  {
  }

  /**
   * Destructor
   */
  ~TeamComm()
  {
  }

  /**
   * Starts up the udp communication on the given port
   *
   * @param port port used for udp communication
   * @param subnet broadcast ip address of the system
   * @return void
   */
  void
  startUp(const int& port, const char* subnet);

  /**
   * Broadcasts the SPLStandardMessage obtained from the sendQueue
   *   front
   *
   * @return void
   */
  void
  send();

  /**
   * Receives the SPLStandardMessage and adds it to the receiveQueue
   *
   * @return unsigned
   */
  unsigned
  receive();

  /**
   * Processes messages stored in receiveQueue
   *
   * @return void
   */
  void
  processReceivedMsgs();

  /**
   * Processes an incoming StandarSPLMessage message and updates the
   * team robots data according to the message.
   *
   * @return void
   */
  void
  updateTeamData(const SPLStandardMessage& msg);

  /**
   * Updates the ball data in world using robots own and other robots data
   *
   * @return void
   */
  void
  updateTeamBallInfo();

  void
  initTBTracker()
  {
    tBTracker = boost::make_shared <TeamBallTracker> (commModule);
    tBTracker->init();
  }

private:
  /**
   * Updates the SPLStandardMsg to be broadcasted based on the current
   * robot state.
   *
   * @param msg message to be updated
   *
   * @return void
   */
  void
  updateMessage(SPLStandardMessage& msg);

  //! Port to be used for udp communication. Team ports are setup as
  //! 10000 + teamNumber whereas teamNumber is defined in
  //! Config/TeamSettings.
  int port;

  //! UdpComm class object for udp connection management.
  UdpComm udpComm;

  //! CommModule pointer
  CommModule* commModule;

  //! The receive queus for SPLStandardMessage.
  ThreadSafeQueue<SPLStandardMessage> receiveQueue;

  //! Team ball tracker class object.
  boost::shared_ptr<TeamBallTracker> tBTracker;
};

typedef boost::shared_ptr<TeamComm> TeamCommPtr;
