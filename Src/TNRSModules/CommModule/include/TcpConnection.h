/**
 * @file CommModule/TcpConnection.h
 *
 * This file declares the class TcpConnection
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Jun 2017
 */

#pragma once

#include "CommModule/include/TcpClient.h"
#include "Utils/include/DebugUtils.h"

/**
 * Max (~64 MB) package size that can be received
 * Prevents from allocating too much buffer
 */
#define MAX_PACKET_SIZE 67108864

/**
 * @class TcpClient
 * @brief This class defines a tcp connection between a client and the
 *   server
 */
class TcpConnection
{
public:
  /**
   * Initializes a tcp connection given the parameters
   *
   * @param port port to be allocated for connection
   * @param maxPacketSendSize max outgoing packet size
   *  This parameter is ignored if set to zero
   * @param maxPacketReceiveSize max incoming packet size
   *  This parameter is ignored if set to zero
   */
  TcpConnection(const int& port, const int& maxPacketSendSize = 0,
    const int& maxPacketReceiveSize = 0);

  /**
   * Destructor
   */
  ~TcpConnection();

  /**
   * Initializes a tcp server
   *
   * @param dataPort the port for data sending/receival
   * @param imagePort the port for image sending/receival
   * @return void
   */
  void
  initServer(const int& dataPort, const int& imagePort);

  /**
   * This function deals with new connections and allocates ids
   *
   * @return bool
   */
  bool
  serviceNewConnections(vector<TcpClient*>& clients);

protected:
  //! The socket for sending variable data
  int createDataSocket;

  //! The socket for sending image data
  int createImageSocket;
};
