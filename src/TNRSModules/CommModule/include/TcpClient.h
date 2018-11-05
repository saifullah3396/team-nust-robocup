/**
 * @file CommModule/TcpClient.h
 *
 * This file declares the classes TcpMessage & TcpClient
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Jun 2017
 */

#pragma once

#include "CommModule/include/ClientInfo.h"
#include "CommModule/include/CommMessage.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/ThreadSafeQueue.h"

using namespace Utils;

#ifdef _WIN32
#include <winsock.h>
#else
#include <netinet/in.h>
#endif

class TcpConnection;

/**
 * @class TcpMessage
 * @brief Defines a message that can be sent with some info over the tcp
 *   connection
 */
struct TcpMessage : CommMessage
{
  /**
   * Constructor
   *
   * @param msg: Message to be sent
   * @param type: Message type
   * @param targetClientType: Type of target client
   * @param relatedClientId: Client id
   */
  TcpMessage(
    const string& msg,
    const CommMsgTypes& type, 
    const TcpClientType& targetClientType, 
    const int& relatedClientId) :
    CommMessage(msg, type),
    targetClientType(targetClientType),
    relatedClientId(relatedClientId)
  {
  }

  //! Target client type for the current message
  TcpClientType targetClientType;

  //! Id of client this message is meant for
  int relatedClientId;
};

/**
 * @class TcpClient
 * @brief This class defines a tcp client
 */
class TcpClient
{
public:
  /**
   * Opens a TCP connection to a remote host
   *
   * @param parentConnection connection related to the client
   * @param transferSocket tranfer socket for the connection
   * @param address connection socket address
   * @param type type of the tcp client concerned
   * @param maxPacketSendSize maximum size of an outgoing packet.
   *   If 0, this setting is ignored.
   * @param maxPacketReceiveSize maximum size of an incoming packet.
   *   If 0, this setting is ignored.
   */
  TcpClient(TcpConnection* parentConnection, int transferSocket,
    sockaddr_in address, TcpClientType type, int maxPacketSendSize = 0,
    int maxPacketReceiveSize = 0);

  /**
   * Destructor
   */
  ~TcpClient();

  /**
   * The function sends a block of bytes. It will return immediately
   * unless the send buffer is full.
   *
   * @param buffer bytes to send
   * @param size number of bytes to send
   * @return boolean
   */
  bool
  send(const char* buffer, int size);

  /**
   * The function receives a block of bytes
   *
   * @param recvDataQueue data receival queue
   * @param buffer the buffer to be filled with incoming data
   * @param size number of bytes to receive
   * @param noMoreData whether there is no data left
   * @param maxSize max data size
   * @return was the data successfully recieved?
   */
  bool
  receive(queue<TcpMessage>& recvDataQueue, char* buffer, int &size,
    bool &noMoreData, int maxSize);

  /**
   * The function receives a block of bytes using the system call recv()
   *
   * @param buffer buffer to be filled with incoming data
   * @param size size of buffer
   * @param flags See RECV(2) manpage
   * @return number of bytes received, or -1 on error
   */
  int
  receiveSys(void* buffer, unsigned size, int flags);

  /**
   * Returns true if the client startup procedure has taken place
   *
   * @return bool
   */
  bool
  getStartup()
  {
    return startup;
  }

  /**
   * Sets the status of client startup procedure
   *
   * @return void
   */
  void
  setStartup(const bool& startup)
  {
    this->startup = startup;
  }

  /**
   * Returns true if the client needs to be deleted
   *
   * @return bool
   */
  bool
  getDeleteClient()
  {
    return deleteClient;
  }

  /**
   * Sets whether the client should be deleted
   *
   * return void
   */
  void
  setDeleteClient(const bool& deleteClient)
  {
    this->deleteClient = deleteClient;
  }

  /**
   * Returns the overall number of bytes sent so far by this object
   *
   * @return int
   */
  int
  getOverallBytesSent() const
  {
    return overallBytesSent;
  }

  /**
   * Returns the overall number of bytes received so far by this object
   *
   * @return int
   */
  int
  getOverallBytesReceived() const
  {
    return overallBytesReceived;
  }

  /**
   * Returns true if the connection was successful
   *
   * @return bool
   */
  bool
  connected() const
  {
    return transferSocket > 0;
  }

  /**
   * Returns the address of the connected client
   *
   * @return sockaddr_in
   */
  sockaddr_in&
  getAddress()
  {
    return address;
  }

  /**
   * Returns the client type
   *
   * @return TcpClientType
   */
  TcpClientType
  getType()
  {
    return type;
  }

  /**
   * Returns the client Id
   *
   * @return long
   */
  long
  getId()
  {
    return clientId;
  }

  /**
   * Returns the client info struct
   *
   * @return ClientInfo
   */
  ClientInfo
  getClientInfo()
  {
    return clientInfo;
  }

private:
  /**
   * Closes the transfer socket
   *
   * @return void
   */
  void
  closeTransferSocket();

  //! The parent connection related to the client
  TcpConnection * parentConnection;

  //! The socket address
  sockaddr_in address;

  //! The maximum size of an outgoing package
  int maxPacketSendSize;

  //! The maximum size of an incoming package
  int maxPacketReceiveSize;

  //! The handle of the actual transfer socket
  int transferSocket;

  //! The overall number of bytes sent so far
  int overallBytesSent;

  //! The overall number of bytes received so far
  int overallBytesReceived;

  //! Whether a tranfer connection was established or not
  bool wasConnected;

  //! Type of TcpClient
  TcpClientType type;

  //! Total connected clients
  static long clientCount;

  //! Client Id
  long clientId;

  //! Client initial startup done?
  bool startup;

  //! Client ready to deleted
  bool deleteClient;
  
  //! ClientInfo
  ClientInfo clientInfo;
};
