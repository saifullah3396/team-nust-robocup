/**
 * @file CommModule/TcpClient.h
 *
 * This file implements the classes TcpMessage & TcpClient
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Jun 2017
 */

#include "CommModule/include/TcpClient.h"
#include "CommModule/include/TcpConnection.h"

#ifndef WINCE
#include <cerrno>
#include <fcntl.h>
#endif

#ifdef _WIN32
#ifndef WINCE
#include <sys/types.h>
#endif
#define ERRNO WSAGetLastError()
#define RESET_ERRNO WSASetLastError(0)
#undef EWOULDBLOCK
#define EWOULDBLOCK WSAEWOULDBLOCK
#undef EINPROGRESS
#define EINPROGRESS WSAEINPROGRESS
#define NON_BLOCK(socket) ioctlsocket(socket, FIONBIO, (u_long*) "NONE")
#define CLOSE(socket) closesocket(socket)
class _WSAFramework
{
public:
  _WSAFramework()
  {
    WORD wVersionRequested = MAKEWORD(1, 0);
    WSADATA wsaData;
    WSAStartup(wVersionRequested, &wsaData);
  }
  ~_WSAFramework() {WSACleanup();}
}_wsaFramework;
#else
#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/ioctl.h>
#define ERRNO errno
#define RESET_ERRNO errno = 0
#define NON_BLOCK(socket) fcntl(socket,F_SETFL,O_NONBLOCK)
#define CLOSE(socket) close(socket)
#endif

long TcpClient::clientCount = 0;

TcpClient::TcpClient(TcpConnection* parentConnection, int transferSocket,
  sockaddr_in address, TcpClientType type, int maxPacketSendSize,
  int maxPacketReceiveSize) :
  parentConnection(parentConnection), type(type), address(address),
    transferSocket(transferSocket), overallBytesSent(0),
    overallBytesReceived(0), maxPacketSendSize(maxPacketSendSize),
    maxPacketReceiveSize(maxPacketReceiveSize), wasConnected(false),
    startup(false), deleteClient(false)
{
  clientId = clientCount++;
  wasConnected = true;
  NON_BLOCK(transferSocket);
}

TcpClient::~TcpClient()
{
  if (connected()) closeTransferSocket();
}

void
TcpClient::closeTransferSocket()
{
  CLOSE(transferSocket);
  transferSocket = 0;
  setDeleteClient(true);
}

bool
TcpClient::send(const char* buffer, int size)
{
  RESET_ERRNO;
  int sent = ::send(transferSocket, (const char*) buffer, size, 0);
  if (sent > 0) {
    overallBytesSent += sent;
    while (sent < size && (ERRNO == EWOULDBLOCK || ERRNO == EINPROGRESS || ERRNO == 0)) {
      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;
      fd_set wset;
      FD_ZERO(&wset);
      FD_SET(transferSocket, &wset);
      RESET_ERRNO;
      if (select(transferSocket + 1, 0, &wset, 0, &timeout) == -1) break;
      RESET_ERRNO;
      int sent2 = ::send(
        transferSocket,
        (const char*) buffer + sent,
        size - sent,
        0);
      if (sent2 >= 0) {
        sent += sent2;
        overallBytesSent += sent;
      }
    }
  }
  if (ERRNO == 0 && sent == size) {
    return true;
  } else {
    closeTransferSocket();
    return false;
  }
}

bool
TcpClient::receive(queue<TcpMessage>& recvDataQueue, char* buffer,
  int & size, bool & noMoreData, int maxSize)
{
  size = 0;
  while (true) {
    //peak ahead and see if there is more data
    RESET_ERRNO;
#ifndef _WIN32
    int received = recv(transferSocket, (char*) buffer, 1, MSG_PEEK);
    if (received < 1) {
      if (!received || (received < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS)) closeTransferSocket();
      noMoreData = true;
      return false;
    }
#else
    char c;
    int received = recv(transferSocket, &c, 1, MSG_PEEK);
    if(!received || (received < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS) ||
      ioctlsocket(transferSocket, FIONREAD, (u_long*) &received) != 0) {
      closeTransferSocket();
      noMoreData = true;
      return false;
    } else if(received == 0) {
      noMoreData = true; //loop exit point
      return false;
    }
#endif
    received = 0;
    char lastReadCh = 0;
    while (lastReadCh != 10 && received < maxSize) {
      RESET_ERRNO;
      //int received2 = recv(transferSocket, (char*) buffer + received,
      //                     size - received, 0);

      //receive byte by byte
      int received2 = recv(transferSocket, (char*) buffer + received, 1, 0);

      lastReadCh = buffer[received];

      if (!received2 || (received2 < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS)) {
        closeTransferSocket();
        return false;
      } else if (ERRNO == EWOULDBLOCK || ERRNO == EINPROGRESS) {
        received2 = 0;
        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        fd_set rset;
        FD_ZERO(&rset);
        FD_SET(transferSocket, &rset);
        if (select(transferSocket + 1, &rset, 0, 0, &timeout) == -1) {
          closeTransferSocket();
          return false;
        }
      }
      received += received2;
      overallBytesReceived += received2;
    }
    buffer[--received] = 0; //mark the end of string
    size += --received; //take out the EOL and CR characters count
    //add to recv queue
    auto msg = string(buffer);
    vector <string> parts = DataUtils::splitString(msg, '@');
    if (parts.size() == 2) {
      stringstream ss(parts[0]);
      unsigned msgType;
      ss >> msgType;
      auto msg = 
        TcpMessage(parts[1], (CommMsgTypes)msgType, type, clientId);
      recvDataQueue.push(msg);
    }
  }
  return true; // ok, data received
}

int
TcpClient::receiveSys(void* buffer, unsigned size, int flags)
{
  return recv(transferSocket, (char*) buffer, size, flags);
}
