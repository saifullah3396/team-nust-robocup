/**
 * @file CommModule/TcpConnection.h
 *
 * This file implements the class TcpConnection
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Jun 2017
 */

#include "CommModule/include/CommModule.h"
#include "CommModule/include/TcpConnection.h"

#ifndef WINCE
#include <cerrno>
#include <fcntl.h>
#endif

//socket related definitions
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

TcpConnection::TcpConnection(
  const int& dataPort, 
  const int& imagePort, 
  const int& maxPackageSendSize,
  const int& maxPackageReceiveSize)
{
  initServer(dataPort, imagePort);
}

TcpConnection::~TcpConnection()
{
  if (createDataSocket > 0)
  CLOSE(createDataSocket);
}

void
TcpConnection::initServer(const int& dataPort, const int& imagePort)
{
  sockaddr_in dataAddress;
  dataAddress.sin_family = AF_INET;
  sockaddr_in imageAddress;
  imageAddress.sin_family = AF_INET;

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
  dataAddress.sin_port = htons(dataPort);
  imageAddress.sin_port = htons(imagePort);
#ifdef __clang__
#pragma clang diagnostic pop
#endif
  createDataSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (createDataSocket <= 0) {
    ERROR("Communication server initiation failed")
    return;
  }
  int val = 1;
  setsockopt(
    createDataSocket,
    SOL_SOCKET,
    SO_REUSEADDR,
    (char*) &val,
    sizeof(val));
  dataAddress.sin_addr.s_addr = INADDR_ANY;
  if (!::bind(createDataSocket, (sockaddr*) &dataAddress, sizeof(sockaddr_in)) == 0) {
    return;
  }
  char *ip = inet_ntoa(dataAddress.sin_addr);
  LOG_INFO("Machine address:" + string(ip))
  if (!listen(createDataSocket, SOMAXCONN) == 0) {
    ERROR("Communication server initiation failed")
    return;
  }
  LOG_INFO(
    "Listening for incoming connections on port " + DataUtils::varToString(dataPort));
  NON_BLOCK(createDataSocket);
  createImageSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (createImageSocket <= 0) {
    return;
  }
  val = 1;
  setsockopt(
    createImageSocket,
    SOL_SOCKET,
    SO_REUSEADDR,
    (char*) &val,
    sizeof(val));
  imageAddress.sin_addr.s_addr = INADDR_ANY;
  if (!::bind(createImageSocket, (sockaddr*) &imageAddress, sizeof(sockaddr_in)) == 0) {
    return;
  }
  if (!listen(createImageSocket, SOMAXCONN) == 0) {
    return;
  }
  LOG_INFO(
    "Listening for incoming connections on port " + DataUtils::varToString(imagePort));
  NON_BLOCK(createImageSocket);
}

bool
TcpConnection::serviceNewConnections(vector<TcpClient*>& clients)
{
  bool clientAdded = false;
  if (createDataSocket) { //!If this is the server
    //!Make a new client if there is a request
#ifndef WIN32
    unsigned int addrlen = sizeof(sockaddr_in);
#else
    int addrlen = sizeof(sockaddr_in);
#endif
    sockaddr_in dataAddress;
    int transferSocket = accept(
      createDataSocket,
      (sockaddr*) &dataAddress,
      &addrlen);
    if (transferSocket > 0) {
      LOG_INFO(
        "Accepted basic client with socket: " + DataUtils::varToString(transferSocket))
      //char *ip = inet_ntoa(dataAddress.sin_addr);
      //LOG_INFO(ip);
      NON_BLOCK(transferSocket);
      TcpClient * client = new TcpClient(
        this,
        transferSocket,
        dataAddress,
        BASIC);
      clients.push_back(client);
      clientAdded = true;
    }
    sockaddr_in imageAddress;
    transferSocket = accept(
      createImageSocket,
      (sockaddr*) &imageAddress,
      &addrlen);
    if (transferSocket > 0) {
      LOG_INFO(
        "Accepted image client with socket: " + DataUtils::varToString(transferSocket))
      NON_BLOCK(transferSocket);
      TcpClient * client = new TcpClient(
        this,
        transferSocket,
        imageAddress,
        IMAGE);
      clients.push_back(client);
      clientAdded = true;
    }
  }
  return clientAdded;
}
