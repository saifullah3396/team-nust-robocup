/**
 * @file Utils/include/ClientInfo.h
 *
 * This file defines the struct ClientInfo
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <cstring>
#include <net/if.h>
#include <ifaddrs.h>
#include <string>

using namespace std;

/**
 * Enumeration that defines the possible types of clients
 *
 * @enum TcpClientType
 */
enum TcpClientType
{
  BASIC,
  IMAGE
};

/**
 * @struct ClientInfo
 * @brief Holds information about a connected client
 */
struct ClientInfo
{
  ClientInfo()
  {
  }
  
  ClientInfo(const string& address, const TcpClientType& type) :
    address(address), type(type)
  {
  }
  
  ClientInfo(const sockaddr_in& address, const TcpClientType& type) :
    type(type)
  {
    this->address = inet_ntoa(address.sin_addr);
  }
  
  //! The socket address
  string address;
  
  //! Type of TcpClient
  TcpClientType type;
};
