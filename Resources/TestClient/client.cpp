#include <iostream>
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
#include <cstdlib>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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

#define SOCKET_PORT "20011"
#define SOCKET_ADR "127.0.0.1"

using namespace std;
using namespace cv;

int charToHex(char ch)
{
  unsigned int res;
  std::stringstream ss;
  ss << ch;
  ss >> std::hex >> res;
  return res;
}

char* hexStringToByteArray(const string& s) {
  int len = s.length();
  char* data = new char[len / 2];
  if(len % 2 == 0)
  { 
    for (int i = 0; i < len; i += 2) {
      std::stringstream ss;
      char temp;
      ss << s[i] << s[i+1];
      ss >> std::hex >> temp; 
      data[i/2] = temp;
    }
  }
  return data;
}

int main()
{
  /* Making the client */
  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;

  char buffer[16192];

  portno = atoi(SOCKET_PORT);

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) 
      return -1;

  server = gethostbyname(SOCKET_ADR);

  if (server == NULL) 
  {
      return -1;
  }

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, 
       (char *)&serv_addr.sin_addr.s_addr,
       server->h_length);
  serv_addr.sin_port = htons(portno);
  try {
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
      cout << "failed to connect" << endl;
        return -1;
      }
  } catch (exception &e) {
    cout << e.what();
  }
  while (true) {
    int size = 0;
    int maxSize = 16192;
    bool noMoreData = false;
    int overallBytesReceived = 0;
    //peak ahead and see if there is more data
    RESET_ERRNO;
    int received = recv(sockfd, buffer, 1, MSG_PEEK);
    if (received < 1) {
      if (!received || (received < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS)) {
        CLOSE(sockfd);
        sockfd = 0;
      }
      noMoreData = true;
      break;
    }
    received = 0;
    char lastReadCh = 0;
    while (lastReadCh != 10 && received < maxSize) {
      RESET_ERRNO;
      int received2 = recv(sockfd, (char*) buffer + received, 1, 0);

      lastReadCh = buffer[received];

      if (!received2 || (received2 < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS)) {
        CLOSE(sockfd);
        sockfd = 0;
        break;
      } else if (ERRNO == EWOULDBLOCK || ERRNO == EINPROGRESS) {
        received2 = 0;
        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        fd_set rset;
        FD_ZERO(&rset);
        FD_SET(sockfd, &rset);
        if (select(sockfd + 1, &rset, 0, 0, &timeout) == -1) {
          CLOSE(sockfd);
          sockfd = 0;
          break;
        }
      }
      received += received2;
      overallBytesReceived += received2;
    }
    buffer[--received] = 0; //mark the end of string
    size += --received; //take out the EOL and CR characters count
    //add to recv queue
    string msg = string(buffer);
    cout << "msg : " << msg << endl;
    ///msg.erase (msg.begin(), msg.begin()+2);
    ///Mat m = Mat(1, msg.length()/2, CV_8UC1);
    ///auto buf= hexStringToByteArray(msg);
    ///std::vector<char> data(buf, buf + size);
    ///cout << "data.size(): " << data.size() << endl;
    ///Mat matrixJprg = imdecode(Mat(data), 1);
    ///imshow("image",matrixJprg);
    ///waitKey(0);
  }
  close(sockfd);
  return 0;
}
