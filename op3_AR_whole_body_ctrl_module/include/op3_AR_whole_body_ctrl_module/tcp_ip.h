#ifndef OP3_AR_TCP_IP_H_
#define OP3_AR_TCP_IP_H_

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h> 

namespace adol
{

class TCPClient
{
public:
  TCPClient();
  ~TCPClient();

  int sockfd_;
  int recv_packet_length_;
  std::string server_ip_addr_;

  struct sockaddr_in server_addr_; 

  double pose_msg_[9]; // 6 (RL arm angle) + 2 (head) + 1 (body height)
  double com_state_[2];

  bool initialize(std::string ip_address, unsigned short port);
  int sendData();
  int  receiveData();
  //void sendIMGData(int width, int height, int pixel_bytes, const char* img_data);
  void disconnect();
};

}

#endif /* OP3_AR_TCP_IP_H_ */