#include <cstring>
#include <stdio.h>
#include "op3_AR_whole_body_ctrl_module/tcp_ip.h"

using namespace adol;

TCPClient::TCPClient()
{
  memset(&pose_msg_, '0', sizeof(pose_msg_));
  memset(&server_addr_, '0', sizeof(server_addr_));
  sockfd_ = 0;
}

bool TCPClient::initialize(std::string ip_address, unsigned short port)
{
  if ((sockfd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    printf("\n Error : Could not create socket \n");
    return false;
  }
  
  bzero(&server_addr_, sizeof(server_addr_));

  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(port);

  server_addr_.sin_addr.s_addr = inet_addr(ip_address.c_str());

  // if (inet_pton(AF_INET, ip_address.c_str(), &server_addr_.sin_addr) <= 0)
  // {
  //   printf("\n inet_pton error occured\n");
  //   return false;
  // }

  if (connect(sockfd_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0)
  {
    printf("\n Error : Connect Failed \n");
    return false;
  }

  return true;
}

int TCPClient::receiveData()
{
  recv_packet_length_ = read(sockfd_, pose_msg_, sizeof(pose_msg_));
  // printf("%f %f %f %f %f %f %f %f %f\n",
  //  pose_msg_[0],
  //  pose_msg_[1],
  //  pose_msg_[2],
  //  pose_msg_[3],
  //  pose_msg_[4],
  //  pose_msg_[5],
  //  pose_msg_[6],
  //  pose_msg_[7],
  //  pose_msg_[8]);

  return recv_packet_length_;
}

int TCPClient::sendData()
{
  int bytes = write(sockfd_, com_state_, sizeof(com_state_));
  return bytes;
}