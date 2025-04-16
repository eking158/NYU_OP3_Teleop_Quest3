#include <cstring>
#include "camera_test/tcp_ip.h"

using namespace adol;

TCPClient::TCPClient()
{
  memset(&pose_msg_, '0', sizeof(pose_msg_));
  memset(&server_addr_, '0', sizeof(server_addr_));
  sockfd_ = 0;
}

TCPClient::~TCPClient()
{  }

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
  return recv_packet_length_;
}

int TCPClient::sendIMGData(uint8_t *img_data, int img_data_size)
{
  int bytes = write(sockfd_, img_data, img_data_size);
  return bytes;
}

void TCPClient::disconnect()
{
  close(sockfd_);
}