#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

#include "camera_test/tcp_ip.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cv_bridge_test");

  adol::TCPClient client;

  //client.initialize("192.168.1.154", 12346);
  client.initialize("192.168.50.233", 12346);

  cv::VideoCapture cap(0);

  cap.open(0);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cv::Mat frame;
  cap >> frame;
  usleep(8000);
  cap >> frame;
  usleep(8000);
  cap >> frame;
  usleep(8000);

  std::cout << frame.rows << " " << frame.cols << " " << frame.channels() << std::endl;
  
  int img_size = frame.rows*frame.cols*frame.channels();
  uint8_t* img_buffer = new uint8_t[img_size];
  memcpy(img_buffer, frame.data, img_size);

  if (cap.isOpened())
  {
    while (1)
    {
      cap >> frame;

      memcpy(img_buffer, frame.data, img_size);

      int n = client.sendIMGData(img_buffer, img_size);
      //std::cout << n << std::endl;

      //cv::imshow("streaming video", frame);
      if (cv::waitKey(16) == 27)
        break;
    }
  }
  else
  {
    std::cout << "NO FRAME, CHECK YOUR CAMERA!" << std::endl;
  }

  //cv::destroyAllWindows();
  
  delete[] img_buffer;

  client.disconnect();
  return 0;
}