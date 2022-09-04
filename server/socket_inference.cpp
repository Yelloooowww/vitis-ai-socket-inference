#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>
#include <iostream>
#include <memory>
#include <vitis/ai/efficientdet_d2.hpp>
#include <vitis/ai/demo.hpp>
#include "./process_result.hpp"

const int PORT = 5678; //port
struct sockaddr_in server_addr, client_addr; //define struct for address of client & server
const int Width = 640;
const int Heigth = 480;
cv::Mat GetImageRGB(int sockfd);
void SendImage(const cv::Mat &mat, int sockfd);
void InitAddr(int sokckfd);

template <typename FactoryMethod, typename ProcessResult>
int socket_inference(const FactoryMethod& factory_method,
              const ProcessResult& process_result){
  auto model = factory_method();
  auto batch = 1;
  std::vector<cv::Mat> images(batch);

  char client_IP[1024];
  int sockfd = socket(AF_INET, SOCK_STREAM, 0); //TCP
  if (sockfd == -1){
    perror("socket error");
    exit(EXIT_FAILURE);
  }
  InitAddr(sockfd);

  // waiting for client
  while(true){
    bzero(&client_addr, sizeof(client_addr));
    socklen_t client_len = sizeof(client_addr);
    std::cout<<"waiting"<<std::endl;
    int connectfd = accept(sockfd, (struct sockaddr *)&client_addr, &client_len);
    if (connectfd == -1){
      perror("accept errno");
      exit(1);
    }

    printf("client ip:%s port:%d\n",
           inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, client_IP, 1024),
           ntohs(client_addr.sin_port));

    // read data from connectfd
    cv::Mat image = GetImageRGB(connectfd);

    // DPU predict
    images[0] = image;
    auto results = model->run(images);
    auto image_inference_output = process_result(images[0], results[0], true);

    // return result image to client
    SendImage(image_inference_output,connectfd);
  }
  return 0;
}

void InitAddr(int sockfd){
  bzero(&server_addr, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT);
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  int opt = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  int ret = bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (ret == -1){
    perror("bind errno");
    exit(1);
  }

  ret = listen(sockfd, 10);
  if (ret == -1){
    perror("listen errno");
    exit(1);
  }
}

cv::Mat GetImageRGB(int connectfd){
  cv::Mat outputMat(Heigth, Width, CV_8UC3);
  const int imgSize = 3 * Heigth * Width;

  // recv data would be saved in a buffer
  uint8_t bufferData[imgSize];
  int bytes = 0;
  int i = 0; // count how many byte I have received
  for (i = 0; i < imgSize; i += bytes){
    bytes = recv(connectfd, bufferData + i, imgSize - i, 0);
    if (bytes == -1){
      std::cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
      exit(EXIT_FAILURE);
    }
  }

  outputMat = cv::Mat(Heigth, Width, CV_8UC3,bufferData).clone(); // make a copy
  return outputMat;
}


void SendImage(const cv::Mat &mat, int sockfd){
  int ret = send (sockfd, mat.data, 3 * mat.cols * mat.rows, MSG_NOSIGNAL );
  if(ret == -1){
    perror("send error");
    exit(EXIT_FAILURE);
  }
}



int main(int argc, char* argv[]) {
  std::string model = argv[1];

  return socket_inference(
      [model] {
        return vitis::ai::EfficientDetD2::create(model);
      },
      process_result);
}
