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
using namespace std;

const int PORT = 5678; //port
struct sockaddr_in server_addr, client_addr; //define struct for address of client & server
const int Width = 640;
const int Heigth = 480;
cv::Mat GetImageRGB(int sockfd);
// void SendImage(const cv::Mat &mat, int sockfd);
void InitAddr(int sokckfd);

struct one_bbox{
  uint16_t xmin;
  uint16_t ymin;
  uint16_t xmax;
  uint16_t ymax;
  uint16_t label;
  uint16_t confidence; //confidence*1000, since memory copy for float is a trouble
};

void bboxes2bytes(const int *datasize, one_bbox *bboxes, uint8_t *bytes_arr){
  // cout<<"datasize="<< *datasize<<endl;
  int total_bytes = (*datasize) * sizeof(one_bbox);
  uint8_t check_sum = 0;
  for(int i=0 ; i<3 ; i++) bytes_arr[i] = 0xAB; // Header
  bytes_arr[3] = (total_bytes >> 8) & 0xFF; // Bytes
  bytes_arr[4] = (total_bytes >> 0) & 0xFF; // Bytes

  // Data
  int bytes_arr_index = 5;
  for(int i=0 ; i< *datasize ; i++){
    bytes_arr[bytes_arr_index] = (bboxes[i].xmin >> 8) & 0xFF;
    bytes_arr[bytes_arr_index+1] = (bboxes[i].xmin >> 0) & 0xFF;//
    bytes_arr[bytes_arr_index+2] = (bboxes[i].ymin >> 8) & 0xFF;
    bytes_arr[bytes_arr_index+3] = (bboxes[i].ymin >> 0) & 0xFF;//
    bytes_arr[bytes_arr_index+4] = (bboxes[i].xmax >> 8) & 0xFF;
    bytes_arr[bytes_arr_index+5] = (bboxes[i].xmax >> 0) & 0xFF;//
    bytes_arr[bytes_arr_index+6] = (bboxes[i].ymax >> 8) & 0xFF;
    bytes_arr[bytes_arr_index+7] = (bboxes[i].ymax >> 0) & 0xFF;//
    bytes_arr[bytes_arr_index+8] = (bboxes[i].label >> 8) & 0xFF;
    bytes_arr[bytes_arr_index+9] = (bboxes[i].label >> 0) & 0xFF;//
    bytes_arr[bytes_arr_index+10] = (bboxes[i].confidence >> 8) & 0xFF;
    bytes_arr[bytes_arr_index+11] = (bboxes[i].confidence >> 0) & 0xFF;//

    check_sum += (bboxes[i].confidence >> 0) & 0xFF; //check_sum
    bytes_arr_index += sizeof(one_bbox);

    // cout<<i<<" " <<bboxes[i].xmin<<" "<<bboxes[i].xmax<<" "
    //                   <<bboxes[i].ymin<<" "<<bboxes[i].ymax<<" "
    //                   <<bboxes[i].label<<" "<<bboxes[i].confidence<<endl;
  }
  bytes_arr[bytes_arr_index] = check_sum;
}

template <typename T>
void SendResultPkg(const T &results, const vector<cv::Mat> &images, int sockfd){
  auto& result = results[0];
  int num = result.bboxes.size();
  one_bbox bboxes[num];
  for (auto i = 0u; i < result.bboxes.size(); ++i) {
    auto& box = result.bboxes[i];
    uint8_t label = box.label + 1;

    float16_t fxmin = box.x * images[0].cols;
    float16_t fymin = box.y * images[0].rows;

    float16_t fwidth = box.width * images[0].cols;
    float16_t fheight = box.height * images[0].rows;

    float16_t fxmax = fxmin + fwidth;
    float16_t fymax = fymin + fheight;
    float16_t confidence = box.score;

    int xmin = round(fxmin * 100.0) / 100.0;
    int ymin = round(fymin * 100.0) / 100.0;
    int xmax = round(fxmax * 100.0) / 100.0;
    int ymax = round(fymax * 100.0) / 100.0;

    xmin = min(max(xmin, 0), images[0].cols);
    xmax = min(max(xmax, 0), images[0].cols);
    ymin = min(max(ymin, 0), images[0].rows);
    ymax = min(max(ymax, 0), images[0].rows);

    bboxes[i].xmin = static_cast<uint16_t>(xmin);
    bboxes[i].xmax = static_cast<uint16_t>(xmax);
    bboxes[i].ymin = static_cast<uint16_t>(ymin);
    bboxes[i].ymax = static_cast<uint16_t>(ymax);
    bboxes[i].confidence = static_cast<uint16_t>(confidence*1000);
    bboxes[i].label = static_cast<uint16_t>(label);
  }

  // encode pkg
  uint8_t bytes_arr[sizeof(bboxes)+6]; //6:Header*3+Bytes*2+checksum*1
  bboxes2bytes(&num, bboxes, bytes_arr);

  // send pkg via socket
  int ret = send (sockfd, bytes_arr, sizeof(bboxes)+6, MSG_NOSIGNAL );
  if(ret == -1){
    perror("send error");
    exit(EXIT_FAILURE);
  }
}

template <typename FactoryMethod, typename ProcessResult>
int socket_inference(const FactoryMethod& factory_method,
                     const ProcessResult& process_result){
  auto model = factory_method();
  auto batch = 1;
  vector<cv::Mat> images(batch);

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
    cout<<"waiting"<<endl;
    int connectfd = accept(sockfd, (struct sockaddr *)&client_addr, &client_len);
    if (connectfd == -1){
      perror("accept errno");
      exit(1);
    }

    printf("client ip:%s port:%d\n",
           inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, client_IP, 1024),
           ntohs(client_addr.sin_port));

    while(true){
      // read data from connectfd
      cv::Mat image = GetImageRGB(connectfd);

      // DPU predict
      images[0] = image;
      auto results = model->run(images);

      // return result to client
      SendResultPkg(results, images, connectfd);
    }
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
      cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
      exit(EXIT_FAILURE);
    }
  }

  outputMat = cv::Mat(Heigth, Width, CV_8UC3,bufferData).clone(); // make a copy
  return outputMat;
}


// void SendImage(const cv::Mat &mat, int sockfd){
//   int ret = send (sockfd, mat.data, 3 * mat.cols * mat.rows, MSG_NOSIGNAL );
//   if(ret == -1){
//     perror("send error");
//     exit(EXIT_FAILURE);
//   }
// }



int main(int argc, char* argv[]) {
  string model = argv[1];

  return socket_inference(
      [model] {
        return vitis::ai::EfficientDetD2::create(model);
      },
      process_result);
}
