#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/opencv.hpp>

const int PORT = 5678;          //端口号
const char *IP = "192.168.0.100"; //IP地址
struct sockaddr_in server_addr;        //定义服务端的地址结构体

const int Width = 640;  //图片的宽
const int Heigth = 480; //图片的高

void SendImage(const cv::Mat &mat, int sockfd);
cv::Mat GetImageRGB(int connectfd);
void InitAddr();

int main(int argc, char **argv)
{
    int sockfd = socket(AF_INET, SOCK_STREAM, 0); //TCP通信
    if (sockfd == -1)
    {
        perror("socket errno");
        exit(EXIT_FAILURE);
    }
    InitAddr();
    //std::cout<<"socket success"<<std::endl;
    int ret = connect(sockfd,(struct sockaddr *)&server_addr,sizeof(server_addr));
	if(ret == -1)
	{
		perror("connect error");
		exit(1);
	}
    std::cout<<"connect succedd"<<std::endl;
    cv::Mat Srcimage = cv::imread("M8876_640_480.jpg");
    cv::Mat image;
    cv::resize(Srcimage,image,cv::Size(Width,Heigth));
    SendImage(image,sockfd);

    cv::Mat image_from_server = GetImageRGB(sockfd);
    //保存图片
    cv::imwrite("image_from_server.jpg", image_from_server);

    close(sockfd);
    return 0;
}

void InitAddr()
{
    //std::cout<<"Initing"<<std::endl;
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    //需要将服务器端的IP地址进行转换
    int ret = inet_pton(AF_INET, IP, &server_addr.sin_addr.s_addr);
    if (ret == -1)
    {
        perror("inet_pton error");
        exit(EXIT_FAILURE);
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
