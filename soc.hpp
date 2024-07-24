#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <thread>
#include <arpa/inet.h>
#include <map>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <unordered_map>
#include "pnp.hpp"
#define BUFFER_SIZE 10240

#pragma pack(1)
typedef struct {
    unsigned short Start=0x0D00 ;   // 起始位 0~1
    unsigned short MessageType;  // 消息类型 2~3
    unsigned int DataID=0;   // 数据ID 4~7
    unsigned int DataTotalLenth=0; // 数据总长度 8~11
    unsigned int Offset=0;  // 偏移量 12~15
    unsigned int DataLenth=0;  // 数据长度 16~19
    unsigned char Data[10218];  // 数据内容 20~10237
    unsigned short End=0x0721 ;   // 结束位 10238~10239
} MessageBuffer;

enum MessageType {
    STRING_MSG = 0x0000,
    IMAGE_MSG = 0x1145,
    CAMERA_INFO = 0x1419,
    TRANSFORM = 0x1981,
    TRANSFORM_REQUEST = 0x1982,
};

typedef struct {
    double CameraMatrix[9];
    double DistortionCoefficients[5];
} CameraInfoData;

typedef struct {
    double Translation[3];
    double Rotation[4];
} TransformData;

typedef struct {
    char From[10218 / 2];
    char To[10218 / 2];
} TransformRequestData;
#pragma pack()

class Application {
    public:
    unsigned char* receive_and_decode(MessageBuffer& message);
    cv::Mat receive_image(MessageBuffer& buffer);
    std::string handle_string_msg(MessageBuffer& buffer);
    private:
    std::unordered_map<unsigned int, std::vector<unsigned char>> data_temp;
    };
unsigned char* Application::receive_and_decode(MessageBuffer& message) {
    unsigned int offset = message.Offset;
    unsigned int length = message.DataLenth;
    unsigned int total_length = message.DataTotalLenth;
    unsigned int dataID = message.DataID;

    if (data_temp.find(dataID) == data_temp.end()) {
        data_temp[dataID] = std::vector<unsigned char>(total_length);
    }
    std::memcpy(data_temp[dataID].data() + offset, message.Data, length);
    if (offset + length >= total_length) {
        unsigned char* data = new unsigned char[total_length];
        std::memcpy(data, data_temp[dataID].data(), total_length);
        data_temp.erase(dataID);
        return data;
    } 
    else {
        return nullptr;
    }
}

void deserializeMessage(unsigned char* buffer, MessageBuffer& message) {
    memcpy(&message.Start, buffer, sizeof(message.Start));
    memcpy(&message.MessageType, buffer + 2, sizeof(message.MessageType));
    memcpy(&message.DataID, buffer + 4, sizeof(message.DataID));
    memcpy(&message.DataTotalLenth, buffer + 8, sizeof(message.DataTotalLenth));
    memcpy(&message.Offset, buffer + 12, sizeof(message.Offset));
    memcpy(&message.DataLenth, buffer + 16, sizeof(message.DataLenth));
    memcpy(&message.Data, buffer + 20, message.DataLenth);
    memcpy(&message.End, buffer + 20 + message.DataLenth, sizeof(message.End));
}
std::string Application::handle_string_msg(MessageBuffer& buffer) {
    return std::string(reinterpret_cast<char*>(buffer.Data), buffer.DataLenth);
}
loc receive_transform_data(MessageBuffer buffer){
    double translation[3];double rotation[4];loc dst;
    memcpy(translation, buffer.Data, sizeof(translation));
    memcpy(rotation, buffer.Data + sizeof(translation), sizeof(rotation));
    dst.P.x=translation[0];dst.P.y=translation[1];dst.P.z=translation[2];
    dst.Q.i=rotation[0];dst.Q.j=rotation[1];dst.Q.k=rotation[2];dst.Q.w=rotation[3];
    dst.G=convert_to_gesture(dst.Q);
    return dst;
}
void send_transform_request(int client_socket){
    MessageBuffer buffer;
    buffer.MessageType=0x1982;buffer.DataTotalLenth=buffer.DataLenth=sizeof(buffer.Data);
    char gimbal[]="Gimbal";
    char camera[]="Camera";
    char odom[]="Odom";
    memset(buffer.Data,0,buffer.DataLenth);
    buffer.Start = 0x0D00; buffer.End = 0x0721;
    // memcpy(buffer.Data,gimbal,sizeof(gimbal)); memcpy(buffer.Data+5109,camera,sizeof(camera));
    memcpy(((TransformRequestData*)buffer.Data)->From,odom,sizeof(gimbal));
    memcpy(((TransformRequestData*)buffer.Data)->To,gimbal,sizeof(camera));
    
    // send(client_socket,&buffer,10240,0);
    write(client_socket,&buffer,10240);
    memset(buffer.Data,0,buffer.DataLenth);
    buffer.Start = 0x0D00; buffer.End = 0x0721;
    // memcpy(buffer.Data,odom,sizeof(odom)); memcpy(buffer.Data+5109,gimbal,sizeof(gimbal));
    memcpy(((TransformRequestData*)buffer.Data)->From,odom,sizeof(odom));
    memcpy(((TransformRequestData*)buffer.Data)->To,gimbal,sizeof(gimbal));
    // send(client_socket,&buffer,10240,0);
    write(client_socket,&buffer,10240);
}
    

void send_txt_msg(int client_socket,std::string msg){
    MessageBuffer buffer = {};
        buffer.MessageType = 0x0000;
        buffer.DataID = 0;
        buffer.Offset = 0;
        buffer.DataLenth = msg.length();
        buffer.DataTotalLenth = msg.length();
        memcpy(buffer.Data, msg.c_str(), msg.length());
        send(client_socket, &buffer, sizeof(MessageBuffer), 0);
}
void send_txt(int client_socket) {
    std::string message;
    while (true) {
        std::getline(std::cin, message);
        if (message == "exit") {
            break;
        }
        MessageBuffer buffer = {};
        buffer.MessageType = 0x0000;
        buffer.DataID = 0;
        buffer.Offset = 0;
        buffer.DataLenth = message.length();
        buffer.DataTotalLenth = message.length();
        if (message.length() > sizeof(buffer.Data)) {
            std::cerr << "Message too long" << std::endl;
            continue;
        }
        memcpy(buffer.Data, message.c_str(), message.length());
        send(client_socket, &buffer, sizeof(MessageBuffer), 0);
    }
}

cv::Mat Application::receive_image( MessageBuffer&buffer) {
    unsigned char *data=receive_and_decode(buffer);
    if (data!=nullptr){
        std::vector<uchar> img_data(data, data + buffer.DataTotalLenth);
        cv::Mat img = cv::imdecode(img_data, cv::IMREAD_COLOR);
        // std::string cnt=std::to_string(buffer.DataID);
        delete[] data;
        return img;
    }
    return cv::Mat();

}

void send_image(const cv::Mat& image, int client_socket) {
    std::vector<unsigned char> encoded_image;
    cv::imencode(".jpg", image, encoded_image);
    int total_image_size = encoded_image.size();
    int count=0;
    int bytes_sent = 0;
    while (bytes_sent < total_image_size) {
        MessageBuffer buffer = {};
        buffer.Start = 0x0D00;
        buffer.MessageType = 0x1145;
        buffer.DataID = count;
        buffer.DataTotalLenth = total_image_size;
        buffer.Offset = bytes_sent;
        buffer.DataLenth = std::min(10218, total_image_size - bytes_sent);
        buffer.End = 0x0721;
        memcpy(buffer.Data, encoded_image.data() + bytes_sent, buffer.DataLenth);
        int bytes = send(client_socket, &buffer, sizeof(MessageBuffer), 0);
        count++;
        if (bytes < 0) {
            perror("send");
            break;
        }
        bytes_sent += buffer.DataLenth;
    }
}

void receive_all(int client_socket) {
    std::vector<uchar> combuffer;
    Application app;
    
    while(true){
        unsigned char recvbuffer[10240]={0};
        int num=recv(client_socket, recvbuffer,10240-combuffer.size(), 0);
        if (num==-1){
            std::cout<<"disconnect";
            break;
        }
        if (num==0){
            std::cout<<"can not get data";
            break;
        }
        combuffer.insert(combuffer.end(), recvbuffer, recvbuffer+num);
        while(combuffer.size()>=10240){
            MessageBuffer buffer;
            deserializeMessage(combuffer.data(),buffer);
            size_t messageSize = sizeof(MessageBuffer) - sizeof(buffer.Data) + buffer.DataLenth;
            if (combuffer.size() < messageSize) {
                break;
            }
            combuffer.clear();
            if (buffer.MessageType==0x0000){
                 std::string str = app.handle_string_msg(buffer);
                std::cout << "Received string message: " << str << std::endl;
            }
            else if(buffer.MessageType==0x1145){
                cv::Mat img=app.receive_image( buffer);
                if (!img.empty()) {
                    std::cout<<"get image successfully"<<std::endl;
                    cv::imshow("Image", img);
                    cv::waitKey(0);
                    
                }
            }

        }
    }
}


