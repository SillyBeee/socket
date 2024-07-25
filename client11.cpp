#define PORT 4399
#define BUFFER_SIZE 10240
#include "armor_detector.cpp"
//3端口客户端
int main() {
    int received_transform_count=0;
    cv::Mat_<double> camera_matrix = cv::Mat_<double>::zeros(3, 3);
    cv::Mat_<double> dist_coeffs = cv::Mat_<double>::zeros(5, 1);
    armor_dectectors armor_dectector;
    char type[2];
    int sock = 0;
    struct sockaddr_in serv_addr;
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(4399);
    if (inet_pton(AF_INET, "10.2.20.66", &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed" << std::endl;
        return -1;
    }
    std::vector<uchar> combuffer;
    Application app;
    
    while(received_transform_count<2){
        if (received_transform_count == 0){
            send_transform_request(sock);
            sleep(2);
        }
        unsigned char recvbuffer[10240]={0};
        int num=recv(sock, recvbuffer,10240-combuffer.size(), 0);
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
                if (camera_matrix.at<double>(0,0)==0){
                    std::cout<<"havent get camera info yet"<<std::endl;
                }
                if (!img.empty()&&camera_matrix.at<double>(0,0)!=0) {
                    std::cout<<"get image successfully"<<std::endl;
                    armor_dectector.armor_dectector(img,sock,camera_matrix,dist_coeffs);
                    // cv::imshow("Image", img);
                    cv::waitKey(1);
                    
                }
            }
            else if (buffer.MessageType == 0x1419){
                for (int i=0;i<3;i++){
                    for (int j=0;j<3;j++){
                        camera_matrix.at<double>(i,j)=buffer.Data[i*3+j];
                    }
                }
                for (int i = 0; i < 5; i++){
                    dist_coeffs.at<double>(i) = buffer.Data[i + 9];
                }
                std::cout<<"successfully get camera info"<<std::endl;
            }
            else if (buffer.MessageType == 0x1981){
                loc ro_g_c;loc ro_o_g;
                if (received_transform_count == 0){
                    ro_g_c=receive_transform_data(buffer);
                    received_transform_count++;
                    armor_dectector.rotate_g_c=ro_g_c;
                    std::cout<<"get rotation gimbal to camera"<<std::endl;
                }
                else if (received_transform_count == 1){
                    ro_o_g=receive_transform_data(buffer);
                    armor_dectector.rotate_o_g=ro_o_g;
                    std::cout<<"get rotation odom to gimbal"<<std::endl;
                     received_transform_count++;
                }
            }
        }
        
    }
    close(sock);
    std::cout<<"sock close"<<std::endl;
    









    received_transform_count=0;
    armor_dectectors armor_dectector2;
    int sock2 = 0;
    struct sockaddr_in serv_addr2;
    if ((sock2 = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }
    serv_addr2.sin_family = AF_INET;
    serv_addr2.sin_port = htons(5140);
    if (inet_pton(AF_INET, "10.2.20.66", &serv_addr2.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        return -1;
    }

    if (connect(sock2, (struct sockaddr *)&serv_addr2, sizeof(serv_addr2)) < 0) {
        std::cerr << "Connection Failed" << std::endl;
        return -1;
    }
    std::vector<uchar> combuffer2;
    Application app2;
    int get_camera_info=0;
    while(get_camera_info<1){
        uchar recvbuffer[10240]={0};
        int num=recv(sock2, recvbuffer,10240-combuffer2.size(), 0);
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
                if (camera_matrix.at<double>(0,0)==0){
                    std::cout<<"havent get camera info yet"<<std::endl;
                    
                }
                if (!img.empty()&&camera_matrix.at<double>(0,0)!=0) {
                    std::cout<<"get image successfully"<<std::endl;
                    armor_dectector.armor_dectector(img,sock,camera_matrix,dist_coeffs);
                    // cv::imshow("Image", img);
                    cv::waitKey(1);
                    
                }
            }
            else if (buffer.MessageType == 0x1419){
                CameraInfoData data;  memcpy(&data,buffer.Data,sizeof(CameraInfoData));
                for (int i=0;i<3;i++){
                    for (int j=0;j<3;j++){
                        camera_matrix.at<double>(i,j)=data.CameraMatrix[i*3+j];
                    }
                }
                for (int k = 0; k < 5; k++){
                    dist_coeffs.at<double>(k) = data.DistortionCoefficients[k];
                }
                std::cout<<"successfully get camera info"<<std::endl;
                std::cout<<camera_matrix<<std::endl;
                std::cout<<dist_coeffs<<std::endl;
                get_camera_info++;
            }
            else if (buffer.MessageType == 0x1981){
                loc ro_g_c;loc ro_o_g;
                if (received_transform_count == 0){
                    ro_g_c=receive_transform_data(buffer);
                    received_transform_count++;
                    armor_dectector.rotate_g_c=ro_g_c;
                    std::cout<<"get rotation gimbal to camera"<<std::endl;
                }
                else if (received_transform_count == 1){
                    ro_o_g=receive_transform_data(buffer);
                    armor_dectector.rotate_o_g=ro_o_g;
                    std::cout<<"get rotation odom to gimbal"<<std::endl;
                     received_transform_count++;
                }
            }
        }
        
    }
    close(sock2);
    std::cout<<"sock2 close"<<std::endl;
    // std::cout<<camera_matrix<<std::endl;



    received_transform_count=0;
    int sock3 = 0;
    struct sockaddr_in serv_addr3;
    if ((sock3 = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }
    serv_addr3.sin_family = AF_INET;
    serv_addr3.sin_port = htons(8000);
    if (inet_pton(AF_INET, "10.2.20.66", &serv_addr3.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        return -1;
    }

    if (connect(sock3, (struct sockaddr *)&serv_addr3, sizeof(serv_addr3)) < 0) {
        std::cerr << "Connection Failed" << std::endl;
        return -1;
    }
    std::vector<uchar> combuffer3;
    Application app3;
    while(true){
        uchar recvbuffer[10240]={0};
        int num=recv(sock3, recvbuffer,10240-combuffer3.size(), 0);
        if (num==-1){
            std::cout<<"disconnect";
            break;
        }
        if (num==0){
            std::cout<<"can not get data";
            break;
        }
        combuffer3.insert(combuffer3.end(), recvbuffer, recvbuffer+num);
        while(combuffer3.size()>=10240){
            MessageBuffer buffer;
            deserializeMessage(combuffer3.data(),buffer);
            size_t messageSize = sizeof(MessageBuffer) - sizeof(buffer.Data) + buffer.DataLenth;
            if (combuffer3.size() < messageSize) {
                break;
            }
            combuffer3.clear();
            if (buffer.MessageType==0x0000){
                 std::string str = app.handle_string_msg(buffer);
                std::cout << "Received string message: " << str << std::endl;
            }
            else if(buffer.MessageType==0x1145){
                cv::Mat img=app.receive_image( buffer);
                if (camera_matrix.at<double>(0,0)==0){
                    std::cout<<"havent get camera info yet"<<std::endl;
                    
                }
                if (!img.empty()&&camera_matrix.at<double>(0,0)!=0) {
                    std::cout<<"get image successfully"<<std::endl;
                    armor_dectector.armor_dectector(img,sock3,camera_matrix,dist_coeffs);
                    // cv::imshow("Image", img);
                    cv::waitKey(1);
                    
                }
            }
            else if (buffer.MessageType == 0x1419){
                for (int i=0;i<3;i++){
                    for (int j=0;j<3;j++){
                        camera_matrix.at<double>(i,j)=buffer.Data[i*3+j];
                    }
                }
                for (int i = 0; i < 5; i++){
                    dist_coeffs.at<double>(i) = buffer.Data[i + 9];
                }
                std::cout<<"successfully get camera info"<<std::endl;
                get_camera_info++;
            }
            else if (buffer.MessageType == 0x1981){
                loc ro_g_c;loc ro_o_g;
                if (received_transform_count == 0){
                    ro_g_c=receive_transform_data(buffer);
                    received_transform_count++;
                    armor_dectector.rotate_g_c=ro_g_c;
                    std::cout<<"get rotation gimbal to camera"<<std::endl;
                }
                else if (received_transform_count == 1){
                    ro_o_g=receive_transform_data(buffer);
                    armor_dectector.rotate_o_g=ro_o_g;
                    std::cout<<"get rotation odom to gimbal"<<std::endl;
                     received_transform_count++;
                }
            }
        }
        
    }
    close(sock3);
}