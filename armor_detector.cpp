#include <fstream>
#include "number_classifier.cpp"
#include "number_classifier.hpp"
#include "/home/ma/socket/kalman.cpp"
#include "soc.hpp"
#include <chrono>
#include <cstring>
using namespace cv;
using namespace std;
using namespace armor;
class armor_dectectors{
    public:
    Eigen::VectorXd x;
    Eigen::MatrixXd P, RI, Q;
    double sum=0;
    loc rotate_g_c;loc rotate_g_s; loc rotate_o_g;
    cv::Mat pretvec,prervec;
    int preflag=0;int last_frame;
    string model_path = "/home/ma/socket/model/mlp.onnx";
    string label_path = "/home/ma/socket/model/label.txt";
    int count=0;
    
    
    int armor_dectector(Mat frame,int dest_socket,Mat cameraMatrix,Mat distCoeffs){
        armor::NumberClassifier number_classifier(model_path, label_path, 0.5);
        if (count==0){
            initializeKalmanFilter(x, P, RI, Q);
        }
        count++;
        Mat framecopy;
        frame.copyTo(framecopy);
        auto start = std::chrono::high_resolution_clock::now();
        vector<vector<Point2f>> pixelpoints = Find_angle_points(frame);
        Armor myarmor;
        vector<cv::Mat>tvecs;vector<cv::Mat>rvecs;  //初始化tvec,rvec
        int list_len = pixelpoints.size(); 
        for (int i = 0; i < list_len; i++) {
            vector<Armor> armorlist;
            pnpsolver(pixelpoints[i],tvecs,rvecs,cameraMatrix , distCoeffs);//pnp解算
            //将找到的脚点赋值并推进armorlist进行图像识别
            myarmor.left_light.bottom = pixelpoints[i][0];
            myarmor.left_light.top = pixelpoints[i][1];
            myarmor.right_light.top = pixelpoints[i][2];
            myarmor.right_light.bottom = pixelpoints[i][3];
            myarmor.type = ArmorType::SMALL;
            armorlist.push_back(myarmor);
            number_classifier.ExtractNumbers(frame,armorlist);
            number_classifier.Classify(armorlist);
            for (int j=0;j<armorlist.size();j++ ){//每帧进行遍历，当结果不为nagative时，输出结果
                if (strncmp(armorlist[j].classification_result.c_str(), "negative", 8) != 0){
                    // std::cout<<"revc:("<<rvecs[j].at<double>(0,0)<<","<<rvecs[j].at<double>(1,0)<<","<<rvecs[j].at<double>(2,0)<<")"<<std::endl;
                    std::cout<<"第"<<count<<"帧:"<<armorlist[j].classification_result<<" ";  //输出识别结果
                    cv::Mat R;Rodrigues(rvecs[j],R); Mat aular(3,1,CV_16F);aular=rotationMatrixToEulerAnglesMat(R);//将camera下的rvecs转换为欧拉角
                    // std::cout<<"aular:("<<aular.at<double>(0,0)<<","<<aular.at<double>(1,0)<<","<<aular.at<double>(2,0)<<")"<<std::endl;
                    loc odom=camera_to_odom(tvecs[j],aular,rotate_g_c,rotate_g_s,rotate_o_g,framecopy);  //获取odom系下的位姿
                    // std::cout<<"odom:("<<odom.P.x<<","<<odom.P.y<<","<<odom.P.z<<","<<odom.G.roll<<","<<odom.G.pitch<<","<<odom.G.yaw<<")"<<std::endl;
                    // std::string odomstr=convertcenterToString(odom);send_txt_msg(dest_socket,odomstr);
                    cv::Mat tmat = cv::Mat::zeros(1, 3, CV_64F);tmat.at<double>(0, 0) = odom.P.x;tmat.at<double>(0, 1) = odom.P.y;tmat.at<double>(0, 2) = odom.P.z;
                    cv::Mat rmat = cv::Mat::zeros(1, 3, CV_64F);rmat.at<double>(0, 0) = odom.G.roll;rmat.at<double>(0, 1) = odom.G.pitch;rmat.at<double>(0, 2) = odom.G.yaw;
                    if (preflag==1&& abs(count-last_frame)<=53  && abs(count-last_frame)>0){
                        loc result=calculate_revolve_center(tmat,rmat,pretvec,prervec,count,last_frame,rotate_g_c,rotate_o_g,framecopy);//需要调整
                        if (result.P.x!=0){  //向服务器输出结果
                            predict(x, P,  Q);//卡尔曼预测阶段
                            Eigen::VectorXd measurement=Eigen::VectorXd::Zero(3);
                            measurement<<result.P.x,result.P.y,result.P.z;
                            if (measurement.norm() != 0) {  //卡尔曼更新阶段
                                update(x, P,  RI, measurement);
                                std::cout<<"predict"<<x[0]<<x[1]<<x[2]<<std::endl;
                                std::string predictstr=predict_msg(x[0],x[1],x[2]);send_txt_msg(dest_socket,predictstr);//发送预测信息
                            }  
                            // std::string resultstr=convertcenterToString(result);send_txt_msg(dest_socket,resultstr);
                            pretvec=tmat;prervec=rmat;
                            last_frame=count;
                        }
                        
                        // imshow("framecpy",framecopy);
                    }
                    else if (preflag==0){
                        pretvec=tmat;prervec=rmat;preflag=1;
                        last_frame=count;
                    }
                    line(frame,pixelpoints[j][0],pixelpoints[j][2],cv::Scalar(0,0,255),2);
                    line(frame,pixelpoints[j][1],pixelpoints[j][3],cv::Scalar(0,0,255),2);
                    putText(frame, to_string(0),pixelpoints[j][0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
                    putText(frame, to_string(1),pixelpoints[j][1], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
                    putText(frame, to_string(2),pixelpoints[j][2], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
                    putText(frame, to_string(3),pixelpoints[j][3], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
                }
            }
        }
        cv::imshow("frame",frame);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        // std::cout << "时间: " << duration.count() << " 秒" << std::endl;sum+=duration.count();
        // waitKey(300);
        double ave=sum/count;
        std::cout<<"ave:"<<ave;
        // cout<<sizeof(armorlist)<<endl;
        cv::destroyAllWindows();
        return 0;
        }
    };