#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include "convert.hpp"
using namespace cv;
using namespace std;
struct lightstick{
    Point2f midpoint_up,midpoint_down;
    double length;
    double angle;
    int flag=0;
};
void printVector(const vector<Point2f>& vec) {
    for (const auto& point : vec) {
        cout << "(" << point.x << ", " << point.y << ") ";
    }
    cout << endl;
}



int pairs (vector<vector<Point2f>> &a,struct lightstick b[],int length ){  //判断灯条是否平行并两两配对返回
    int count=0;
    for (int i=0;i<length;i++){
        for (int j=i+1;j<length;j++){
            if (b[j].flag==1||b[i].flag==1){
            continue;
            }
            double angle_diff=abs(b[i].angle-b[j].angle);
            double temp;
            double longs=norm(b[i].midpoint_up-b[j].midpoint_up);
            double shorts=norm(b[i].midpoint_down-b[i].midpoint_up);
            longs<shorts? temp=longs,longs=shorts,shorts=temp:0;
            double ratio=norm(b[i].midpoint_up-b[i].midpoint_down)/norm(b[j].midpoint_up-b[j].midpoint_down);//两灯条长度比
            if (angle_diff<6 && (longs/shorts)<3 && (longs/shorts)>1.5 && ratio<1.8 &&ratio>0.5){  //判定平行与长宽比
                a[count].push_back(b[i].midpoint_up);a[count].push_back(b[i].midpoint_down);
                a[count+1].push_back(b[j].midpoint_up);a[count+1].push_back(b[j].midpoint_down);
                b[i].flag=1;
                b[j].flag=1;
                count+=2;
            }
        }
    }
    return count/2;
}





std::vector<vector<cv::Point2f>> Find_angle_points(cv::Mat img) {     //寻找配对好的灯条的角点
    cv::Mat image;
    img.copyTo(image);
    cv::Mat gray, binary, eroded, gammaed;
    cv::Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    float gamma = 1.5;
    unsigned char lut[256];
    for (int i = 0; i < 256; i++) {
        lut[i] = cv::saturate_cast<uchar>(pow((float)(i / 255.0), gamma) * 255.0);
    }
    LUT(img, cv::Mat(1, 256, CV_8U, lut), gammaed);
    cvtColor(gammaed, gray, COLOR_BGR2GRAY);
    // imshow("gray",gray);
    threshold(gray, binary, 120, 255, THRESH_BINARY);
    // cv::imshow("binary1",binary);
    erode(binary, eroded, element); // 预处理
    dilate(eroded, eroded, element);
    vector<vector<cv::Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(eroded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    struct lightstick pixellists[contours.size()];
    for (size_t i = 0; i < contours.size(); i++) {
        // 计算轮廓的边界框
        RotatedRect minrect = minAreaRect(contours[i]);
        Point2f rectPoints[4];
        minrect.points(rectPoints);
        
        if ((minrect.size.width / minrect.size.height <= 1.4&& minrect.size.width / minrect.size.height >= 4)||contours[i].size()<10) {  //筛选灯条长度
            continue;
        }
        Point2f midPoint1, midPoint2;
        if (norm(rectPoints[0] - rectPoints[1]) < norm(rectPoints[1] - rectPoints[2])) {
            midPoint1 = (rectPoints[0] + rectPoints[1]) * 0.5;
            midPoint2 = (rectPoints[2] + rectPoints[3]) * 0.5;
        } else {
            midPoint1 = (rectPoints[1] + rectPoints[2]) * 0.5;
            midPoint2 = (rectPoints[3] + rectPoints[0]) * 0.5;
        }
        // 计算短边的中点坐标
        // circle(image, midPoint1, 2, Scalar(0, 0, 255), -1);
        // circle(image, midPoint2, 2, Scalar(0, 0, 255), -1);
        if (midPoint1.y < midPoint2.y) {
            pixellists[i].midpoint_up = midPoint1;
            pixellists[i].midpoint_down = midPoint2;
        }
        else{
            pixellists[i].midpoint_up = midPoint2;
            pixellists[i].midpoint_down = midPoint1;
        }
        pixellists[i].length = norm(midPoint1 - midPoint2);
        pixellists[i].angle = minrect.angle;
    }
    vector<vector<Point2f>> temp(contours.size() * 2); // 初始化temp
    int length = sizeof(pixellists) / sizeof(pixellists[0]); // 修正数组长度计算
    int num = pairs(temp, pixellists, length);
    vector<vector<Point2f>> result(num); // 初始化result
    for (int i = 0; i < num; i++) {
        temp[i * 2].insert(temp[i * 2].end(), temp[i * 2 + 1].begin(), temp[i * 2 + 1].end());
        vector<Point2f> pixel=temp[i * 2];
        std::sort(pixel.begin(), pixel.end(), [](const Point2f& a, const Point2f& b) {
            return a.x < b.x;
        });
        // printVector(pixel);
        Point2f pixel1[4];
        if (pixel[0].y > pixel[1].y) { // 调整四个点顺序
            pixel1[0] = pixel[0];
            pixel1[1] = pixel[1];
        } else {
            pixel1[0] = pixel[1];
            pixel1[1] = pixel[0];
        }
        if (pixel[2].y > pixel[3].y) {
            pixel1[2] = pixel[3];
            pixel1[3] = pixel[2];
        } else {
            pixel1[2] = pixel[2];
            pixel1[3] = pixel[3];
        }
        vector<Point2f> pixelVector(pixel1, pixel1 + 4);
        // for (const auto& point : pixelVector) {
        //     std::cout << "Point: (" << point.x << ", " << point.y << ")" << std::endl;
        // }
        result[i] = pixelVector;
    }
    // cv::imshow("result", image);
    return result;
}
//pnp解算
void pnpsolver(vector<Point2f> pixelVector,vector<cv::Mat> &tvecs,vector<cv::Mat> &rvecs,Mat camera_matrix,Mat dist_coeffs){//pnp解算函数，输入四个角点，返回tvec,rvec
    const float SMALL_ARMOR_WIDTH  = 0.135;
    const float SMALL_ARMOR_HEIGHT = 0.055;
    const float LARGE_ARMOR_WIDTH  = 0.225;
     float LARGE_ARMOR_HEIGHT = 0.055;
    // x 垂直装甲板向内，从左下角开始顺时针
    const std::vector<cv::Point3f> SMALL_ARMOR_POINTS = {
        { 0, +SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2},
        { 0, +SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
        { 0, -SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
        { 0, -SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2}
    };
    // x 垂直装甲板向内，从左下角开始顺时针
    const std::vector<cv::Point3f> LARGE_ARMOR_POINTS = {
        { 0, +LARGE_ARMOR_WIDTH / 2, -LARGE_ARMOR_HEIGHT / 2 },
        { 0, +LARGE_ARMOR_WIDTH / 2, +LARGE_ARMOR_HEIGHT / 2 },
        { 0, -LARGE_ARMOR_WIDTH / 2, +LARGE_ARMOR_HEIGHT / 2 },
        { 0, -LARGE_ARMOR_WIDTH / 2, -LARGE_ARMOR_HEIGHT / 2 }
    };
// clang-format on
    // cv::Mat camera_matrix = (Mat_<double>(3, 3) << 2102.080562187802, 0, 689.2057889332623, //相机内参
	// 0, 2094.0179120166754, 496.6622802275393,
	// 0, 0, 1);
    // cv::Mat dist_coeffs = (Mat_<double>(5, 1) <<-0.06478109387525666 ,0.39036067923005396 ,-0.0042514793151166306 , 0.008306749648029776  , -1.6613800909405605);  //畸变参数
    cv::Mat rotation_vector;
    cv::Mat translation_vector;
    cv::solvePnP(SMALL_ARMOR_POINTS, pixelVector, camera_matrix, dist_coeffs, \
		rotation_vector, translation_vector, cv::SOLVEPNP_ITERATIVE);

    // std::cout << "Rotation Vector " << endl << rotation_vector << endl << endl;
	// std::cout << "Translation Vector" << endl << translation_vector << endl << endl;
    tvecs.push_back(translation_vector);
    rvecs.push_back(rotation_vector);
}
//计算线速度
vector<double> calculate_linear_velocity(Mat tvec1,Mat tvec2,int time1,int time2){
    int delta_time = time2 - time1;
    double delta_x = abs(tvec2.at<double>(0, 1) - tvec1.at<double>(0, 1));
    double delta_y= abs(tvec2.at<double>(0, 2) - tvec1.at<double>(0, 2));
    double delta_z = abs(tvec2.at<double>(0, 3) - tvec1.at<double>(0, 3));
    vector<double> velocity;
    velocity.push_back(delta_x/delta_time);
    velocity.push_back(delta_y/delta_time);
    velocity.push_back(delta_z/delta_time);
    return velocity;
}
//计算角速度函数
vector<double> calculate_Angular_velocity(Mat rvec1,Mat rvec2,int time1,int time2){
    int delta_time = time2 - time1;
    double delta_roll = abs(rvec2.at<double>(0, 1) - rvec1.at<double>(0, 1));
    double delta_pitch= abs(rvec2.at<double>(0, 2) - rvec1.at<double>(0, 2));
    double delta_yaw = abs(rvec2.at<double>(0, 3) - rvec1.at<double>(0, 3));
    vector<double> velocity;
    velocity.push_back(delta_roll/delta_time);
    velocity.push_back(delta_pitch/delta_time);
    velocity.push_back(delta_yaw/delta_time);
    return velocity;
}
//计算旋转中心函数
loc calculate_revolve_center(const Mat& tvec1, const Mat& rvec1, const Mat& tvec2, const Mat& rvec2,int time1,int time2,loc rotate_g_c,loc rotate_o_g,Mat image){
    vector<double> linear_velocity=calculate_linear_velocity(tvec1, tvec2, time1, time2);
    vector<double> angular_velocity=calculate_Angular_velocity(rvec1, rvec2, time1, time2);
    double norm_linear=norm(linear_velocity);double norm_angular=norm(angular_velocity);
    double r=norm_linear/norm_angular;
    if (r>0.1&&r<1.3){
        std::cout<<"r is "<<r<<std::endl;
        loc center=armor2odom(r,tvec2,rvec2,rotate_g_c,rotate_o_g);
        std::cout<<"center is("<<center.P.x<<","<<center.P.y<<","<<center.P.z<<")"<<std::endl;
        // circle(image,Point(center.P.x,center.P.y),5,Scalar(0,0,255),-1);
        return center;
    }
    loc null;
    return null;
    
}


