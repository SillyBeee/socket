#include "position_and_gesture.h"
#include "Quaternion.h"
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

gesture convert_to_gesture(Quarternion::Quarternion a){  //四元数转姿态
            double roll=atan2(2*(a.w*a.i+a.k*a.j),(1-2*(a.i*a.i+a.j*a.j)));
            double pitch=asin(2*(a.w*a.j-a.i*a.k));
            double yaw=atan2(2*(a.w*a.k+a.j*a.i),(1-2*(a.j*a.j+a.k*a.k)));
            gesture b(roll,pitch,yaw);
            return b;
        }
Quarternion::Quarternion convert_to_Quarternion(gesture a){  //姿态转四元数
        double w=cos(a.roll/2)*cos(a.pitch/2)*cos(a.yaw/2)+sin(a.roll/2)*sin(a.pitch/2)*sin(a.yaw/2);
        double i=sin(a.roll/2)*cos(a.pitch/2)*cos(a.yaw/2)-cos(a.roll/2)*sin(a.pitch/2)*sin(a.yaw/2);
        double j=cos(a.roll/2)*sin(a.pitch/2)*cos(a.yaw/2)+sin(a.roll/2)*cos(a.pitch/2)*sin(a.yaw/2);
        double k=cos(a.roll/2)*cos(a.pitch/2)*sin(a.yaw/2)-sin(a.roll/2)*sin(a.pitch/2)*cos(a.yaw/2);
        Quarternion::Quarternion b(w,i,j,k);
        return b;
        }
class loc{
    public:
        Quarternion::Quarternion Q;
        pose P;
        gesture G;
        loc():
        Q(0,0,0,0),P(0,0,0),G(0,0,0){
        }
        void print(){
            printf("(%lf,%lf,%lf,%lf,%lf,%lf)\n",P.x,P.y,P.z,G.yaw,G.pitch,G.roll);
        }  //输出为yaw，pitch，roll
    
};
pose rotate(pose position,gesture angle){   //旋转后的坐标计算
    Quarternion::Quarternion angle_Q=convert_to_Quarternion(angle);
    Quarternion::Quarternion temp=angle_Q*Quarternion::Quarternion(0,position.x,position.y,position.z)*angle_Q.reverse(angle_Q);
    pose result(temp.i,temp.j,temp.k);
    return result;
}
gesture rotate_angle(gesture pre,gesture angle){//旋转后的姿态计算
    Quarternion::Quarternion pre_Q=convert_to_Quarternion(pre);
    Quarternion::Quarternion angle_Q=convert_to_Quarternion(angle);
    pre_Q=pre_Q*angle_Q;
    gesture after_G=convert_to_gesture(pre_Q);
    return after_G;
}
loc camera_to_odom (Mat &tvecs , Mat &rvecs , loc rotate_g_c , loc rotate_g_s,loc rotate_o_g,Mat frame) {
        loc camera,gimbal,shooter,odom,armor; //定义全部坐标系
        camera.P.x=tvecs.at<double>(0,0);camera.P.y=tvecs.at<double>(0,1);camera.P.z=tvecs.at<double>(0,2);
        camera.G.roll=rvecs.at<double>(0,0);camera.G.pitch=rvecs.at<double>(0,1);camera.G.yaw=rvecs.at<double>(0,2);
        camera.Q=convert_to_Quarternion(camera.G);
        //camera系得到所有参数
        //转化至gimbal系
        gimbal.P=rotate(camera.P,rotate_g_c.G);gimbal.P=gimbal.P+rotate_g_c.P; //先旋转，再平移
        gimbal.G=rotate_angle(camera.G,rotate_g_c.G);  gimbal.Q=convert_to_Quarternion(gimbal.G); //gimbal系计算完毕
        //转化至shooter系（无旋转）
        shooter.P=gimbal.P-rotate_g_s.P;shooter.G=gimbal.G;shooter.Q=gimbal.Q;
        //gimbal转化至odom系(无位移)
        odom.P=rotate(gimbal.P,rotate_o_g.G); odom.G=rotate_angle(gimbal.G,rotate_o_g.G); odom.Q=convert_to_Quarternion(odom.G);
        return odom;
        // std::cout<<"odom:";odom.print();
        // circle(frame,Point2f(odom.P.x,odom.P.y),10,Scalar(0,0,255),2);
        // imshow("camera",frame);
        
        
}
void getstatic(loc &rotate_g_c,loc &rotate_g_s,loc &rotate_o_g){
    // cout<<"请输入gimbal到camera的位姿变化"<<endl;
    // scanf("%lf %lf %lf %lf %lf %lf",&rotate_g_c.P.x,&rotate_g_c.P.y,&rotate_g_c.P.z,&rotate_g_c.G.yaw,&rotate_g_c.G.pitch,&rotate_g_c.G.roll);
    // cout<<"请输入gimbal到shooter的位姿变化"<<endl;
    // scanf("%lf %lf %lf %lf %lf %lf",&rotate_g_s.P.x,&rotate_g_s.P.y,&rotate_g_s.P.z,&rotate_g_s.G.yaw,&rotate_g_s.G.pitch,&rotate_g_s.G.roll);
    // cout<<"请输入odom到gimbal的位姿变化"<<endl;
    // scanf("%lf %lf %lf %lf %lf %lf",&rotate_o_g.P.x,&rotate_o_g.P.y,&rotate_o_g.P.z,&rotate_o_g.G.yaw,&rotate_o_g.G.pitch,&rotate_o_g.G.roll);
    rotate_g_c.P.x=0.2;rotate_g_c.P.y=0;rotate_g_c.P.z=0;rotate_g_c.G.roll=1.57;rotate_g_c.G.pitch=0;rotate_g_c.G.yaw=1.57;
    rotate_g_s.P.x=0;rotate_g_s.P.y=0;rotate_g_s.P.z=0;rotate_g_s.G.roll=0;rotate_g_s.G.pitch=0;rotate_g_s.G.yaw=0;
    rotate_o_g.P.x=0;rotate_o_g.P.y=0;rotate_o_g.P.z=0;rotate_o_g.G.roll=0;rotate_o_g.G.pitch=0;rotate_o_g.G.yaw=0;
}

loc armor2odom(double r,Mat tvecs,Mat rvecs,loc rotate_g_c,loc rotate_o_g){
    loc armor;loc rotate_a_c;loc camera;loc odom;loc gimbal;
    armor.P.z=r;
    rotate_a_c.P.x=-tvecs.at<double>(0,0);rotate_a_c.P.y=-tvecs.at<double>(0,1);rotate_a_c.P.z=-tvecs.at<double>(0,2);
    rotate_a_c.G.roll = -rvecs.at<double>(0,0);rotate_a_c.G.pitch = -rvecs.at<double>(0,1);rotate_a_c.G.yaw = -rvecs.at<double>(0,2);
    rotate_a_c.Q=convert_to_Quarternion(rotate_a_c.G);
    camera.P=rotate(armor.P,rotate_a_c.G);camera.G=rotate_angle(armor.G,rotate_a_c.G);camera.Q=convert_to_Quarternion(camera.G);
    gimbal.P=rotate(camera.P,rotate_g_c.G);gimbal.P=gimbal.P+rotate_g_c.P; //先旋转，再平移
    gimbal.G=rotate_angle(camera.G,rotate_g_c.G); gimbal.Q=convert_to_Quarternion(gimbal.G);
    odom.P=rotate(gimbal.P,rotate_o_g.G); odom.G=rotate_angle(gimbal.G,rotate_o_g.G); odom.Q=convert_to_Quarternion(odom.G);
    return odom;
}   

cv::Mat rotationMatrixToEulerAnglesMat(const cv::Mat &R) {
    float sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    
    bool singular = sy < 1e-6; // 如果 sy 接近于零，则矩阵接近奇异
    
    float x, y, z;
    if (!singular) {
        x = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }

    cv::Mat eulerAngles = (cv::Mat_<double>(3, 1) << x, y, z);
    return eulerAngles;
}
std::string convertcenterToString(const loc& odom) {
    std::stringstream ss;
    ss << "center:(" << odom.P.x << "," << odom.P.y << "," << odom.P.z << ","
       << odom.G.roll << "," << odom.G.pitch << "," << odom.G.yaw << ")";
    return ss.str();
}
std::string predict_msg(int x,int y,int z){
    std::stringstream ss;
    ss << "predict center:(" << x << "," << y << "," << z <<  ")";
    return ss.str();
}
