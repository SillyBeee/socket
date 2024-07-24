#include <iostream>
#include <cmath>
#include <iomanip>

class gesture{
    
    public:
        double yaw;   //构造函数
        double pitch;
        double roll;
        gesture(double a, double b, double c){
           yaw = c;
           pitch = b;
           roll = a;
        }
        void print() const {  
        std::cout << "(" << roll << ", " << pitch << ", " << yaw <<")"<<std::endl;  
    }
        
} ;      
class pose{
    public:
        double x;   //构造函数
        double y;
        double z;
        pose(double a, double b, double c){
           x = a;
           y = b;
           z = c;
        }
        pose& operator=(const pose& other) {  
        // 自赋值检查  
        if (this != &other) {  
            x = other.x;  
            y = other.y;  
            z = other.z;    
        }  
        // 返回当前对象的引用  
        return *this;  
    }  
        pose operator+(const pose& other) const {  //加法
        double x_sum = x + other.x;
        double y_sum = y + other.y;
        double z_sum = z + other.z;
        return pose(x_sum, y_sum, z_sum);}
        pose operator-(const pose& other) const {  //减法
        double x_sum = x - other.x;
        double y_sum = y - other.y;
        double z_sum = z - other.z;
        return pose(x_sum, y_sum, z_sum);}
        void print() const {  
        std::cout << "(" << x << ", " << y << ", " << z <<")"<<std::endl;  
    }
};