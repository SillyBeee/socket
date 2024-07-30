#include <iostream>
#include <cmath>
#include <iomanip>
namespace Quarternion{
class Quarternion{
    
    public:
        double w;
        double i;
        double j;
        double k;
        Quarternion(double a, double b, double c,double d):
        w(a),i(b),j(c),k(d){  //构造函数
        }
        void print() const {  
        std::cout << "(" << w << ", " << i << "i, " << j << "j, " << k << "k)" << std::endl;  
    }  
        Quarternion operator+(const Quarternion& other) const {  //加法
        double w_sum = w + other.w;
        double i_sum = i + other.i;
        double j_sum = j + other.j;
        double k_sum = k + other.k;
        return Quarternion(w_sum, i_sum, j_sum, k_sum);}
        Quarternion operator-(const Quarternion& other) const {  //加法
        double w_sum = w - other.w;
        double i_sum = i - other.i;
        double j_sum = j - other.j;
        double k_sum = k - other.k;
        return Quarternion(w_sum, i_sum, j_sum, k_sum);}
        Quarternion operator*(const Quarternion& other) const { //乘法
        double w_sum = w*other.w-i*other.i-j*other.j-k*other.k;
        double i_sum = w*other.i+i*other.w+j*other.k-k*other.j;
        double j_sum = w*other.j-i*other.k+j*other.w+k*other.i;
        double k_sum = w*other.k+i*other.j-j*other.i+k*other.w;
        return Quarternion(w_sum, i_sum, j_sum, k_sum);
        }
        Quarternion& operator=(const Quarternion& other) {  
        // 自赋值检查  
        if (this != &other) {  
            w = other.w;  
            i = other.i;  
            j = other.j;  
            k = other.k;  
        }  
        // 返回当前对象的引用  
        return *this;  
    }  
        double modd()const{  //模
           return sqrt(w*w+i*i+j*j+k*k);
        }
        Quarternion gonge(){  //共轭
            Quarternion b(0,0,0,0);
            b.w=w;
            b.i=-i;
            b.j=-j;
            b.k=-k;
            return b;
        }
        Quarternion reverse(Quarternion a){ //逆定义
            Quarternion b(0,0,0,0);
            Quarternion c(0,0,0,0);
            b=a.gonge();
            double d;
            d=a.modd();
            c.w=b.w/(a.w*a.w+a.i*a.i+a.j*a.j+a.k*a.k);
            c.i=b.i/(a.w*a.w+a.i*a.i+a.j*a.j+a.k*a.k);
            c.j=b.j/(a.w*a.w+a.i*a.i+a.j*a.j+a.k*a.k);
            c.k=b.k/(a.w*a.w+a.i*a.i+a.j*a.j+a.k*a.k);
            return c;
        }
        Quarternion revolve_yaw(double dy){  //旋转
            Quarternion q(0,0,0,0);
            q.w=cos(dy/2);
            q.k=sin(dy/2);
            return q;
        }
        Quarternion revolve_roll(double dr){  //旋转
            Quarternion q(0,0,0,0);
            q.w=cos(dr/2);
            q.i=sin(dr/2);
            return q;
        }
        Quarternion revolve_pitch(double dp){  //旋转
            Quarternion q(0,0,0,0);
            q.w=cos(dp/2);
            q.j=sin(dp/2);
            return q;
        }
        Quarternion getq(double dy,double dr,double dp){
            Quarternion q(0,0,0,0);
            q=revolve_yaw(dy)*revolve_pitch(dp)*revolve_roll(dr);
            return q;
        }


        
};
}