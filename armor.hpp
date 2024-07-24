#include <string>
#include <vector>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <cmath>
class light {
    public:
    cv::Point2f top;
    cv::Point2f bottom;
};
enum class ArmorType{
    SMALL,
    LARGE
};
class Armor{
    public:
    light left_light;
    light right_light;
    ArmorType type;
    cv::Mat number_image;
    double classification_confidence;
    std::string number;
    std::string classification_result;
    

};