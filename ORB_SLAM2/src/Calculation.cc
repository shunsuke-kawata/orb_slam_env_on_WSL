#include<math.h>
#include<opencv2/core/core.hpp>

using namespace std;

//xy平面におけるユークリッド距離を計算する
float CalcDistance2Dim(cv::Mat point1, cv::Mat point2){
    float distance = sqrt(pow(point2.at<float>(0)-point1.at<float>(0),2)+pow(point2.at<float>(1)-point1.at<float>(1),2));
    return distance;
}
//xyz空間におけるユークリッド距離を計算する
float CalcDistance3Dim(cv::Mat point1, cv::Mat point2){
    float distance = sqrt(pow(point2.at<float>(0)-point1.at<float>(0),2)+pow(point2.at<float>(1)-point1.at<float>(1),2)+pow(point2.at<float>(2)-point1.at<float>(2),2));
    return distance;
};

//決められた範囲内に点があるかを判定する
bool IsInCircle(cv::Mat center,cv::Mat point, float radius){
    float distance = CalcDistance2Dim(center,point);
    return distance<=radius; 
}
