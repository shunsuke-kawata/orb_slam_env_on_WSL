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

float CalcPerpendicular(cv::Mat targetPoint,cv::Mat startPoint, cv::Mat endPoint){
    cv::Mat directionVector = endPoint-startPoint;
    cv::Mat targetVector = targetPoint-startPoint;

    float innerProduct = targetVector.dot(directionVector);
    float perpendicularDistance = cv::norm(targetVector) * sqrt(1 - pow(innerProduct / (cv::norm(targetVector) * cv::norm(directionVector)), 2));
    return perpendicularDistance;

}
//決められた範囲内に点があるかを判定する
bool IsInCircle(cv::Mat center,cv::Mat point, float radius){
    float distance = CalcDistance2Dim(center,point);
    return distance<=radius; 
}

bool IsInCircleRange(float distance,float radius){
    return distance<=radius;
}
