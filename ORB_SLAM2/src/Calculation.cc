#include<math.h>
#include<string>
#include <sstream>
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

//カメラとカメラから最も近い点を結んだベクトルと任意の点との距離を求める
float CalcPerpendicular(cv::Mat targetPoint,cv::Mat startPoint, cv::Mat endPoint){
    cv::Mat directionVector = endPoint-startPoint;
    cv::Mat targetVector = targetPoint-startPoint;

    float innerProduct = targetVector.dot(directionVector);
    float perpendicularDistance = cv::norm(targetVector) * sqrt(1 - pow(innerProduct / (cv::norm(targetVector) * cv::norm(directionVector)), 2));
    return perpendicularDistance;

}

//カメラとカメラから最も近い点を結んだベクトルと垂直なベクトルを求める
cv::Mat CalcVarticalVector(cv::Mat directionVector,float radius){
    float z = (directionVector.at<float>(0)-directionVector.at<float>(1))/directionVector.at<float>(2);
    cv::Mat varticalVector =(cv::Mat_<float>(3, 1) << directionVector.at<float>(0), directionVector.at<float>(1), z)*radius/cv::norm(directionVector);
    return varticalVector;
}

//カメラとカメラから最も近い点を結んだベクトル方向にある点を回転させた点を求める
cv::Mat CalcRotatedPoint(cv::Mat directionVector,cv::Mat varticalVector,cv::Mat endPoint,int angle){
    float sin_angle = sin(angle);
    float cos_angle = cos(angle);

    //ロドリゲスの回転公式
    cv::Mat rodriguesRotationMatrix(3, 3, CV_32F);
    rodriguesRotationMatrix.at<float>(0, 0) = directionVector.at<float>(0) * directionVector.at<float>(0) * (1 - cos_angle) + cos_angle;
    rodriguesRotationMatrix.at<float>(0, 1) = directionVector.at<float>(0) * directionVector.at<float>(1) * (1 - cos_angle) - directionVector.at<float>(2) * sin_angle;
    rodriguesRotationMatrix.at<float>(0, 2) = directionVector.at<float>(2) * directionVector.at<float>(0) * (1 - cos_angle) + directionVector.at<float>(1) * sin_angle;

    rodriguesRotationMatrix.at<float>(1, 0) = directionVector.at<float>(0) * directionVector.at<float>(1) * (1 - cos_angle) + directionVector.at<float>(2) * sin_angle;
    rodriguesRotationMatrix.at<float>(1, 1) = directionVector.at<float>(1) * directionVector.at<float>(1) * (1 - cos_angle) + cos_angle;
    rodriguesRotationMatrix.at<float>(1, 2) = directionVector.at<float>(1) * directionVector.at<float>(2) * (1 - cos_angle) - directionVector.at<float>(0) * sin_angle;

    rodriguesRotationMatrix.at<float>(2, 0) = directionVector.at<float>(0) * directionVector.at<float>(2) * (1 - cos_angle) - directionVector.at<float>(1) * sin_angle;
    rodriguesRotationMatrix.at<float>(2, 1) = directionVector.at<float>(1) * directionVector.at<float>(2) * (1 - cos_angle) + directionVector.at<float>(0) * sin_angle;
    rodriguesRotationMatrix.at<float>(2, 2) = directionVector.at<float>(2) * directionVector.at<float>(2) * (1 - cos_angle) + cos_angle;

    cv::Mat rotatedVector = rodriguesRotationMatrix*varticalVector;

    return endPoint+rotatedVector;
}
//決められた範囲内に点があるかを判定する
bool IsInCircle(cv::Mat center,cv::Mat point, float radius){
    float distance = CalcDistance2Dim(center,point);
    return distance<=radius; 
}

//半径と距離を比較して円の中に点があるかどうかを比較する
bool IsInCircleRange(float distance,float radius){
    return distance<=radius;
}

//float2stringを行った際の0埋めを削除する
string removeTrailingZeros(const std::string& str) {
    size_t dotPos = str.find('.');
    if (dotPos != std::string::npos) {
        size_t lastNonZero = str.find_last_not_of('0');
        if (lastNonZero == dotPos) {
            return str.substr(0, dotPos);
        } else {
            return str.substr(0, lastNonZero + 1);
        }
    }
    return str;
}


