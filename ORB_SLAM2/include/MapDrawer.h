#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

//使用する関数の宣言
// void writeToTextFile(const std::string& filePath);
float CalcDistance2Dim(cv::Mat point1, cv::Mat point2);
float CalcDistance3Dim(cv::Mat cameraPosition, cv::Mat highestPoint);
float CalcPerpendicular(cv::Mat targetPoint,cv::Mat startPoint, cv::Mat endPoint);
cv::Mat CalcVarticalVector(cv::Mat directionVector,float radius);
cv::Mat CalcRotatedPoint(cv::Mat directionVector,cv::Mat varticalVector,cv::Mat endPoint,int angle);
bool IsInCircle(cv::Mat center,cv::Mat point, float radius);
bool IsInCircleRange(float distance,float radius);

struct PointInfo {
    int sumOfNearPoints;
    float distance;
};


namespace ORB_SLAM2
{

class MapDrawer
{
public:
    MapDrawer(Map* pMap, const string &strSettingPath);

    Map* mpMap;
    PointInfo CountNearMapPoints(const float radius);
    void DrawRangeCircle(const float radius,const int angle);
    void DrawMapPoints(const bool bDrawCurrentPoints);
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
