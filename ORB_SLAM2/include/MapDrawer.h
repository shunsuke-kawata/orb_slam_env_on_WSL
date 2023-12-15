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
bool IsInCircle(cv::Mat center,cv::Mat point, float radius);
bool IsInCircleRange(float distance,float radius);

namespace ORB_SLAM2
{

class MapDrawer
{
public:
    MapDrawer(Map* pMap, const string &strSettingPath);

    Map* mpMap;
    int  CountNearMapPoints(const bool bDrawCurrentPoints);
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
