#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{

MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

//近い点をカウントする関数
PointInfo MapDrawer::CountNearMapPoints(const float radius){
    PointInfo result;
    result.sumOfNearPoints = -1;
    result.distance = -1;
    const vector<MapPoint *> &vpCurrentMPs  = mpMap->GetCurrentMapPoints(); 
    if (vpCurrentMPs.size()>0){
        cv::Mat Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*mCameraPose.rowRange(0,3).col(3);

        MapPoint* nearestP = mpMap->GetNearestMapPoint(vpCurrentMPs,twc);
        cv::Mat nearestPPos = nearestP->GetWorldPos();

        if(nearestP!=nullptr){
            int sumOfNearPoints = 0;
            for (size_t i = 0; i < vpCurrentMPs.size(); i++) {
                float perpendicularDistance = CalcPerpendicular(vpCurrentMPs[i]->GetWorldPos(),twc,nearestPPos);
                if(IsInCircleRange(perpendicularDistance,radius)){
                    sumOfNearPoints+=1;
                }
            }

            float distance = CalcDistance3Dim(twc,nearestPPos);
            cout<<distance<<endl;
            result.sumOfNearPoints = sumOfNearPoints;
            result.distance = distance;
            return result;
        }
    }
    return result;
}

void MapDrawer::DrawRangeCircle(const float radius,const int angle)
{
    const vector<MapPoint *> &vpCurrentMPs  = mpMap->GetCurrentMapPoints();
    if (vpCurrentMPs.size()>0){
        cv::Mat Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        MapPoint* nearestP = mpMap->GetNearestMapPoint(vpCurrentMPs,twc);
        cv::Mat nearestPPos = nearestP->GetWorldPos();

        //視点と終点を結んだベクトル
        cv::Mat directionVector = nearestPPos-twc;
        //360度以上は入ってこないと想定
        cv::Mat varticalVector = CalcVarticalVector(directionVector,radius);
        //角度を360で割った数だけ描画をおこなう
        //あおおの店で描画を行う
        glPointSize(mPointSize*4);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 1.0);
        for (int i=0;i<360/angle;i++){
            int tmpAngle = angle*i;
            cv::Mat rotatedPoint = CalcRotatedPoint(directionVector,varticalVector,nearestPPos,tmpAngle);
            glVertex3f(rotatedPoint.at<float>(0), rotatedPoint.at<float>(1), rotatedPoint.at<float>(2));
        }
        glEnd();
    }
    return;
}

void MapDrawer::DrawMapPoints(const bool bDrawCurrentPoints)
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();
    const vector<MapPoint *> &vpCurrentMPs  = mpMap->GetCurrentMapPoints();   
    
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;
    
    if (bDrawCurrentPoints)
    {
        // Define points
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 0.0);
        
        //currentFrameに写っている点飲み描画する
        for (std::vector<MapPoint *>::const_iterator i = vpCurrentMPs.begin(); i != vpCurrentMPs.end(); i++)
        {
            if ((*i)->isBad())
                continue;
            cv::Mat pos = (*i)->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
        }
        glEnd();
    }

    /*
    もともとあった描画処理
    重くなるのと必要ないので一旦削除
    */
    // 黒の点を描画している
    // glPointSize(mPointSize);
    // glBegin(GL_POINTS);
    // glColor3f(0.0,0.0,0.0);
    // for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    // {
    //     if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
    //         continue;
    //     cv::Mat pos = vpMPs[i]->GetWorldPos();
    //     glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    // }
    // glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    //赤の点を描画している
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }
    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif
    //点の中心の描画
    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
