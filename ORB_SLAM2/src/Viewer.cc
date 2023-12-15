#include "Viewer.h"
#include <pangolin/pangolin.h>
#include "unistd.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <mutex>

//ロボットアームの座標を入力するテキストファイルのパス
std::string armPositionTextPath = "../pos_txt/pos.txt";
//ロボットアームの座標と特徴点を紐づけて保持するためのテキストファイルのパス
std::string armPositionDatabaseTextPath = "../pos_txt/pos_tracking_data.txt";

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
    
}

void Viewer::Run()
{
    //ロボットアームを移動するための位置座標を書き込むためのテキストファイル
    std::fstream armPositionTxt(armPositionTextPath);
    std::ofstream armPositionDatabaseTxt(armPositionDatabaseTextPath,std::ios::trunc);
    float posX, posY, posZ;
    float radius = 0.3;
    if (armPositionTxt.is_open() && armPositionDatabaseTxt.is_open()) {
        // ファイルが正常に開かれた場合に読み込みを行う
        std::string line;
        while (std::getline(armPositionTxt, line)) {
            std::istringstream iss(line);
            if (iss >> posX >> posY >> posZ) {
                ;
            } else {
                // 読み込みが失敗した場合
                cout<< "値の読み込みに失敗しました。" << endl;
            }
        }
        armPositionTxt.close();
        armPositionDatabaseTxt.close();
    } else {
        // ファイルが開けなかった場合
        cout<< "ファイルを開くことができませんでした。" <<endl;
    }

    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1160,840);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(200));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowCurrentPoints("menu.Show Current Points", true, true);
    pangolin::Var<bool> menuShowRangeCircle("menu.Show Range", false, true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",false,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<std::string> inputX("menu.X coordinate", std::to_string(posX));
    pangolin::Var<std::string> inputY("menu.Y coordinate", std::to_string(posY));
    pangolin::Var<std::string> inputZ("menu.Z coordinate", std::to_string(posZ));
    pangolin::Var<std::string> inputRadius("menu.Radius", std::to_string(radius));
    pangolin::Var<int> labelSumOfPoint("menu.Feature Points", 0);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1160,840,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");
    cv::moveWindow("ORB-SLAM2: Current Frame",1160,840);

    bool bFollow = true;
    bool bLocalizationMode = false;

    //初期値
    float userInputToWriteX = std::stof(inputX.Get());
    float userInputToWriteY = std::stof(inputY.Get());
    float userInputToWriteZ = std::stof(inputZ.Get());
    float userInputToWriteRadius = std::stof(inputRadius.Get());

    //テキストで保持する最大値の初期値
    int maxOfNearPoints = -1;

    bool isValidUserInput = false;
    float userInputX,userInputY,userInputZ;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph){
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        }

        //viewerから読み込む値と正しく読み込めているかを保持する変数
        try {
        //viewerの入力欄から取得した座標
            userInputX = std::stof(inputX.Get());
            userInputY = std::stof(inputY.Get());
            userInputZ = std::stof(inputZ.Get());
            userInputToWriteRadius = std::stof(inputRadius.Get());
            isValidUserInput = true;
        } catch (const std::invalid_argument& e) {
        // 例外が発生した場合
            cout << "Invalid Input: " << e.what() << std::endl;
            isValidUserInput = false;
        } catch (const std::out_of_range& e) {
            // 例外が発生した場合
            cout << "Out of range: " << e.what() << std::endl;
            isValidUserInput = false;
        }
        //有効にxyzが入力されている場合実行する
        if(isValidUserInput){
            //入力による値の変更を検知
            if(userInputToWriteX!=userInputX || userInputToWriteY!=userInputY || userInputToWriteZ!=userInputZ){
                //現在のロボットアームの座標と特徴点の数をテキストに保持
                std::fstream tmpArmPositionDatabaseTxt(armPositionDatabaseTextPath,std::ios::app);
                tmpArmPositionDatabaseTxt<<fixed<<setprecision(2)<<userInputToWriteX<<" "<<userInputToWriteY<<" "<<userInputToWriteZ<<" "<<maxOfNearPoints<<endl;
                tmpArmPositionDatabaseTxt.close();

                userInputToWriteX = userInputX;
                userInputToWriteY = userInputY;
                userInputToWriteZ = userInputZ;
                std::fstream tmpArmPositionTxt(armPositionTextPath);
                tmpArmPositionTxt<<fixed<<setprecision(2)<<userInputToWriteX<<" "<<userInputToWriteY<<" "<<userInputToWriteZ;
                //最大値を初期化と1秒間停止（アームの移動の前に特徴点が取得されることを防ぐため）
                maxOfNearPoints = -1;
                labelSumOfPoint = 0;
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }else{
                int sumOfNearPoints = mpMapDrawer->CountNearMapPoints(userInputToWriteRadius);
                if(sumOfNearPoints>maxOfNearPoints){
                    maxOfNearPoints = sumOfNearPoints;
                    labelSumOfPoint=maxOfNearPoints;
                }
            }
            
            // //有効となる特徴点の範囲を描画
            if(menuShowRangeCircle){
                mpMapDrawer->DrawRangeCircle(userInputToWriteRadius,30);
            }
            
        }
        if(menuShowPoints){
            mpMapDrawer->DrawMapPoints(menuShowCurrentPoints);
        }
    
        pangolin::FinishFrame();

        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow("ORB-SLAM2: Current Frame",im);
        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }
        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
