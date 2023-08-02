# orb_slam2_env
卒業研究用のorb_slam2の実行環境の構築や改良の保存をwindows11のwsl上で行ったもの

## 実行環境

| | |
|----|----|
| OS | ubuntu 20.04.6 (wsl) on Windows11 |
| ROS | noetic |

## インストールが必要なライブラリなど
- Opencv
- Pangolin
- Eigen
- ros
- GUIアプリ(wslでlinuxGUIを使用するためのアプリケーション)
- usb Webカメラ認識ツール

## 追加ライブラリの インストール先
- /usr/local/include/

## 

## wslにusb Webカメラを認識する
```
#numの部分にデバイスのナンバーを入力
#PowerShellで実行
usbipd wsl attach --busid (num)-(num)

#wslで接続されているデバイスを確認
#Ubuntuで実行
lsusb
```

## 実行方法

- 以下のコマンドをそれぞれ別の端末で実行する

```
roscore

rosrun usb_cam usb_cam_node /usb_cam/image_raw:=/camera/image_raw

#./ORB_SLAM2/ 内にいることを確認して実行する
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml 
```

