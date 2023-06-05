# orb_slam2_env
卒業研究用のorb_slam2の実行環境の構築や改良の保存

## 実行環境

| | |
|----|----|
| OS | ubuntu 20.04 |
| ROS | noetic |

## 実行方法

- 以下のコマンドを別の端末で実行する

```
roscore

rosrun usb_cam usb_cam_node /usb_cam/image_raw:=/camera/image_raw

#./ORB_SLAM2/ 内にいることを確認して実行する
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml 
```

## 取得したい結果

``` ./ORB_SLAM2/KeyFrameTrajectory.txt```　に保存されていると考えられる

