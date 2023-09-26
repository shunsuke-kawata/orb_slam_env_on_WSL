echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2
rm -rf build
rm -rf build Mono MonoAR RGBD Stereo
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j4
