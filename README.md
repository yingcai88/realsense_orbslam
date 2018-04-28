# rs_orb_slam for using ORB_SLAM2 on realsense camera 

## 0. Install realsense driver package
* [librealsense] http://wiki.ros.org/librealsense - The realsense driver library
* [realsense_camera] http://wiki.ros.org/realsense_camera - The realsense ROS package

```
wget -O enable_kernel_sources.sh http://bit.ly/en_krnl_src
bash ./enable_kernel_sources.sh
sudo apt-get update
```
```
sudo apt-get --reinstall install 'ros-*-librealsense'
sudo apt-get install 'ros-*-realsense-camera'
roslaunch realsense_camera [cam_type]_nodelet_[mode].launch
```
## 1. Install pangolin

```
sudo apt-get install libglew-dev
sudo apt-get install cmake
sudo apt-get install libpython2.7-dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
```

## 2. Install orbslam

```
git clone https://github.com/raulmur/ORB_SLAM2.git
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

## 3. Build rs_orbslam package

* clone and make this package in a ROS workspace
* modify ORBSLAM_DIR in CMakeLists.txt
* modify rs_orbslam.yaml
* modify rs_orbslam.launch
* run this package using roslaunch
```
roslaunch rs_orbslam rs_orbslam.launch 
```
