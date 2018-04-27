################### rs_orb_slam #########################

# 0. install realsense driver package
# http://wiki.ros.org/librealsense
# http://wiki.ros.org/realsense_camera

wget -O enable_kernel_sources.sh http://bit.ly/en_krnl_src
bash ./enable_kernel_sources.sh
sudo apt-get update

sudo apt-get --reinstall install 'ros-*-librealsense'
sudo apt-get install 'ros-*-realsense-camera'
roslaunch realsense_camera [cam_type]_nodelet_[mode].launch

# 1. install pangolin

git clone https://github.com/stevenlovegrove/Pangolin.git

sudo apt-get install libglew-dev
sudo apt-get install cmake
sudo apt-get install libpython2.7-dev

cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .

# 2. install orbslam

git clone https://github.com/raulmur/ORB_SLAM2.git

cd ORB_SLAM2
chmod +x build.sh
./build.sh

# 3. build rs_orbslam package

# modify ORBSLAM_DIR in CMakeLists.txt
# modify rs_orbslam.yaml
# modify rs_orbslam.launch
roslaunch rs_orbslam rs_orbslam.launch 

