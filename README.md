# monocular_exploration

For more infromation, see FYP_Final_Report.pdf

# How to build:
The following setup has been successfully tested for these instructions: 
Markup : * Ubuntu 18.04
         * CUDA 10.1
         * ROS Melodic (used full-desktop installation)

## Update and install compilers
```console
sudo apt update
sudo apt upgrade
apt-get install -y gcc-6 g++-6  gcc-7 g++-7 gcc-8 g++-8
```

## gstreamer
```
sudo apt install -y build-essential git
sudo apt install -y \
libssl1.0.0 \
libgstreamer1.0-0 \
gstreamer1.0-tools \
gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly \
gstreamer1.0-libav \
libgstrtspserver-1.0-0 \
libjansson4
```
## opencv
```
sudo apt install -y cmake pkg-config unzip yasm git checkinstall
sudo apt install -y libjpeg-dev libpng-dev libtiff-dev
sudo apt install -y libavcodec-dev libavformat-dev libswscale-dev libavresample-dev
sudo apt install -y libxvidcore-dev x264 libx264-dev libfaac-dev libmp3lame-dev libtheora-dev 
sudo apt install -y libfaac-dev libmp3lame-dev libvorbis-dev
sudo apt install -y libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install -y libdc1394-22 libdc1394-22-dev libxine2-dev libv4l-dev v4l-utils
cd /usr/include/linux
sudo ln -s -f ../libv4l1-videodev.h videodev.h
cd ~
sudo apt-get install -y libgtk-3-dev
sudo apt-get install -y python3-dev python3-pip
sudo -H pip3 install -U pip numpy
sudo apt install python3-testresources
sudo apt-get -y install libtbb-dev
sudo apt-get -y install libatlas-base-dev gfortran
echo "Create a virtual environtment for the python binding module"
sudo pip3 install virtualenv virtualenvwrapper
sudo rm -rf ~/.cache/pip3
echo "Edit ~/.bashrc"
export WORKON_HOME=$HOME/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source /usr/local/bin/virtualenvwrapper.sh
mkvirtualenv cv -p python3
pip3 install numpy
git clone --recurse-submodules https://github.com/danigr99727/monocular_exploration

echo "Procced with the installation"
cd monocular_exploration/third_party/opencv
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_C_COMPILER=/usr/bin/gcc-6 \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D WITH_TBB=ON \
-D WITH_CUDA=ON \
-D BUILD_opencv_cudacodec=OFF \
-D ENABLE_FAST_MATH=1 \
-D CUDA_FAST_MATH=1 \
-D WITH_CUBLAS=1 \
-D WITH_V4L=ON \
-D WITH_QT=OFF \
-D WITH_OPENGL=ON \
-D WITH_GSTREAMER=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_PC_FILE_NAME=opencv.pc \
-D OPENCV_ENABLE_NONFREE=ON \
-D OPENCV_PYTHON3_INSTALL_PATH=~/.virtualenvs/cv/lib/python3.6/site-packages \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
-D PYTHON_EXECUTABLE=~/.virtualenvs/cv/bin/python \
-D WITH_CUDNN=OFF \
-D BUILD_EXAMPLES=ON ..

make -j6
sudo make install
```
## AIRSIM
```
cd ../../AirSim
./setup.sh
./build.sh
```

## Pangolin
```
cd ../Pangolin
sudo apt install -y libgl1-mesa-dev
sudo apt install -y libglew-dev
sudo apt install -y libpython2.7-dev
sudo apt install -y pkg-config
sudo apt install -y libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols
sudo apt install -y libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev
mkdir build
cd build
cmake ..
cmake --build .
sudo cmake --install .
```

## ORB-SLAM2-CUDA
```
cd ../../ORB_SLAM2_CUDA
sudo apt-get install -y libblas-dev
sudo apt-get install -y liblapack-dev
sudo apt-get install -y libopenni2-dev
sudo apt-get install -y python-vtk
./build.sh
```

## FLAME
```
cd ../flame
sudo apt-get install -y libboost-all-dev libpcl-dev python-catkin-tools
# Create a dependencies folder.
mkdir -p dependencies/src

# Checkout Eigen and Sophus into ./dependencies/src and install into ./dependencies.
./scripts/eigen.sh ./dependencies/src ./dependencies
./scripts/sophus.sh ./dependencies/src ./dependencies

# Copy and source environment variable script:
cp ./scripts/env.sh ./dependencies/


source ./dependencies/env.sh
cd flame
mkdir build
mkdir install
cd build
cmake -D CMAKE_INSTALL_PREFIX=../install -D BUILD_TESTS=OFF  ..
make install
```

## TDNET ENV
```
cd ../../../
sudo apt-get install -y python3.7
sudo apt-get install -y python3.7-venv
sudo apt-get install -y python3.7-dev 
python3 -m venv ./TDNetEnv
source ./TDNetEnv/bin/activate
sudo apt-get install -y python3-yaml
sudo apt-get install -y python-catkin-tools python3.7-dev python3-numpy
sudo pip3.7 install -r requirements.txt
```
## AirSim ROS
```
cd third_party/AirSim/ros
sudo apt-get install -y ros-melodic-tf2-sensor-msgs ros-melodic-tf2-geometry-msgs ros-melodic-mavros*
catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
source ./devel/setup.bash
```
## FUEL dependencies
cd ../../../
sudo apt-get install -y libdw-dev
sudo apt-get -y install libarmadillo-dev ros-melodic-nlopt

## Build catkin packages
./catkin_make_python3
