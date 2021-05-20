#conda init bash
#conda activate TDNet
source ./third_party/AirSim/ros/devel/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/root/anaconda3/envs/TDNet/bin/python3.7 -DPYTHON_INCLUDE_DIR=/root/anaconda3/envs/TDNet/include/python3.7m/ -DPYTHON_LIBRARY=/root/anaconda3/envs/TDNet/lib/libpython3.7m.so
source ./devel/setup.bash
