#conda init bash
#conda activate TDNet

catkin_make -DGSTREAMER_VERSION_1_x=On -DPYTHON_EXECUTABLE=../TDNetEnv/bin/python3.7 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m/ -DPYTHON_LIBRARYi=/usr/lib/x86_64-linux-gnu/libpython3.7m.so

