# ROS packages in Project Venom

## Contents

| package name | purpose |
| --- | --- |
| venom_offb | Nodes that are related to offboard mode control, including navigation decision making. | 
| venom_perception | Sensor nodes that is integrated with ROS. `Class Perceiver ` is designed to wrap different sensors. Note that zed-ros-wrapper should be placed here.|
| venom_zed | Direct use of zed's api |


## Prerequisite

1. [PX4 toolchain](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#convenience-bash-scripts): we will not build mavros from scratch. Choose ubuntu_sim.sh to install.
2. mavros:  `sudo apt-get install ros-kinetic-mavros ros-indigo-kinetic-extras`
3. [CUDA 9.1](https://developer.nvidia.com/cuda-downloads): remember to setup library path.
`echo export PATH=/usr/local/cuda-9.1/bin${PATH}`
`export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/lib64`
4. [ZED SDK 2.3](https://www.stereolabs.com/developers/release/2.3/)
