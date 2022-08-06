This is a ROS package to simulate multi-sensor platforms through Gazebo. Two platforms are included. One platform includes an IMU and a camera, whose model is built in `integrated_platform_imu_1cam.xacro`. The other platform includes an IMU, four cameras and two LiDARs, whose model is described in `integrated_platform_2side.xacro`.

# Download and Usage
Please install ROS and Gazebo before using this package.

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone  https://github.com/STAR-Center/Multi-sensor_Simulation.git 
cd Multi-sensor_Simulation
git submodule add https://bitbucket.org/DataspeedInc/velodyne_simulator.git velodyne_simulator
cd ../..
catkin_make
source devel/setup.sh
cd src/Multi-sensor_Simulation
source sim_move_and_record.sh
```

# Calibration
For multi-sensor calibration, please refer to [Mutical](https://github.com/zhixy/multical), proposed by the paper "Multical: Spatiotemporal Calibration for Multiple IMUs, Cameras and LiDARs".
We also provide [real dataset](https://robotics.shanghaitech.edu.cn/datasets/multical) to test our calibration method.
