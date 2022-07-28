#Download and Compile

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
