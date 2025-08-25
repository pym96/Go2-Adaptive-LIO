# 3d Unitree Quadruped robot LIO simulation

<p align="center">
  <img src="https://github.com/user-attachments/assets/c56ca79e-c744-48f7-a4ca-7d35727373a3" alt="Timeline1" />
</p>


## prerequisites
- ros1 noetic or melodic
```
wget http://fishros.com/install -O fishros && . fishros
```

- preconfiguration
```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install libgoogle-glog-dev libeigen3-dev libpcl-dev libyaml-cpp-dev liblcm-dev
```

- onnxruntime configuration

[onnx-release](https://github.com/microsoft/onnxruntime/releases/tag/v1.21.0) or
```
wget https://github.com/microsoft/onnxruntime/releases/download/v1.21.0/onnxruntime-linux-x64-1.21.0.tgz
```
then uncompress it and run command below:
```
sudo cp -r ./onnxruntime-linux-x64-1.21.0 /usr/lib/onnxruntime-linux-x64
```

clone this repository
```
git clone https://github.com/pym96/Go2-Adaptive-LIO.git
```
- compile
```
catkin build
```

- run simulation (source devel/setup.bash) first 
```
roslaunch robot_control RobotBringup.launch 
```

- run keyboard cmd control (w a s d to control velocity, space bar to stop)
```
roslaunch robot_cmd keyboard_teleop.launch 
```

- run LIO system
```
roslaunch faster_lio mapping_velodyne.launch 
```


## Reference link
* **`Onnxruntime`** : https://github.com/microsoft/onnxruntime.git

* **`Faster-lio`** (A light-weight Lidar-inertial odometry for lidar pose tracking and point cloud mapping.):https://github.com/gaoxiang12/faster-lio.git


* **`Unitree-guide`** (The following will quickly introduce the use of unitree_guide in the gazebo simulator. For more usage, please refer to 《四足机器人控制算法--建模、控制与实践》.) :https://github.com/unitreerobotics/unitree_guide.git


## Future work
- Terrian analysis with [elevation_mapping](https://github.com/ANYbotics/elevation_mapping.git)
- Final 3d planner with things above.....
