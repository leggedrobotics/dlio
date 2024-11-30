# A local fork of DLIO for GrandTour Project

## Sensor Configurations

All sensor combinations (IMUs + Hesai/Livox) are supported. Currently, a seperate launch file exist for STIM + HesaiXT-32 combo but rest will be added.

## How to build?

**Note:** make sure to configure your catkin workspace after `catkin init` with `catkin config -DCMAKE_BUILD_TYPE=Release` such that the code is optimized. This version of DLIO explicitly expect PCL 1.10 which comes default with Ubuntu 20.04.

`catkin build direct_lidar_inertial_odometry` and source with `source devel/setup.bash`.

## How to Run?

If it's your first time here, get the [[ Example Data ](https://drive.google.com/drive/folders/1--H2bow_XGQW5u3hcs3EFAwqDOzuidR0?usp=sharing)]. This bag contains `/tf_static` for automatic calibration reading, STIM320 IMU data and HesaiXT32 motion-distorted pointclouds. You will need to provide the path of the bag to the replay launch file or, you will need to play it yourself fir live operation.

# Real-time
`roslaunch direct_lidar_inertial_odometry dlio.launch` -> This launch file is dedicated for live operation. 

The launched nodes expects the topics specified in the launch file (current defult`/gt_box/stim320/imu` and `/gt_box/hesai/points`) So play your favorite bag with `rosbag play --clock --pause -s 0 -r 1.0 your.bag`. The node will identify the IMU biases for some seconds and start publishing.

# Replay

**Note:** Replay mode expects you to provide an input bag path in the launch file `dlio_replay.launch`. The bag will be read automatically and played automatically. The node will terminate automatically once the bag ends, the results of the framework will be saved automatically by default to `$(find direct_lidar_inertial_odometry)/data"` which can be changed in `dlio_replay.launch`.

`roslaunch direct_lidar_inertial_odometry dlio_replay.launch` after setting the right topic names and the bag file path, you can run the replay with this line.

# Replay Viz

`roslaunch direct_lidar_inertial_odometry post_play.launch` you can use this launch file to automatically visualize the outputs of the framework. By default it loads the last replayed bag from it's default path. It plays the replayed results in 5x speed.

# Important Outputs

`/dlio/deskewed_point_cloud` this is the deskewed point cloud. Same timestamp as the original cloud, and frame. The difference is that the motion compansation has been performed and it can be used for other applications.

`/dlio/lidar_map_path` the path that the LiDAR took in the `dlio_map` frame.

`/dlio/lidar_odometry` the LiDAR pose in `dlio_odom` as nav_msgs::Odometry message. Reads as pose of LiDAR frame in Odometry frame.

`/dlio/lidar_odometry_as_posestamped` the LiDAR pose in `dlio_odom` as geometry_msgs::PoseStamped message. Reads as pose of LiDAR frame in Odometry frame.

`/dlio/lidar_map_odometry` the LiDAR pose in `dlio_map` as nav_msgs::Odometry message. Reads as pose of LiDAR frame in Map frame.

`/tf` , the odometry node add the odometry frame as a child of LiDAR to respect the tree structure. You can accumulate the clouds using this frame.
Odometry frame is called `dlio_odom` and publishes poses at IMU rate. Map frame is called `dlio_map` and publishes at the LiDAR rate.

**Note:** if you want to change to a different sensor you need too adapt the extrinsic available in `dlio.yaml` and change the topic names / read parameter files accordingly.

**Note:** After the destruction of the node, the mapper will automatically save a voxel map to the package directory.

Below you can find the original readme of the authors of this work.


# Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction

#### [[ IEEE ICRA ](https://ieeexplore.ieee.org/document/10160508)] [[ arXiv ](https://arxiv.org/abs/2203.03749)] [[ Video ](https://www.youtube.com/watch?v=4-oXjG8ow10)] [[ Presentation ](https://www.youtube.com/watch?v=Hmiw66KZ1tU)]

DLIO is a new lightweight LiDAR-inertial odometry algorithm with a novel coarse-to-fine approach in constructing continuous-time trajectories for precise motion correction. It features several algorithmic improvements over its predecessor, [DLO](https://github.com/vectr-ucla/direct_lidar_odometry), and was presented at the IEEE International Conference on Robotics and Automation (ICRA) in London, UK in 2023.

<br>
<p align='center'>
    <img src="./doc/img/dlio.png" alt="drawing" width="720"/>
</p>

## Instructions

### Sensor Setup & Compatibility
DLIO has been extensively tested using a variety of sensor configurations and currently supports Ouster, Velodyne, Hesai, and Livox LiDARs. The point cloud should be of input type `sensor_msgs::PointCloud2` and the 6-axis IMU input type of `sensor_msgs::Imu`. 

For Livox sensors specifically, you can use the `master` branch directly if it is of type `sensor_msgs::PointCloud2` (`xfer_format: 0`), or the `feature/livox-support` branch and the latest [`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2) package if it is of type `livox_ros_driver2::CustomMsg` (`xfer_format: 1`) (see [here](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/issues/5) for more information).

For best performance, extrinsic calibration between the LiDAR/IMU sensors and the robot's center-of-gravity should be inputted into `cfg/dlio.yaml`. If the exact values of these are unavailable, a rough LiDAR-to-IMU extrinsics can also be used (note however that performance will be degraded).

IMU intrinsics are also necessary for best performance, and there are several open-source calibration tools to get these values. These values should also go into `cfg/dlio.yaml`. In practice however, if you are just testing this work, using the default ideal values and performing the initial calibration procedure should be fine.

Also note that the LiDAR and IMU sensors _need_ to be properly time-synchronized, otherwise DLIO will not work. We recommend using a LiDAR with an integrated IMU (such as an Ouster) for simplicity of extrinsics and synchronization.

### Dependencies
The following has been verified to be compatible, although other configurations may work too:

- Ubuntu 20.04
- ROS Noetic (`roscpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `pcl_ros`)
- C++ 14
- CMake >= `3.12.4`
- OpenMP >= `4.5`
- Point Cloud Library >= `1.10.0`
- Eigen >= `3.3.7`

```sh
sudo apt install libomp-dev libpcl-dev libeigen3-dev
```

DLIO supports ROS1 by default, and ROS2 using the `feature/ros2` branch.

### Compiling
Compile using the [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/) package via:

```sh
mkdir ws && cd ws && mkdir src && catkin init && cd src
git clone https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git
catkin build
```

### Execution
After compiling, source the workspace and execute via:

```sh
roslaunch direct_lidar_inertial_odometry dlio.launch \
  rviz:={true, false} \
  pointcloud_topic:=/robot/lidar \
  imu_topic:=/robot/imu
```

for Ouster, Velodyne, Hesai, or Livox (`xfer_format: 0`) sensors, or 

```sh
roslaunch direct_lidar_inertial_odometry dlio.launch \
  rviz:={true, false} \
  livox_topic:=/livox/lidar \
  imu_topic:=/robot/imu
```

for Livox sensors (`xfer_format: 1`).

Be sure to change the topic names to your corresponding topics. Alternatively, edit the launch file directly if desired. If successful, you should see the following output in your terminal:
<br>
<p align='center'>
    <img src="./doc/img/terminal.png" alt="drawing" width="480"/>
</p>

### Services
To save DLIO's generated map into `.pcd` format, call the following service:

This feature is currently disabled due to a build error. [TT]


```sh
rosservice call /robot/dlio_map/save_pcd LEAF_SIZE SAVE_PATH
```

### Test Data
For your convenience, we provide test data [here](https://drive.proton.me/urls/Z83QCWKZWW#bMIqDh02AJZZ) (1.2GB, 1m 13s, Ouster OS1-32) of an aggressive motion to test our motion correction scheme, and [here](https://drive.proton.me/urls/7NQSK9DXJ0#gZ9yjGNrDBgG) (16.5GB, 4m 21s, Ouster OSDome) of a longer trajectory outside with lots of trees. Try these two datasets with both deskewing on and off!

<br>
<p align='center'>
    <img src="./doc/gif/aggressive.gif" alt="drawing" width="720"/>
</p>

## Citation
If you found this work useful, please cite our manuscript:

```bibtex
@article{chen2022dlio,
  title={Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction},
  author={Chen, Kenny and Nemiroff, Ryan and Lopez, Brett T},
  journal={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2023},
  pages={3983-3989},
  doi={10.1109/ICRA48891.2023.10160508}
}
```

## Acknowledgements

We thank the authors of the [FastGICP](https://github.com/SMRT-AIST/fast_gicp) and [NanoFLANN](https://github.com/jlblancoc/nanoflann) open-source packages:

- Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno, “Voxelized GICP for Fast and Accurate 3D Point Cloud Registration,” in _IEEE International Conference on Robotics and Automation (ICRA)_, IEEE, 2021, pp. 11 054–11 059.
- Jose Luis Blanco and Pranjal Kumar Rai, “NanoFLANN: a C++ Header-Only Fork of FLANN, A Library for Nearest Neighbor (NN) with KD-Trees,” https://github.com/jlblancoc/nanoflann, 2014.

We would also like to thank Helene Levy and David Thorne for their help with data collection.

Many thanks to [@shrijitsingh99](https://github.com/shrijitsingh99) for [porting DLIO to ROS2](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/pull/16)!

## License
This work is licensed under the terms of the MIT license.

<br>
<p align='center'>
    <img src="./doc/img/ucla.png" alt="drawing" width="720"/>
</p>
<p align='center'>
    <img src="./doc/img/trees.png" alt="drawing" width="720"/>
</p>
