# LiDAR_datastream_transceiver


This is part of the **LiDAR Data Streaming** Project developed at TiHAN, IIT-H

## Environment Setup
[ROS](https://www.ros.org/) (preferably [melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) version for Ubuntu 18.04) is required to be setup on both the sender (encoder) and the receiver (decoder). 

### (Optional) ROS-Bridge Server Setup
ROS-Bridge server needs to be installed for [roslibjs](http://wiki.ros.org/roslibjs) which was part of the project that this repository was intended for. 

Installation: (Change `melodic` to the required version)
```bash
sudo apt-get install ros-melodic-rosbridge-server
```

Starting Server:
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

## Obtaining Source Code and Building
The source code is available [here](https://github.com/bhaskar-anand-iith/LiDAR_datastream_transceiver/). 

Run the following commands:
```bash
cd ~ #directory where the workspace is to be located
git clone https://github.com/bhaskar-anand-iith/LiDAR_datastream_transceiver/
cd LiDAR_datastream_transceiver/ #root of workspace
catkin build #building
```

### Troubleshooting Build Errors
* If on Ubuntu 20.04+ / ROS Neotic (or later): in `CMakeLists.txt` add the lines
```makefile
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
```
* `octomap_ros` not found: in `CMakeLists.txt` remove the dependency:
```makefile
  octomap_ros #(line 13)
```

## Execution
From workspace root folder:
```bash
source devel/setup.bash
rosrun compress_encoder encoder #For Encoder
rosrun compress_decoder decoder #For Decoder
```

### ROSTopic Specifications
#### Encoder
Subscribes to `/velodyne_points` topic (from velodyne sensor)

Publishes to `/n2b_data` topic (to browser)

#### Decoder
Subscribes to `/b2n_data` topic (from browser)

Publishes to `/n2v_data` topic (to visualizer)
