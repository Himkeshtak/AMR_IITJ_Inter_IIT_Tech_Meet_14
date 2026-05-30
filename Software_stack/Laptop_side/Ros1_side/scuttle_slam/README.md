# scuttle_slam

The `scuttle_slam` [ROS](https://www.ros.org/) package provides configurations for different
[SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) algorithms.


## Dependencies

The configurations in this repository assume you have the following prerequisites installed on the
device on which you want to run this code. That device might be an Ubuntu machine or a physical
SCUTTLE using Raspberry Pi OS.

1. [ROS Noetic](http://wiki.ros.org/noetic) with the `ros-noetic-robot` and
   `ros-noetic-tf2` packages.
1. A working [ROS workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
1. The [scuttle_description](https://github.com/scuttlerobot/scuttle_description) package installed
   in your workspace.
1. The [scuttle_bringup](https://github.com/scuttlerobot/scuttle_bringup) package installed in your
   workspace.

The ROS SLAM algorithms assume that at least one sensor is present that is able to either create a
[LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) or a
[PointCloud](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html). Additionally some
SLAM algorithms require that your robot publishes odometry data.

## Implemented algorithms

The implemented algorithms are:

1. [Gmapping](http://wiki.ros.org/gmapping) - [OpenSLAM's gmapping](<https://openslam-org.github.io/gmapping.html>)
   algorithm

## Usage

After installing the prerequisites you can pull this repository to the device on which you want to
run the code.

    cd <YOUR_ROS_WORKSPACE>/src
    git clone -b noetic https://github.com/scuttlerobot/scuttle_slam.git

Once the code is on the device you can 'build' the workspace

    cd <YOUR_ROS_WORKSPACE>
    catkin_make

Finally you can run the navigation stack for SCUTTLE

    roslaunch scuttle_slam scuttle_slam.launch
