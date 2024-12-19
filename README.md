# ROS-Bridge-Noetic
ROS Noetic support packages that can be used with the ROS Bridge.

## Table of Contents
- [Contributors](#contributors)
- [See Also](#see-also)
- [Usage](#usage)
    - [Physical Robot Data Re-Publishers](#physical-robot-data-re-publishers)
        - [Physical Robot Data Re-Publisher - Lidar Data](#physical-robot-data-re-publisher---lidar-data)
        - [Physical Robot Data Re-Publisher - Transforms](#physical-robot-data-re-publisher---transforms)

## Contributors
Created by [Shaun Altmann](https://github.com/ShaunAlt).

## See Also
This package is implemented by the [ROS-Bridge-Docker](https://github.com/ShaunAlt-Unitree-Go1/ROS-Bridge-Docker) repository.

## Usage
If used as a part the [ROS-Bridge-Docker](#see-also) repository, then all packages will have been created within the directory specified in that documentation. Otherwise, you can build these packages using the following commands in a terminal:
``` bash
# create the workspace you wish to build the packages in
$ mkdir -p ~/noetic_ws/src/

# clone this repository
$ cd ~/noetic_ws/src/
$ git clone https://github.com/ShaunAlt-Unitree-Go1/ROS-Bridge-Noetic.git
$ cd ~/noetic_ws/

# install dependencies
$ source /opt/ros/noetic/setup.bash # (use `source_noetic` if setup)
$ rosdep update --rosdistro noetic
$ rosdep install -y --from-paths src --ignore-src --rosdistro noetic

# build this workspace
$ catkin_make

# source this workspace
$ source devel/setup.bash
```

### Physical Robot Data Re-Publishers
The [relay_physical](./relay_physical) package provides scripts which are used when working with the physical Unitree Go1 Robot.

### Physical Robot Data Re-Publisher - Lidar Data
The [relay_physical/scripts/lidar_relay.py](./relay_physical/scripts/lidar_relay.py) script is used to convert the scan data from the lidar on the Go1 robot into the standard topics, prefixed with a given namespace.

>[!NOTE]
> When setting the namespace for this script, if you don't set the namespace or you set it to a blank string, then no namespace will be used.

``` bash
# source workspace as shown above
$ source devel/setup.bash

# set ros namespace
$ export ROS_NAMESPACE="robot1"

# run lidar relay
$ rosrun relay_physical lidar_relay.py
```

### Physical Robot Data Re-Publisher - Transforms
The [relay_physical/scripts/tf_republish.py](./relay_physical/scripts/tf_republish.py) script is used to republish any invalid transforms received from the lidar.

>[!IMPORTANT]
> For some reason, some of the Lidar modules will publish transforms with a timestamp that is 1e8 seconds in the past (e.g. 1.8e9 instead of 1.9e9). In these instances, all of these transforms are disregarded resulting in an incomplete transform tree of the robot. This causes the robot to be unable to localize itself.

>[!NOTE]
> When setting the namespace for this script, if you don't set the namespace or you set it to a blank string, then no namespace will be used.

``` bash
# source workspace as shown above
$ source devel/setup.bash

# set ros namespace
$ export ROS_NAMESPACE="robot1"

# run transform republisher
$ rosrun relay_physical tf_republish.py
```
