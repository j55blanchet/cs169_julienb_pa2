# PA2 - CS169
Julien Blanchet | Robotics Perception Systems | 2/2/20

<hr>

## Requirements
* Run on an Husarion ROSbot
* Ubuntu 16.04 with ROS Kinetic Installed

## Setup

This repository is intended to be a *single package* within a catkin repository. Therefore, you must use an exising catkin workspace or create a new one. Steps for setup:

1. `cd` to the `src` folder of your catkin workspace
1. copy the contents of this repositrory into a folder titled `julienb_kalmanfilter` (this will be the package name)
1. Download the bag file from https://1drv.ms/u/s!AiFiPYRO3Kyph49sCXQj4euRLGRaRA?e=ATnOvU and save it as `<workspaceroot>/src/julienb_kalmanfilter/bags/record.bag`
1. Run `catkin_make` at the root of your workspace.

## Running
1. `roslaunch julienb_kalmanfilter filter1.launch` to execute task #1
    * `mode:=b` The path estimated by the Kalman Filter, using cmd_vel and scan
    * `mode:=c` The path estimated by the Kalman Filter, using pose and scan
    * `mode:=d` The path estimated by the Kalman Filter, using cmd_vel
and scan extracted by the RGB-D camera using the depthimage_to_laserscan
    * `mode:=e` The path estimated by the Kalman Filter, using pose and
scan extracted by the RGB-D camera using the depthimage_to_laserscan