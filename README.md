# 3D-visualization-of-depth-maps
3D ROS visualization of depth maps which are created by any depth model such as Monodepth, Packnet, etc.. All steps including build are written one by one. To build source code, **catkin** is used. All required links are added to references. 

## Requirements
- ROS (tests are made with Kinetic)

## BUILD
create a workspace folder and create catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
source /opt/ros/kinetic/setup.bash 
catkin_make
```

2 folder should be created with *catkin_make* command: "build, devel". `src` folder contains all packages. Hence, we should create a package in `src`. 

```
cd src
catkin_create_pkg visualize_depth_map roscpp visualization_msgs
```

`visualize_depth_map` folder should be created. 

```
cd visualize_depth_map/src
cp ~/Downloads/constructing_map.cpp .
cd ~/catkin_ws
catkin_make
```

Build is done! Let's try: Open 3 Terminal

1. Terminal
```
source /opt/ros/kinetic/setup.bash 
roscore
```

2. Terminal
```
source /opt/ros/kinetic/setup.bash 
rosrun rviz rviz 
```

3. Terminal
```
source 
rosrun visualize_depth_map constructing_map.cpp <first frame number> <frequence of frame (unit : Hertz)>
```

Example:
```python
# means starting from first frame and 30 frame per second
rosrun visualize_depth_map constructing_map.cpp 0 30 
```


## References
- [Creating catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- [Creating catkin package](http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage)
- [Ros RViz Tutorial](http://wiki.ros.org/rviz/Tutorials/Markers%3A Points and Lines)