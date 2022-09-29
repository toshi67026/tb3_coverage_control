# tb3_coverage_control
Coverage control with turtlebot3 burger

## Requirements
- Ubuntu20.04
- ROS Noetic
- Python3.8

## Installation
```sh
cd ~/catkin_ws/src/tb3_coverage_control
python3 -m pip install -r requirements.txt
git submodule update --init
rosdep install -i -y --from-paths .
```

## Usage
### simple coverage control
#### Real mode
1. `roscore` on remote pc

2. bring up tb3_{id}
    ```sh
    ROS_NAMESPACE=tb3_{id} roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_{id}" set_lidar_frame_id:="tb3_{id}/base_scan"
    ```

3. launch on remote pc
    ```sh
    roslaunch tb3_coverage_control tb3_scc.launch real:=true
    ```

#### Sim mode
```sh
roslaunch tb3_coverage_control tb3_scc.launch real:=false
```

### geometric coverage control
#### Real mode
3. launch on remote pc
    ```sh
    roslaunch tb3_coverage_control tb3_gcc.launch real:=true gp:={line, gauss, circle}
    ```

4. Change geometric parameters through joy-con manipulation

#### Sim mode
```sh
roslaunch tb3_coverage_control tb3_gcc.launch real:=false gp:={line, gauss, circle}
```

### TODO list
- persisten coverage control
- agent prefix

## tools
### mypy
```sh
./tools/run_mypy.sh
```

### black
```sh
./tools/run_black.sh
```
