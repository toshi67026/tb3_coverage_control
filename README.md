# tb3_coverage_control

## Requirements
- Ubuntu20.04
- ROS Noetic
- Python3.8

```sh
cd ~/catkin_ws/src/tb3_coverage_control
python3 -m pip install -r requirements.txt
git submodule update --init
rosdep install -i -y --from-paths .
```

## Usage
### Real mode
#### tb3 coverage control
```sh
roslaunch tb3_coverage_control tb3_cc.launch real:=true
```

### Sim mode
```sh
roslaunch tb3_coverage_control tb3_cc.launch real:=false
```

### TODO list
- geometric coverage control
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
