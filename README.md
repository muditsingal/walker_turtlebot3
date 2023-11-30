# walker_turtlebot3
ENPM808x ROS assignment 4

## Overview
This repo contains the walker algorithm that helps in basic movement of the turtlebot 3 done with ROS2 and C++. <br>
The source files are: walker_turtlebot3.cpp <br>
Launch file to launch the turtlebot3, gazebo, and walker node <br>
cpp lint, and cpp check output is present in *results* folder. <br>
Recorded rosbags are in *bagfiles* folder

## Steps to run

1. Clone the repo using `git clone https://github.com/muditsingal/walker_turtlebot3.git` inside your _ros2\_ws/src_ folder
2. Build your workspace using `colcon build` from the ros2_ws folder
3. Open 2 terminals
4. Launch the turtlebot3 with all sensors, and walker node using the command: `ros2 launch walker_turtlebot3 walker_turtlebot3.launch.py`
5. A simulation using gazebo will open with the desired turtlebot3 behaviour. Rosbag will be recorded as well.

## Steps to disable rosbag recording

`ros2 launch walker_turtlebot3 walker_turtlebot3.launch.py --rosbag_record:=False`

## CppLint, CppCheck, and Clangd formattting

```bash
# Cpp Lint
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cpplint_output_a3.txt

# Cpp Check
cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cppcheck_output_a3.txt

# Clangd format
clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^./build/")
```


## Dependencies
> ROS2 Humble <br>
> rclcpp <br>
> cpplint <br>
> cppcheck <br>
> gazebo <br>
> Turtlebot3 <br>
> Turtlebot3_gazebo <br>
> C++14 or greater <br>
