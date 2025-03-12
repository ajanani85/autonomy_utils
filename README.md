# ros2_sandbag

## Overview
`ros2_sandbag` is a ROS 2 package that provides template solutions for your problems.

## Installation 
Simply, clone this package into your ros2 workspace and build

## Generating Doxigen Documentation

There are two aspects when talking about doxigen core documentations

### 1. Function Documentation
In this view, you provide documentation for each function in the code.

If you use VSCODE, install "doxygen documentation generator" extension, and 

above your function: 

1. type /**

2. press enter


### 2. Package Documenation (HTML generation)

Install the following packages:

```bash
sudo apt install doxygen graphviz xdg-utils

colcon build --packages-select ros2_sandbag --cmake-target doc
```

Pay attention to Doxyfile.in and CMakeList.txt. Specifically, pay attention to how the INPUT is defined inside the Doxyfile.in

The colcon build creates your docs in ~/ros2_ws/build/ros2_sandbag/docs/html. To inspect your files: 

```bash
cd ~/ros2_ws/build/ros2_sandbag/docs/html

python3 -m http.server 8080

```

Now, open a webpage on your browser and copy and paste the following address:

```bash
http://localhost:8080/index.html
```

## Running gtest

1. To run gtest, make sure to add the required flags and options to your package.xml and CMakeList.txt.

2. To build with for testing: first, build it normally:

```bash
colcon build --packages-select ros2_sandbag
```

3. run the test and view the results:

```bash
colcon test --packages-select ros2_sandbag --event-handlers console_cohesion+
```

## Using the Scripts

In order to use the scripts, source setup.bash in the scripts directory


