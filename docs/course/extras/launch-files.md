---
title: "Launch Files"
---

## Introduction

As we know from the work we've done in the Simulation Labs, ROS applications can be executed in two different ways:  

1. Using the `ros2 run` command:

    ``` { .bash .no-copy }
    ros2 run {Package name} {Node name}
    ```

1. Using the `ros2 launch` command:

    ``` { .bash .no-copy }
    ros2 launch {Package name} {Launch file}
    ```

The `ros2 launch` command, used in combination with *launch files*, offers a few advantages over `ros2 run`, for example:

1. **Multiple nodes** can be executed **simultaneously**.
1. From within one launch file, we can call *other* launch files.
1. We can pass in **additional arguments** to launch things conditionally, or to change the launch behaviours.

Points 1 and 2 above are explored in [Part 3](../sim/part3.md) (Exercises 1 & 2). In this section we'll explore Point 3 further[^more].

[^more]: For more advanced launch file features, [have a look at this guide](https://github.com/MetroRobots/rosetta_launch){target="_blank"}.


## Identifying Launch Arguments

We can use the `-s` option with `ros2 launch` to discover the additional arguments that can be supplied to any given launch file. Take the `waffle.launch.py` launch file from `tuos_simulations`, for example:

```bash
ros2 launch tuos_simulations waffle.launch.py -s
```

You should be presented with a range of arguments here, starting with:

``` { .txt .no-copy }
$ ros2 launch tuos_simulations waffle.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):
urdf_file_name : turtlebot3_waffle.urdf

    'with_gui':
        Select whether to launch Gazebo with or without Gazebo Client (i.e. the GUI).
        (default: 'true')
```

Scroll to the *bottom* of the list, and you should see the following:

``` { .txt .no-copy }
    'x_pose':
        Starting X-position of the robot
        (default: '0.0')

    'y_pose':
        Starting Y-position of the robot
        (default: '0.0')

    'yaw':
        Starting orientation of the robot (radians)
        (default: '0.0')
```

Using these arguments, we can control the position and orientation of the Waffle when it is spawned into the simulated world. Try this:

```txt
ros2 launch tuos_simulations waffle.launch.py x_pose:=1 y_pose:=0.5
```

The robot should spawn into an empty world, but at coordinate position $x=1.0$, $y=0.5$, rather than $x=0$, $y=0$, as would normally be the case.

## Launching Launch Files from Within Launch Files!

This was covered in [Part 3 Exercise 2](../sim/part3.md#ex2), where we learnt how to launch an "Empty World" simulation from within our own launch file (and also launch a velocity control node from one of our own packages alongside this).

## Passing Launch Arguments

How do we pass an argument to a launch file (`tuos_simulations/waffle.launch.py`, for example) that we are executing from within *another* launch file? 

As per [Part 3 Exercise 2](../sim/part3.md#ex2), a basic launch file would look like this (in this case configured to launch `tuos_simulations/waffle.launch.py`):

```py title="launch_args_example.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("tuos_simulations"), 
                    "launch", "waffle.launch.py"
                )
            )
        )
    ])
```

To launch this *and* supply the `x_pose` and `y_pose` launch arguments to it as well, we need to add the following:

```py title="launch_args_example.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("tuos_simulations"), 
                    "launch", "waffle.launch.py"
                )
            ),
            launch_arguments={ # (1)!
                'x_pose': '1.0',
                'y_pose': '0.5' # (2)!
            }.items() 
        )
    ])
```

1. Arguments are passed to the launch file via the `launch_arguments` option of `IncludeLaunchDescription()`.
2. Arguments are passed as a dictionary, which can contain multiple key value pairs separated by commas: `#!py dict = {key1:value1, key2:value2, ... }`. 
    
    In this case, *keys* are the names of the launch arguments to be passed to the `waffle.launch.py` launch file, and **values** are the actual values we want to assign to those arguments (and which can be changed as required).