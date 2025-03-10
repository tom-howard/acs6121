---
title: "Creating your Team's Package"
description: "TODO" 
---

## Creating your Team's Package

You should use the same procedures as you did in the Simulation Labs to create a ROS package for this assignment (described again below). You should create **one package per team**, so nominate **one** member of your team to do this bit now.

You should do this from within your own local ROS installation (i.e. on your own computer), or a WSL-ROS2 terminal instance.

1. First, [make sure you have the `tuos_ros` Course Repo installed](../extras/course-repo.md#installing), and if you've already installed it previously then [make sure it's up-to-date](../extras/course-repo.md#updating).

1. Head to the `src` folder of your ROS2 workspace in your terminal and into the `tuos_ros` Course Repo from there:

    ```bash
    cd ~/ros2_ws/src/tuos_ros/
    ```

1. Use the `create_pkg.sh` helper script to create your team's ROS package (pay attention to the further information below about the naming of your package).

    ``` { .bash .no-copy }
    ./create_pkg.sh acs6121_teamXX_2025
    ```

    Your package **must** be named as follows:

    ``` { .txt .no-copy }
    acs6121_teamXX_2025
    ```

    ... where `XX` should be replaced with *your* team number (see Blackboard if you are unsure what your team's number is).

    **If your team number is less than 10**: put a zero before it, so that the team number is **always** 2 digits long, e.g.: 
    
    * `acs6121_team03_2025` for **Team 3**
    * `acs6121_team08_2025` for **Team 8**
    * `acs6121_team15_2025` for **Team 15**

    !!! warning "Important"
        Your repository name should match the above format **exactly**:
            
        * The name should be **19 characters long** in total.
        * All characters should be **lower case** (e.g. `acs6121`, **NOT** `ACS6121`)

1. Then navigate into the **root** of your new package:

    ``` { .bash .no-copy }
    cd ../acs6121_teamXX_2025/
    ```

    ...and create a new directory in there called `launch`:

    ```bash
    mkdir launch
    ```

1. Inside here create an empty file called `explore.launch.py`:

    ```bash
    touch ./launch/explore.launch.py
    ```

    ... leave this empty for now. You'll need to populate this appropriately later (more details on the Task Brief).

1. Open up your package's `CMakeLists.txt` file and add the following text **just above** the `ament_package()` line at the very bottom:

    ```txt title="acs6121_teamXX_2025/CMakeLists.txt"
    install(DIRECTORY
      launch
      DESTINATION share/${PROJECT_NAME}
    )
    ```

1. You can now build this using Colcon by following **the same three-step process** that you have been following throughout the Simulation Lab Course:

    1. **Step 1**, navigate to the **root** of the ROS2 workspace:

        ```bash
        cd ~/ros2_ws/
        ```

    1. **Step 2**, build your package with Colcon:

        ``` { .bash .no-copy }
        colcon build --packages-select acs6121_teamXX_2025 --symlink-install
        ```

    1. **Step 3**, re-source the `.bashrc`:

        ```bash
        source ~/.bashrc
        ```
    
    !!! tip "Don't forget"
        You'll need to follow the above **three-step** `colcon build` process whenever you do things like:

        1. Add a new node to your package (don't forget to modify the `CMakeLists.txt` file too)
        1. Add **or modify** a launch file
        1. Add **or modify** a custom interface (like in [Part 1](../sim/part1.md#ex7))
        1. Copy your package onto a different computer 

## See Also

* [Transferring Your ROS Package to the Robotics Laptops](./first-lab.md#transferring-your-ros-package-to-the-robotics-laptops)
* [Exporting Your ROS Package for Submission](./submission.md#exporting-your-ros-package-for-submission)