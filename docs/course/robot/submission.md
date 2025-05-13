---
title: "Week 11: Submission Details (& Key Requirements)"
description: "How to submit your ROS package for the Real-Robot Lab Assignment"
---

## Deadline

The deadline for submission of your ROS package for this task is **23:59 on Friday of Week 11**.

You should submit your team's ROS package to Blackboard, by following the steps outlined below. You should make **one submission per team**, so nominate one member of the team to do this.

Your submitted ROS package will be assessed by the teaching team off-line during the Spring Exam Period. Before the end of the exam period you will receive your marks, plus video recordings of the assessment.

##  Key Requirements

!!! warning
    Failure to follow all the requirements listed on this page could result in **penalties** being applied to your mark, or you being awarded **zero marks** for the assignment task!

Before submitting your work, you **must** ensure that the following *Key Requirements* are met in regard to your ROS package: 

* [ ] The name of your ROS package must be:

    ``` { .txt .no-copy }
    acs6121_teamXX_2025
    ```

    ... where `XX` should be replaced with your team number.

    <a name="build-files"></a>

* [ ] Your package must contain **no build files** (`build/`, `install/`, `log/`) that would be generated as a result of running `colcon build` from inside your package.

    !!! warning "Remember"
        **Always** run `colcon build` from the **root** of the ROS workspace (e.g. `~/ros2_ws/`), to ensure that all build files are generated in the right location in the filesystem (`~/ros2_ws/build/`, `~/ros2_ws/install/`, `~/ros2_ws/log/`).

For the assessment of the task, your package will be built and deployed on one of the Robotics Laptops that you'll have been working with during the lab sessions. We will use the standard `student` user account, and your package will be downloaded to the `~/ros2_ws/src/` directory. 

* [ ] It must be possible to build your package by running the following command from the root of the local ROS2 Workspace, and this must build without errors:
    
    ``` { .bash .no-copy }
    colcon build --packages-select acs6121_teamXX_2025
    ```

* [ ] You must ensure that a launch file exists for the task and that this is executable (after having run the above `colcon build` command) so that we are able to launch your work as follows[^launch-files]:
    
    [^launch-files]: Make sure you have [defined an appropriate `install` directory in your package's `CMakeLists.txt`](../sim/part3.md#ex1) 

    ``` { .bash .no-copy }
    ros2 launch acs6121_teamXX_2025 explore.launch.py
    ```

    ... where `XX` will be replaced by your team number.

* [ ] Any nodes within your package that are executed by the above launch files **must** have been correctly defined as package executables (i.e. in your `CMakeLists.txt`) and must **also** have been assigned the appropriate execute permission (i.e. with `chmod`).  

    !!! warning 
        It's up to **you** to ensure that your code launches as intended for a given task. If it doesn't, then you'll be awarded zero marks, so **make sure you test it all out prior to submission**!

## Other Important Information 

* The [`tuos_ros` Course Repo](../extras/course-repo.md) will be installed and up-to-date on the Robotics Laptop that we use to assess your work with.

* The Robotics Laptop that we use for the assessment will be selected at random.

* This laptop will have been paired with a robot prior to us attempting to run your submission.

* The robot will also be selected at random.

* We will have already [launched the *bringup* on the robot](../../waffles/launching-ros.md#step-3-launching-ros), so ROS will be up and running, and the robot will be ready to go in the arena.

* [A bridge between the robot and laptop will have already been established](../../waffles/launching-ros.md#step-4-robot-laptop-bridging), and communications will be tested, prior to us attempting to launch your work for each task.

## Exporting your ROS Package for Submission

When it comes to submission time, it's important that you follow the steps below carefully to create an archive of your ROS package correctly. 

1. From your local ROS installation (i.e. WSL-ROS2), run the `tar` command to compress your team's package into a `.tar` file:

    ``` { .bash .no-copy }
    tar -cvf ~/acs6121_teamXX_2025.tar -C ~/ros2_ws/src/ acs6121_teamXX_2025
    ```
    
    ... replacing `XX` with your own team number, of course!

    This will create a `.tar` archive of your package in your home directory. 

1. **If you are doing this from within WSL-ROS2 (i.e. on Windows)**, access this using the Windows File Explorer. In the same terminal as above enter the following command to first navigate to the home directory, and then launch the Windows Explorer in this location:

    ```bash
    cd ~ && explorer.exe .
    ```

1. An Explorer window should then open, and in there you should be able to see the `acs6121_teamXX_2025.tar` file that you just created. Copy and paste this to somewhere convenient on your machine.

6. Submit this `.tar` file to the appropriate submission portal on Blackboard:

<center>

**Blackboard** > **ACS6121** > **Assessment** > **Real-Robot Lab Assignment Submission**

</center>

