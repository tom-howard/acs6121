---
title: "Week 8: Your First Real-Robot Lab"
description: "TODO"
---

## Getting to Know the Real Robots

This assignment task involves extensive work with our real robots, and you'll therefore have access to the robots for every lab session from Week 8 onwards. All the details on how the robots work, how to get them up and running and start programming them can be found in the "Waffles" section of this course site, which we'll get to in a minute.

Each team has been assigned a specific robot (there's a list on Blackboard). The teaching team will provide you with your robot (and a *robot laptop*) at the beginning of the lab session. 

!!! warning "Important"
    * The robot and laptop are only accessible during the lab sessions. You must return all hardware at the end of the lab, and you won't be able to access the hardware at any other times.
    * Lab session start on the hour, and are **1 hour and 50 minutes** long. You must ensure that all hardware is turned off and returned to teaching staff **promptly** at the end of the lab session (**at 10 minutes to the hour**). Follow the proper shutdown procedures to power off the hardware at the end of each lab session.

### Your Week 8 To-Do List

Once you have been provided with your robot and laptop, work through each page of [the "Waffles" section of this site](../../waffles/README.md) (**in order**) to familiarise yourselves with how they work:
   
* [ ] Read about [the hardware](../../waffles/intro.md).
* [ ] Learn how to [launch ROS and get the robots up and running](../../waffles/launching-ros.md).
* [ ] Work through the [Waffle (& ROS) Basics](../../waffles/basics.md), which will help to get you started and understand how ROS and the robots work.
* [ ] There is also some further *Essential Information* that you must all be aware of when working with the real robots. Work through [the further exercises here](../../waffles/essentials.md) now.
* [ ] Finally, review the [Shutdown Procedures](../../waffles/shutdown.md). Follow these steps to shut down the robot and power off the robotics laptop at the end of each lab session.

## Transferring Your ROS Package to the Robotics Laptops

In Weeks 5 & 6 you will have started to develop the necessary algorithms for this task, and you'll have tested these out in a simulated world. The next step is to get things working on the real robot!

Having familiarised yourselves with how the robots work (by working through the exercises above), the next task is to get your package installed on the robotics laptop to test your algorithms out in the real world, and start to debug and optimise.

There are a few methods that you could use to copy your ROS package onto the robot laptop, the easiest of which is to use a USB flash drive, or to upload your code to Google Drive and then download it on the laptop[^git].

[^git]: Another alternative is to use Git and GitHub, but this requires knowledge of these particular software tools. If you want to take this approach, then you would need to turn your ROS package into a git repo, push it to GitHub and then [refer to these instructions for cloning your package onto the robotics laptops](https://tom-howard.github.io/com2009/course/assignment2/ros-pkg-tips/){target="_blank"}.

There is a ROS2 Workspace on each of the robot laptops (much the same as in the WSL-ROS2 environment), and your package must reside within the `ros2_ws/src` directory (much like ALL the packages that you created throughout the simulation labs!) 

The best way to transfer your package between systems is as a `.tar` file. The following steps illustrate how this could be done:

1. From your local ROS installation (i.e. WSL-ROS2), run the `tar` command to compress your team's package into a `.tar` file:

    ``` { .bash .no-copy }
    tar -cvf ~/acs6121_teamXX_2025.tar ~/ros2_ws/src/acs6121_teamXX_2025
    ```
    
    ... replacing `XX` with your own team number, of course!

    This will create a `.tar` archive of your package in your home directory. 

2. If you're using WSL-ROS2 (or any other WSL-based ROS installation) then you can then access this using the Windows File Explorer. In the terminal enter the following command:

    ```bash
    cd ~ && explorer.exe .
    ```

    You can then copy the `acs6121_teamXX_2025.tar` file to a portable location, which you can then transfer to the robot laptop.

3. Copy the `acs6121_teamXX_2025.tar` file onto the robotics laptop, ideally to the `Downloads` folder.

4. Open up a terminal instance on the laptop, either by using the ++ctrl+alt+t++ keyboard shortcut, or by clicking the Terminal App icon in the favourites bar on the left-hand side of the desktop:
    
    <figure markdown>
      ![](../../images/laptops/terminal_icon.svg){width=60px}
    </figure>

5. Use `cd` to navigate to the folder that you copied your `acs6121_teamXX_2025.tar` file to, e.g.:

    ```bash
    cd ~/Downloads/
    ```

6. Then, use the `tar` command again to extract your ROS package into the ROS2 Workspace:

    ``` { .bash .no-copy }
    tar -xvf acs6121_teamXX_2025.tar -C ~/ros2_ws/src/
    ```

7. Follow the usual **three-step** `colcon build` process:
    
    1. Navigate into the ROS2 Workspace:

    ```bash
    cd ~/ros2_ws/ 
    ```

    1. Build your package:

    ``` { .bash .no-copy }
    colcon build --packages-select acs6121_teamXX_2025 --symlink-install 
    ```

    1. And finally, don't forget to re-source:

    ```bash
    source ~/.bashrc
    ```
