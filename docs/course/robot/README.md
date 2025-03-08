---
title: "Team Assignment: Real World Exploration"
description: Apply your ROS2 knowledge to real TurtleBot3 Waffles in the lab.
--- 

## Overview

In Weeks 8-11 you will attend weekly two-hour lab sessions in **Diamond Computer Room 5 (DIA-CR5)**, where you will work **in teams** with our **Real TurtleBot3 Waffle Robots**, and program them to complete a *"Real World Exploration"* task. You'll use the ROS2 knowledge gained throughout [the Simulation Labs](../sim/README.md), as well as the theory you are learning in the ACS6121 lectures to help you with this.

In preparation for this, you will form your teams in Week 5, and work together during the labs in Weeks 5 & 6 to develop the necessary control algorithms *in simulation* first. The labs in Weeks 5 & 6 will take place in **Diamond Computer Room 6 (DIA-CR6)**.

Unlike the Simulation Labs in Weeks 2-4, there won’t be any taught content or weekly exercises for you to work through for the Real-Robot Labs: from Week 5 onwards you are expected to work more *autonomously* ^^in your teams^^. 

## Teams & Team Working

You are expected to actively work together with your team members to complete this assignment. Details on the teams can be found in the **"Real-Robot Labs"** section on the ACS6121 Blackboard Course Page.

## Lab Sessions

The lab schedule is summarised in the table below.

<center>

| Semester Week | Location | Details |
| :---:         | :---     | :---    |
| 5 & 6  | DIA-CR6 | Work in teams on the Real World Exploration task, *in simulation*.  | 
| 7  | DIA-CR6 | Individual Assessment for the Simulation Labs | 
| 8 - 11 | DIA-CR5 | Work in teams on the Real World Exploration task, with the *real Waffles*.  | 

</center>

From Week 8 onwards the class is divided into two groups: **1** and **2**, each group attending a *different* weekly lab session in Diamond Computer Room 5. Check your timetable to find out which weekly lab session to attend and **ONLY** attend the lab session that you are supposed to attend. You will attend the same weekly lab session as the rest of your team.

!!! warning "Important"
    You will only have **four lab sessions** to work with the real robots, and you **won't** be able to gain additional access the robots outside your scheduled lab sessions.
    
    As such, it is vital that you work together in your teams in advance, to prepare for this. 

    Before week 8, you will need to work in the lab **and** in your own time (with your team) to develop your ROS package for the assignment task, so that you have something ready to go in time for the first real robot lab in Week 8.

## Assessment

This assignment is **worth 20%** of your overall mark for the ACS6121 course.

Your team will need to submit a ROS package via Blackboard before the deadline at the end of Week 11 (**see Blackboard for the exact date and time**). The submitted ROS package will then be assessed by the teaching team. The team will run your ROS package three times, and the performance of the robot during all three runs will be considered in the marking. 

### Useful Links

* [Preparing your ROS package for Submission (TODO)]()
* [Marking Criteria (TODO)]()

   
## Getting Support

The best way for you to get support is to attend the weekly lab sessions, where the teaching team will be present to answer any questions that you may have. Although we are more than happy to discuss things and help you, it is not our job to debug your entire code on your behalf on the spot.

Remember to use the [ROS Simulation Lab course](../sim/README.md) as a reference throughout this work: taking the things that we worked on here and applying them to your team’s ROS package.

Finally, don’t forget to use the ACS6121 Discussion Board on Blackboard to post any questions you may have.

<!-- A separate page from here on...? -->

## Submission Requirements

The submission deadline for your team's ROS package is available on Blackboard. The Blackboard submission portal will become available in Week 10. In order to be assessed, you must ensure that the following requirements are met with regard to the ROS package that your team submits (as well as any additional requirements described in the later sections of this document):

* As mentioned earlier, everything your team submits for this lab must be contained within a single ROS package. Inside this you will develop all the necessary nodes that allow your robot to complete the exploration task.
* In the root of your team's package directory, there should be a `launch` folder; in the `launch` folder, there is a launch file with the following naming convention:
    
    `explore.launch`
    
    The task will be assessed by the teaching team using this launch file.
* Your team's ROS package must work ‘out-of-the-box’, i.e. the module leader and GTAs will not fix errors or modify your codes for you during the assessment.
* Your package must be submitted to Blackboard as a `.tar` file with the following naming convention:
    
    `acs6121_team{}.tar`
    
    Where the `{}` must be replaced with your own team number. Detailed instructions on how to convert your ROS package to .tar file are provided below.

## Exporting your ROS Package for Submission

When it comes to submission time, it's important that you follow the steps below carefully to create a `.tar` archive of your ROS package correctly. We recommend that you do this from WSL-ROS on a University Managed Desktop Computer (rather than one of the robot laptops), so that you can save it to your University `U:` Drive.

1.  First, navigate to the `catkin_ws/src` directory in a WSL-ROS terminal instance:
    
    `cd ~/catkin_ws/src/`
2.  Then, use the `tar` command to create an archive of your package:
    
    `tar -cvf /mnt/u/wsl-ros/acs6121_team{}.tar acs6121_team{}`
    
    ... replacing `{}` with your own team number again.
    
    This will create the `.tar` archive in your own personal University `U:` Drive, which you can access using the *Windows File Explorer*...
3.  In *Windows*, open up Windows Explorer, click ``This PC'' in the left-hand toolbar and locate your own personal `U:` Drive in the ``Network locations'' area.
4.  In here there should be a `wsl-ros` folder, which should contain the `.tar` file that you have just created.
5.  Submit this `.tar` file to Blackboard via the appropriate submission portal.

<center>

*Now, let's get started!*

</center>

## Getting Started

### Working with the Real Robots

Before Week 8, your team must have developed a fully working ROS package for testing on the real-robot and all your team members must have finished the health & safety quiz and its associated tasks before being allocated with the robot and its laptop.

During the lab sessions on weeks 8-10, your team will be given a Turlebot3 Waffle robot and a laptop. As mentioned earlier, to get started controlling the real robot, you can refer to [Working with the Real TurtleBot3 Waffles](https://lincaolab.github.io/acs6121/waffles/) for your reference (pay special attention to the out-of-the-range LIDAR data). There are a few exercises that you can have a go there too (