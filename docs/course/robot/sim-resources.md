---
title: "Simulation Resources"
description: "TODO"
---

Within the `tuos_simulations` package there is an example arena which can be used to develop and test out your team's work for this task.

!!! info 
    Make sure you [check for updates to the Course Repo](../extras/course-repo.md#updating) to ensure that you have the most up-to-date version of this.

The simulation can be launched using the following `ros2 launch` command:

```bash
ros2 launch tuos_simulations acs6121.launch.py
```

<figure markdown>
  ![](./figures/explore_sim.jpg){width=700px}
  <figcaption>The development arena for the Real-World Exploration Task.</figcaption>
</figure>

You can test out different starting orientations for the robot by supplying an additional command line argument to the `acs6121.launch.py` file: 

## Simulation vs the Real World

This simulation is provided as a way for you to develop your algorithms in simulation during the Weeks 5 & 6 labs, and to allow you to do some work on this assignment outside the lab sessions too. However, **just because it works in simulation ^^DOESN'T^^ mean it will work in the real world**!

Make sure you test things out thoroughly during the real-robot lab sessions in Weeks 8-11.
