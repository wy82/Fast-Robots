---
permalink: /lab-12/
title: "Lab 12: Path Planning and Execution"
sidebar:
  nav: "lab-12"
---

For the final lab, the goal was to utilize the algorithms and tools developed in previous labs to compute a motion planning task in the lab maze, where the robot would navigate to 9 marked waypoints.

## Waypoints
We first provide a list of the waypoints:

```python
1. (-4, -3)    
2. (-2, -1)
3. (1, -1)
4. (2, -3)
5. (5, -3)
6. (5, -2)
7. (5, 3)
8. (0, 3)
9. (0, 0)
```

These can be visualized with the following image:

![Waypoints](/lab-12-assets/Waypoints.png)

## Path Planning

To ensure that at least one successful run would be completed, the first approach was to implement a naive trajectory consisting of only forward movements and pure rotations.

The robot would then reach the waypoints via PID controllers on the distance to the walls and the yaw from the IMU.

To simplify the trajectory even further, we use an idea borrowed from Ryan Chan (class of 2022 offering) to ensure only 90 degree rotations are required:

```python
1. (-4, -3)    
2. (-4, -1)
3. (2, -1)
4. (2, -3)
5. (5, -3)
6. (5, 3)
7. (0, 3)
8. (0, 0)
```

![Trajectory](/lab-12-assets/Trajectory.png)

In a sense, using PID control on the distance from the walls is almost no different from using a localization algorithm, since it uses distance measurements and knowledge about the map to guide the robot to each waypoints.

The main difference is that the localization algorithm is also capable to discerning the exact angular pose of the robot, whereas simple PID control relies on the measurements from the IMU to get the robot's orientation. The other disadvantage with this approach is that it makes no effort to correct the orientation of the robot while driving forward, which puts it at risk to collide with walls.

To program the simplified trajectory on the robot, this required essentially programming every single action. 

A different way to program the trajectory that could better scale with large numbers of waypoints would be to write a script to calculate the exact angle or distance setpoints to reach the next waypoint.

Below we include a list of the setpoints and actions used to complete the trajectory:

```python
1. Distance Setpoint: 
2. Angular Setpoint:
3. Distance Setpoint;
4. Angular Setpoint:
5. Distance Setpoint:

```

## Localization

