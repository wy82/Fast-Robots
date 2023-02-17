---
permalink: /lab-3/
title: "Lab 3: Time of Flight Sensors"
sidebar:
  nav: "lab3"
---
This lab was primarily focused on setting up the Time of Flight (ToF) (VL53L1X) sensors used to make depth measurements. The goal is ultimately give sensory feedback about the robot's surrounding environment, which will play a crucial role in localization and mapping.

## Setup:

To start off, a quick look at the sensor datasheet
[datasheet]()

## Wiring:

## I2C Address:

![Arduino I2C](/lab-3-assets/Arduino_I2C.png)

## Sensor Characteristics:

![Range Graph](/lab-3-assets/Range_Graph.png)

| Byte Count  | C++ Datatypes (32-bit processor)  | Maximum Storage Time (s) |
| ----------- | --------------------------------- | ------------------------ |
| 1           | char                              | 2.56                     | 
| 2           | short                             | 1.28                     |
| 4           | single-precision float, int, long | 0.64                     |
| 8           | double-precision float            | 0.32                     |

## Two ToF Sensors:

<iframe width="560" height="315" src="https://www.youtube.com/embed/3vamad-_anY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscop    e; picture-in-picture; web-share" allowfullscreen></iframe>

## Sensor Speed:

## Bluetooth:
