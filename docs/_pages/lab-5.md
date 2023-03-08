---
permalink: /lab-5/
title: "Lab 5: Motors and Open Loop Control"
sidebar:
  nav: "lab-5"
---
In this lab, the motor drivers were installed, which allowed the Artemis board to finally send direct commands to drive the car. Although the motors were mainly driven using open loop control here, the goal is to eventually integrate the sensors into a more robust feedback loop.

## Setup:

Before the Artemis board can send commands to the motors, motor drivers (DRV8333) for each motor were installed. 

Although we are using motor drivers each with dual outputs, we connect two outputs in parallel to each of the motors to increase the maximum output current and power they can source. As shown in the diagram, this essentially amounts to shorting the outputs A and B for each motor driver: 
 
![Wiring Diagram](/lab-5-assets/Wiring_Diagram.png)

The Artemis board was then connected to the motor drivers via pins 4, A5, A15, and A16, all of which are hardware PWM pins capable of sending analog signals via analogWrite().

In addition to the motor drivers, a separate 3.7 V 850 mAh battery was used to power the motor drivers and motors. 

The main advantages in doing so are a higher power output due to the lack of power regulation from the Artemis board. Furthermore, the higher battery capacity (850 mAh vs 650 mAh) provides a longer battery lifetime, allowing the motors to run for longer in general. Finally, a separate power supply could potentially isolate the Artemis board from possibly power supply noise generated from the power draw of the motor.

## Motor Drivers:

![Test Setup](/lab-5-assets/Test_Setup.png)

```cpp
```

![Oscilloscope](/lab-5-assets/Oscilloscope.png)

## Wiring:

![Car](/lab-5-assets/Car.png)

## Calibration:

## Open Loop Control: 

