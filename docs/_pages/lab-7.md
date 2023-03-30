---
permalink: /lab-7/
title: "Lab 7: Kalman Filter"
sidebar:
  nav: "lab-7"
---
This lab was dedicated to implementing a Kalman filter, which will help extrapolate the collected sensor measurements and help reduce the latency between each input to the PID controller.

## System ID
Before implmenting the Kalman filter, it was crucial to first identify the state equations associated with the position $x$ of the robot. These are chararacterized by a simple drag equation with input velocities $u$, which is stated as follows:

$$\begin{align}
\begin{bmatrix}
\dot{x} \\
m\ddot{x}
\end{bmatrix}
&=
\begin{bmatrix}
x \\
-d\dot{x} + u
\end{bmatrix}
\end{align}$$

This can then be expressed as the following linear state space equation:

$$\begin{align}
\begin{bmatrix}
\dot{x} \\
\ddot{x}
\end{bmatrix}
&=
\begin{bmatrix}
0 & 1 \\
0 & -\frac{d}{m}
\end{bmatrix} 
\begin{bmatrix}
x \\
\dot{x}
\end{bmatrix}
+
\begin{bmatrix}
0
\frac{1}{m}
\end{bmatrix}
u
\end{align}$$

Here, the drag $d$ and mass $m$ of the robot are not initially known, and must be measured via taking the step response of the system, which was done by sending a stream of constant PWM values to the motors.

This was basically accomplished by setting the PWM values of the robot to the maximum values achieved using the position PID controller developed in Lab 6, which were around 126 after shifting and rescaling to account for the motor's PWM deadband:

Youtube

The ToF sensor data was then collected as the robot approached a wall, and the resulting sensor outputs, motor inputs were plotted. The velocity of the robot was also estimated by taking finite differences of the ToF data. 

 ToF Data                                  | PWM Data                                  | Calculated Speed                              | Averaged Speed                                            |
|:----------------------------------------:|:-----------------------------------------:|:---------------------------------------------:|:---------------------------------------------------------:|
![Carpet ToF](/lab-7-assets/carpet_tof.png)|![Carpet PWM](/lab-7-assets/carpet_pwm.png)|![Carpet Speed](/lab-7-assets/carpet_speed.png)|![Carpet Average Speed](/lab-7-assets/carpet_avg_speed.png)|


Since the drag coefficient could depend quite heavily on the kind of surface the robot is traveling on, the process was repeated on a smoother floor:

Youtube

 ToF Data                                | PWM Data                                | Calculated Speed                            | Averaged Speed                                          |
|:--------------------------------------:|:---------------------------------------:|:-------------------------------------------:|:-------------------------------------------------------:|
![Floor ToF](/lab-7-assets/floor_tof.png)|![Floor PWM](/lab-7-assets/floor_pwm.png)|![Floor Speed](/lab-7-assets/floor_speed.png)|![Floor Average Speed](/lab-7-assets/floor_avg_speed.png)|

## Setup

## Testing

## Implementation

## Speedup

