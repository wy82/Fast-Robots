---
permalink: /lab-9/
title: "Lab 9: Mapping"
sidebar:
  nav: "lab-9"
---

The goal of this lab was to construct a map of the robots environment, which will crucial to localizing the robot for later labs. The main approach for this lab was to rotate the robot and read out depth measurements from the ToF sensors, similar to a LiDAR scan.

## Angular Velocity Control
Before reading out distance measurements, the robot needed a reliable way of rotating in place or at least on some fixed axis in a predictable way. 

Although this could be implemented using orientation control on yaw, the concern was that the drift from integrating gyroscope measurements would make these measurements very inaccurate over a long enough period of time. This would especially become more of a problem if the robot moves very slow to acquire a higher resolution of the map.

For this reason, a PID controller on the angular velocity of the robot was instead implemented, which hopefully would circumvent integrating the gyroscope readings. 

While implementing the controller, it was observed that rotating along hte robot's center axis was especially difficult at slow angular velocities, as one of the motors would potentially stall while the other would turn without any predictable pattern. 

Since these asymmetries in motor performance would easily throw the rotation off the center axis, the right motor was held still so that the robot would turn around an axis centered between the left wheels.

Furthermore, because the vibrations of the motors could add significant amounts of noise to the gyroscope while moving, a complementary low pass filter was applied to the readings.

Although this would make the sensor readings slower to sharp transitions in angular velocity, this is not really a concern since the goal is to move as slowly as possible.

The above ideas were then implemented as the following function:

```cpp
float yaw_vel_setpoint = 0;
float f_gz = 0;
void yaw_vel_control(){
  float windup_cap = 100000;
  float beta = 0.25;
  f_gz = beta*gz[previdx] + (1-beta)*f_gz; 
  float error = yaw_vel_setpoint-gz[previdx];
  
  if (abs(sum_error + error*h) <= windup_cap) {
    sum_error += error*h;
  }
   
  float alpha = 0.25;
  float diff_error = (error - prev_error)/h;
  prev_error = error;
  f_diff_error = alpha*diff_error + (1-alpha)*f_diff_error; 
  
  float p_term = KP*error;
  float i_term = KI*sum_error;
  float d_term = KD*f_diff_error;
  int pwm = round(p_term + i_term + d_term);
  
  pwml = -pwm;
  pwmr = 0;
  writepwm(pwml,pwmr,40);
}
```


After tuning the controller, it was found that the slowest velocity that could be achieved was about 50 degrees per second, where the corresponding PID parameters were $K_P = 3.0$, $K_I = 0.001$, and $K_D = 0.0$. 

Assuming a sampling period of roughly 20 milliseconds (the measurement time budget in the "long" mode) for each distance sensor, this would correspond to 50 sensor readings per second and 360 sensor readings per rotation. If both ToF sensors are used, then this rate would essentially be doubled, although this would require finding the transformation matrices for both sensors precisely.

Below we demonstrate the performance of the controller:

<iframe width="560" height="315" src="https://www.youtube.com/embed/MxB0IlATCDk" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

![Rotation](/lab-9-assets/Rotation.png)

Note that to reduce the friction associated from the wheels skidding, a generous amount of duct tape was applied to the wheels, which can be seen from the video above. 

Furthermore, although the robot could potentially move slower, the noise of the gyroscope readings as seen in the figure above range up to 20 degrees per second from the nominal setpoint of 50 degrees per second. Since this would essentially mean a relative error of nearly 80% at 25 degrees per second, the setpoint velocity was increased to hopefully reduce the errors in the angular velocity.

Finally, the gyroscope readings also demonstrate periodic dips in angular velocity, which seem to be due to the wheels getting stuck at a certain point of the rotation. This might have been because the tape was applied somewhat unevenly, or possibly due to the mechanism of the wheels or motor. 

While increasing the integral term would in theory provide an extra kick in motor power to reduce the magnitude of the dips, this would also increase the overshoot on the way out of the dip, which would require a delicate balance with the dampening of the derivative term. 

Unfortunately, I was ultimately unable to eliminate these dips from the output even after a substantial amount of tuning.

## Distance Scan

## Mapping
