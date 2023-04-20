---
permalink: /lab-8/
title: "Lab 9: Mapping"
sidebar:
  nav: "lab-9"
---

The goal of this lab was to construct a map of the robots environment, which will crucial to localizing the robot for later labs. The main approach for this lab was to rotate the robot and read out depth measurements from the ToF sensors, similar to a LiDAR scan.

## Angular Velocity Control
Before reading out distance measurements, the robot needed a reliable way of rotating in place or at least on some fixed axis in a predictable way. 

Although this could be implemented using orientation control on yaw, the concern was that the drift from integrating gyroscope measurements would make these measurements very inaccurate over a long enough period of time. 

For this reason, a PID controller on the angular velocity of the robot was instead implemented, which hopefully would circumvent integrating the gyroscope readings:
```cpp
float yaw_vel_setpoint = 0;
float f_gx = 0;
void yaw_vel_control(){
  float windup_cap = 10000;
  float beta = 0.1;
  f_gx = beta*gx[previdx] + (1-beta)*f_gx; 
  float error = yaw_vel_setpoint-f_gx; 
  
  if (abs(KI*(sum_error + error)) <= windup_cap) {
    sum_error += error;
  }
   
  float alpha = 0.25;
  float diff_error = error - prev_error;
  prev_error = error;
  f_diff_error = alpha*diff_error + (1-alpha)*f_diff_error; 
  
  float p_term = KP*error;
  float i_term = KI*sum_error;
  float d_term = KD*f_diff_error;
  int pwm = round(p_term + i_term + d_term);

  pwml = -pwm;
  pwmr = pwm;
  writepwm(pwml,pwmr,100);
}
```

Because the gyroscope readings could be very noisy especially while the robot is moving, a complementary low pass filter was applied the the readings. 

Although this would make the sensor readings slower to sharp transitions in angular velocity, this is not really a concern since the goal is to minimize the angular velocity so that the robot can collect as many depth measurements as it can at each angle.

As a side note, the rotation was implemented by simply assigning pwm values with opposing signs to the left and right motors. 

To get the robot to properly turn on an axis close to its center, a small bias was applied to each of the motors that was carefully tuned. In practice, a bias of about -10 was usually sufficient.

After a substantial amount of tuning, it was found that the slowest velocity that could be achieved was about 50 degrees per second, where the corresponding PID parameters were $K_P = 1.0$, $K_I = 0.1$, and $K_D = 5.0$:

YOUTUBE

![Yaw Velocity](/lab-9-assets/yaw_velocity.png)

Note that to reduce the friction associated from the wheels skidding, a generous amount of duct tape was applied to the wheels, which can be seen from the video above.

## Distance Scan


## Mapping
