---
permalink: /lab-6/
title: "Lab 6: Closed-loop Control (PID)"
sidebar:
  nav: "lab-6"
---
The objective of this lab was to implement a simple PID controller for the robot, which controlled the robot's position from a wall at a setpoint of 200 mm.

## Setup:

To facilitate the debugging and data analysis process, a simple Bluetooth codebase was created to ensure that the Artemis board would periodically send timestamped arrays of sensor data, including the ToF, IMU, motor PWM levels, and PID controller parameters. 

To store the data, separate arrays for each kind of data were created with a maximum size set safely below the maximum dynamic memory storage capacity of the Artemis board. 

```cpp
if (myICM.dataReady() && distanceSensor1.checkForDataReady() && distanceSensor2.checkForDataReady() && idx < MAX_SIZE) {
    myICM.getAGMT();
      
    // Time
    t[idx] = int(millis());

    // IMU
    ax[idx] = (&myICM)->accX();
    ay[idx] = (&myICM)->accY();
    az[idx] = (&myICM)->accZ();
    gx[idx] = (&myICM)->gyrX();
    gy[idx] = (&myICM)->gyrY();
    gz[idx] = (&myICM)->gyrZ();

      // ToF
    int distance1 = distanceSensor1.getDistance();
    int distance2 = distanceSensor2.getDistance();
    d1[idx] = distance1;
    Serial.print("D1: ");
    Serial.println(distance1);
    d2[idx] = distance2;
    Serial.print("D2: ");
    Serial.println(distance2);

    // Motor
    ml[idx] = pwml;
    mr[idx] = pwmr;
     
    prev = idx;
    idx = (idx+1)%MAX_SIZE;
}
```

Once the data was received on the laptop, it was subsequently processed into arrays in Python and then saved into a CSV file for future analysis.

```python
async def get_data(uuid,byte_array):
    global time_millis
    global acc_mg, gyr_dps, depth1_mm, depth2_mm, temp5s_str
    global kp, ki, kd, motorl, motorr
    MAX_SIZE = 1000
    vecfloat = np.vectorize(lambda x: float(x[3::]))
    
    # save data
    if len(motorl) >= MAX_SIZE:
        with open(filename, 'ab') as file:
            pickle.dump([time_millis,acc_mg,gyr_dps,depth1_mm,depth2_mm,kp,ki,kd,motorl,motorr], file)
        time_millis = []
        acc_mg,gyr_dps,depth1_mm,depth2_mm = [],[],[],[]
        kp,ki,kd,motorl,motorr = [],[],[],[],[]
        temp5s_str = ''
    
    # parse string
    temp5s_str = ble.bytearray_to_string(byte_array)
    temp5s_list = temp5s_str.split("|")
    mode = temp5s_list[0]
    time = temp5s_list[1]
    
    if int(mode):
        a,g,d = temp5s_list[2:5],temp5s_list[5:8],temp5s_list[8:10]
        time_millis.append(int(time[2::]))
        if len(acc_mg) == 0:
            acc_mg = np.reshape(vecfloat(a),(3,1))
        else:
            acc_mg = np.hstack((acc_mg,np.reshape(vecfloat(a),(3,1))))
        if len(gyr_dps) == 0:
            gyr_dps = np.reshape(vecfloat(g),(3,1))
        else:
            gyr_dps = np.hstack((gyr_dps,np.reshape(vecfloat(g),(3,1))))
        depth1_mm.append(int(d[0][3::]))
        depth2_mm.append(int(d[1][3::]))
    else:
        p,i,d = temp5s_list[1:4]
        m = temp5s_list[4:6]
        kp.append(float(p[3::]))
        ki.append(float(i[3::]))
        kd.append(float(d[3::]))
        motorl.append(int(m[0][3::]))
        motorr.append(int(m[1][3::]))
```

Unfortunately, compiling all of the code and dependencies needed to facilitate Bluetooth communication on the Artemis required inconveniently large compile times. 

This would make testing various PID parameters somewhat painfully slow and inefficient, as there would otherwise essentially be a 5 minute wait time between each test run.

To circumvent this issue, a set of Bluetooth commands was developed that could be activated via keyboard commands on the laptop.

 This included simple commands for remote control access, activating various control modes, and tuning PID parameters: 

```python
# PWM (or PID) tuning using the arrow keys
elif keyboard_input == "up":
    ble.send_command(CMD.P_CONTROL , "")
elif keyboard_input == "down":
    ble.send_command(CMD.PI_CONTROL , "")
elif keyboard_input == "left":
    ble.send_command(CMD.PID_CONTROL , "")
elif keyboard_input == "right":
    ble.send_command(CMD.STOP, "")
    ble.send_command(CMD.SEND , "")
elif keyboard_input == "p":
    x = input()
    ble.send_command(CMD.SET_KP, x)
    ble.send_command(CMD.STOP, "")
elif keyboard_input == "i":
    x = input()
    ble.send_command(CMD.SET_KI, x)
    ble.send_command(CMD.STOP, "")
elif keyboard_input == "c":
    x = input()
    ble.send_command(CMD.SET_KD, x)
    ble.send_command(CMD.STOP, "")
```

The use of the nonstandard keyboard module and the general outline of the Python code was heavily inspired by the helpful template script provided by Raphael Fortuna, who generously shared this idea and provided a code template to help make the testing session somewhat more bearable.

## P Controller:

In implementing a proportional (and eventually a PID controller), the following function was inserted into the control loop:

```cpp
float sum_error = 0;
float prev_error = 0;
float f_diff_error = 0;
void pidcontrol(){
  float setpoint = 200;
  float windup_cap = 1000;
  float error = d2[previdx]-setpoint;
  if (abs(sum_error + error) <= windup_cap) {
    sum_error += error;
  }
  
  float alpha = 0.5;
  float diff_error = error - prev_error;
  prev_error = error;
  f_diff_error = alpha*diff_error + (1-alpha)*f_diff_error; 
  
  float p_term = KP*error;
  float i_term = KI*sum_error;
  float d_term = KD*f_diff_error;
  int pwm = round(p_term + i_term + d_term);
  writepwm(pwm,pwm);
}
```

The loop relies on a helper function used to write PWM values to the motors:

```cpp
void writepwm(int l,int r){
    float alpha = 0.05;
    if (l >= 0) {
      analogWrite(A15,0);
      analogWrite(A16,rescalepwm(calibratepwm(l, alpha)));
    } else {
      analogWrite(A15,rescalepwm(calibratepwm(-l, alpha)));
      analogWrite(A16,0);
    }
    if (r >= 0) {
      analogWrite(4,0);
      analogWrite(A5,rescalepwm(calibratepwm(r, -alpha)));
    } else {
      analogWrite(4,rescalepwm(calibratepwm(-r, -alpha)));
      analogWrite(A5,0);
    }     
}
```

Furthermore, to ensure that the PWM values would be rescaled outside of the motor's deadband regime, as well as to ensure that the values were calibrated to help to robot move straight, and to clip values outside of the maximum magnitude of 255, the following helper functions were used:

```cpp
int rescalepwm(float pwm){
  float deadband = 45;
  return round((pwm+deadband)*225.0/255.0);
}

float calibratepwm(float pwm, float alpha){
  return clippwm(pwm*(1+alpha));
}

int clippwm(int pwm){
  if (abs(pwm) >= 255) return 255;
  else return pwm;
}
```

To tune the controller, the proportional error gain was first adjusted such that the robot would begin to oscillate around the setpoint of 200 mm. This occured at around a gain of 1.1, as shown in the following:

![P Control](/lab-6-assets/pcontrol.png)

From here it is clear that the robot manages to reach the setpoint quite closely within a few millimeters, but suffers from a significant overshoot. 

We also note that the robot seems to reach the setpoint somewhat slower as it gets closer to the wall. 

To address this issue, an integral term was then added, which could hopefully add an additional kick to the motors towards the end as the sum of the past errors would accrue to a large enough value:

## PI Controller:

In tuning the PI controller, the proportional gain was scaled back by half to 0.055, and the integral gain was then increased unti la loss of stability was achieved:

![PI Control](/lab-6-assets/picontrol.png)

As shown in above, we see that the robot demonstrates significant oscillations around the setpoint, and approaches the setpoint much faster.

Furthermore, we note that although the integral error was carefully clamped to a maximum value to prevent integral windup issues, the integral term was enough to build up momentum such that the robot would severely overshoot the setpoint. This resulted in the robot crashing into the wall rather than settling at the setpoint.

To address this, a derivative term was then added, which could hopefully penalize fast transitions in the error by setting negative gain values. Though this would potentially add instability when left alone, the hope is that this would be cancelled out anyway by the integral term.

## PID Controller:

To tune the fully fledged PID controller, the integral gain was scaled back by half to 0.025, and the derivative gain was set to around 0.1:o

![PID Control](/lab-6-assets/pidcontrol.png)

This procedure was then iterated for each of the steps (setting proportional gain, integral gain, and derivative gain) until a more optimal controller was achieved.

 The resulting gains were then tweaked slightly to achieve better rise times and lower overshoot percentages. 

The final result is shown below:

![PID Control Final](/lab-6-assets/pidcontrolfinal.png)

## Speed:

From the data, we note that the maximum speed of the robot is around 1000 m/s, and that the maximum sampling rate under this PID loop is around 10 samples per second.
