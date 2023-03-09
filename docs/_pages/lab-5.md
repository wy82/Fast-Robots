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

After soldering a motor driver to the Artemis board, the output was first measured via an oscilloscope. To ensure that the motor driver had a reasonable power supply, it was connected to a 3.7 V supply set to a current limit of 1.5 A: 

![Test Setup](/lab-5-assets/Test_Setup.png)

Afterwards, to demonstrate the the motor driver was functional, the following code generated a PWM input:

```cpp
analogWrite(4,200);
analogWrite(A5,0);
```

This then generated the following output:

![Oscilloscope](/lab-5-assets/Oscilloscope2.png)

After the correct outputs from the motor driver were confirmed, the motor drivers were then soldered to the motor leads. To test that the motor could drive in both directions, the following PWM commands were used:

```cpp
  // forward
  analogWrite(4,200);
  analogWrite(A5,0);
  delay(1000);

  // stop
  analogWrite(4,255);
  analogWrite(A5,255);
  delay(1000);

  // backward
  analogWrite(4,0);
  analogWrite(A5,200);
  delay(1000);

  //stop
  analogWrite(4,255);
  analogWrite(A5,255);
  delay(1000);
```

This then resulted in the following reaction:

<iframe width="560" height="315" src="https://www.youtube.com/embed/3VEBldo_y08" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

This process was then repeated for the second motor driver, and once both motor drivers were confirmed to work when connected to their respective motors, the 850 mAh battery was soldered to the drivers. After writing a somewhat similar set of PWM commands compared to before (alternating forward, braking, and backward commands) functionality of the entire control circuit was then tested:

<iframe width="560" height="315" src="https://www.youtube.com/embed/_4UBoPb1LRU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Wiring:

After everything was soldered together, the components were then secured into the car:

![Car](/lab-5-assets/Car.png)

Here, the time-of-flight sensors were place at the front of the car and to the right, which was somewhat different from the initial plan to include them both at the front. The main reason for the difference is that it would be difficult to run the sensor wires around the motor drivers without incurring a significant amount of noise from EMI. 

The motor drivers were then placed near the front, but more to the left side so that the wires would not run too close to the sensors. AS with the other components, wires were twisted to ensure minimal effects from EMI.

The rest of the components, including the IMU, Artemis board, and 650 mAh battery were then housed in the back of the car, which fit tightly enough to secure the IMU so that disturbances from vibrations or noise from the motor drivers would not be an issue. 

## Calibration:

Next, to ensure that the car was capable of moving along the ground, the following program was used to command the robot to drive forward for 3 seconds:

```cpp
while(millis() - prev < 3000) {
  analogWrite(A16,100);
  analogWrite(A15,0);
  analogWrite(4,100);
  analogWrite(A5,0);
}
```

After testing a few PWM values, it was found that the minimum PWM value required for the robot to still move was about .

Finally, to ensure that the robot could move accurately in a straight line given the same PWM values for both motors, the script was run 

After a bit of tweaking, a conversion factor of $$\alpha = $ was found to correct the robot's path into a straigt line:

## Open Loop Control: 


