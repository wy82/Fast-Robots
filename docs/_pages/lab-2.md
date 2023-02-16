---
permalink: /lab-2/
title: "Lab 2"
---
This lab was dedicated to establishing a wireless connection from my personal laptop to the Artemis Nano via the Bluetooth Low Energy (BLE) protocol. The main objective in doing so is to setup an efficient way to monitor and debug the sensors and actuators of the robot, and to allow for computational offloading of control actions. 

## Setup:
To start off, I created a Python virtual environment dedicated to installing and running python packages related to the BLE protocol:
'''python
pip install numpy pyyaml colorama nest_asyncio bleak jupyterlab
'''
After setting up this on my laptop, I then used the ArduinoBLE library to establish a simple connection to the Artemis Nano that recorded the MAC address of the board:
![MAC Address](_images/MAC_Address.png)

Combined with a newly generated UUID, I recorded this information on both my personal computer and the Artemis Nano so that both devices would be guaranteed to establish a shared connection.

The connection was then tested via importing the ble package in a Jupyter notebook, followed by simply instantiating a controller object and connecting: 
'''python
%load_ext autoreload
%autoreload 2

from ble import get_ble_controller
from base_ble import LOG
from cmd_types import CMD
import time
import numpy as np

LOG.propagate = False
'''

'''python
# Get ArtemisBLEController object
ble = get_ble_controller()

# Connect to the Artemis Device
ble.connect()
'''
![Connection](_images/connection.png)

## Codebase:
In short, the Bluetooth protocol mainly consists of peripheral devices such as the Artemis Nano that advertise services, each of which contain various characteristics that hold 512 byte values. To identify a particular service or characteristic, the protocol uses binary numbers known as a Universal Unique Identifier (UUID). Hence, to establish the connection between the laptop and Artemis Nano, it was crucial to first ensure that both devices agreed on the same UUIDs and addresses.

The code necessary to establish the Bluetooth connection contained quite a lot of different parts, both for the laptop and on the Artemis Nano. The laptop files included a connections.yaml file for recording the MAC address and a service UUID to connect to the Artemis Nano with, as well as characteristic UUIDs for receiving and transmitting data. A second file, cmd_types.py, contained a Python enum datastructure for storing specific Bluetooth command names listed by number. 

Besides these simpler accessory files, the python code responsible for doing the heavy lifting was left to ble_base.py and ble.py, which are modules that setup the groundwork to build a user-friendly BLEController class that can easily connect/disconnect and send/receive data. Most importantly, the class also offers the freedom to write notification handlers that will be called whenever the computer receives data through a particular characteristic UUID, which helps facilitate asynchronous data collection. Finally, a couple other printing and logging features are left in a utils.py script to better record and organize events that occur while using the Bluetooth connection.

A somewhat similar code setup is also burned onto the Artemis Nano, which consists of a ble_arduino.ino file recording the UUIDs and enumerated command types, as well as a command handler executed with a specific response to each command. This script is also used to advertise the Bluetooth service, setup the relevant characteristics, and read out the MAC address of the Artemis Nano. A few other header files including BLECStringCharacteristic.h, EString.h, and RobotCommand.h define classes that are used to handle the characteristic data in cases that are not covered by the ArduinoBLE library. In particular, the BLECStringCharacterstic.h and RobotCommand files are dedicated specifically for formatting and parsing string characterstics and robot commands, whereas EString.h mainly just holds a bunch of helper functions to make string manipulation (technically on character arrays) a bit easier. 

## Configurations:
As mentioned before, the first step in establishing the Bluetooth connection was to ensure that the configurations were consistent across both devices. This included the UUIDs and command types:
![Python UUID](_images/Python_UUID.png)
![Arduino UUID](_images/Arduino_UUID.png)
![Connection](_images/Python_cmd_types.png)
![Connection](_images/Arduino_cmd_types.png)
Admittedly, the list of command types was a bit longer than necessary, which was mainly to prepare for the rest of the lab and future labs.

## Demo:
The next step was to run a simple demo of basic Bluetooth commands, which mainly consisted of sending and receiving strings, integers, and floating point values::
![Python Demo](_images/Python_Demo.png)
![Arduino Demo](_images/Arduino_Demo.png)

## Echo Command:
The first lab task was to send an ECHO command, which basically involved modifying the string characteristic value on the side of the Artemis Nano and then simply returning the modified string back to the computer:
'''cpp
case ECHO:
    char char_arr[MAX_MSG_SIZE];

    // Extract the next value from the command string as a character array
    success = robot_cmd.get_next_value(char_arr);
    if (!success)
        return;

    // Write augmented string back
    tx_estring_value.clear();
    tx_estring_value.append("Robot says -> ");
    tx_estring_value.append(char_arr);
    tx_estring_value.append(" :)");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
    break;
'''
![Echo](_images/ECHO.png)

## Get Time Command:
The next command was GET_TIME_MILLIS, which simply returned a timestamp at the moment of receiving the command:
'''cpp
case GET_TIME_MILLIS:
    // Write string back
    tx_estring_value.clear();
    tx_estring_value.append("T:");
    tx_estring_value.append(int(millis()));
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
    break;
'''
![Get Time](_images/GET_TIME_MILLIS.png)

## Setting up a Notification Handler:
To make things more interesting, a simple notification handler function was added, which just extracted the time in milliseconds out of the string:
'''python
async def get_time_millis(uuid,byte_array):
    global time_millis
    time_millis = [];
    time_str = ble.bytearray_to_string(byte_array)
    time_millis = int(time_str[2::])
'''
![Notification Handler](_images/Notif_Handler.png)

## Get Temperature Command:

'''python
async def get_temp_5s(uuid,byte_array):
    global time_millis
    global temp_celsius
    time_millis = [];
    temp_celsius = [];
    
    temp5s_str = ble.bytearray_to_string(byte_array)
    temp5s_list = temp5s_str.split("|")
    for i in range(0,len(temp5s_list),2):
        time = temp5s_list[i]
        temp = temp5s_list[i+1]
        time_millis.append(time[2::])
        temp_celsius.append(temp[2::])
'''
'''python
async def get_temp_5s_rapid(uuid,byte_array):
    global time_millis
    global temp_celsius
    
    temp5s_str = ble.bytearray_to_string(byte_array)
    temp5s_list = temp5s_str.split("|")
    for i in range(0,len(temp5s_list),2):
        time = temp5s_list[i]
        temp = temp5s_list[i+1]
        time_millis.append(time[2::])
        temp_celsius.append(temp[2::])
'''


## Limitation:

