---
permalink: /lab-1/
title: "Lab 1: The Artemis Board"
sidebar:
  nav: "lab-1"
---
This lab was basically just an introduction to the Artemis Nano and the Arduino IDE. I mainly just ran example programs from the Arduino IDE to test some basic functionality.

## Example 1:
The first example was the Blink program which just told the Artemis Nano to blink the LED pin. Since the program set the pin number to 13 by default, I had to change this to LED_BUILTIN to make sure the right pin was being used.
<iframe width="560" height="315" src="https://www.youtube.com/embed/3vamad-_anY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Example 2:
The next example was Example04_Serial, which printed to the Serial Monitor at a baud rate of 115200. As shown in the video, any character I wrote to the Serial Monitor was then echoed back.
<iframe width="560" height="315" src="https://www.youtube.com/embed/jVTM1ANTWWs" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Example 3:
This example was Example02_AnalogRead, which tested the temperature sensor on the Artemis Nano. After breathing on the sensor for a bit, the temperature value rose from around 32500 to around 32900.
<iframe width="560" height="315" src="https://www.youtube.com/embed/gTJFwGr1jW0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Example 4:
The last example was Example01_MicrophoneOutput. This example uses Pulse Density Modulation (PDM), a form of analog-to-digital conversion, to record digital audio samples. 

Loosely speaking, PDM basically encodes the amplitude of an analog signal via the density of 1s and 0s via sigma-delta modulation. The main advantages of this are lower quantization noise and bits per sample, although it requires a higher sampling rate (around 6 MHz) to achieve this. The samples are then digitally converted to PCM via low-pass filtering and decimating the signal, where each sample contains a binary number than corresponds to the amplitude level. The PCM data can finally be converted to the frequency domain using the Fast Fourier Transform (FFT). 

For more information, see this [link](https://tomverbeure.github.io/2020/10/04/PDM-Microphones-and-Sigma-Delta-Conversion.html), which gives an in-depth discussion. For this particular script, the frequency bin with the largest magnitude is then printed to the Serial Monitor. To test this, I tried saying "hello" in the video, where nonzero frequency values are printed whenever sound is detected.
<iframe width="560" height="315" src="https://www.youtube.com/embed/8_SpiQbmx8A" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
