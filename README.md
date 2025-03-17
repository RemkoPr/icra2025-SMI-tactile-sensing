# ICRA2025: Self-Mixing Laser Interferometry for Robotic Tactile Sensing

[[Paper](https://arxiv.org/abs/2502.15390)] The source files in this repo are provided as-is, additional support for recreating the design can be requested, see contact information below.

# Repo structure

`code` contains readout code for the fingertips. In the paper, the SMI fingertip was read by an Arduino UNO, the microphone was read by an Arduino MKR 1000 WiFi.

`data` contains the data files used in the paper.

`mechanical-design` contains SolidWorks design files for the structural parts of the fingertips.

`pcb-design` contains KiCad design files for the PCBs.

# User Guide

<img align="right" width="250" height="250" src="https://github.com/RemkoPr/icra2025-SMI-tactile-sensing/blob/main/img/mold.png">

The info in this paragraph is relevant for both the microphone and the laser.
Order the PCBs (`pcb-design > microphone` and `pcb-design > laser`) from your favorite manufacturer, refer to the BOM in the main directory of this repo for required parts.
Print the .stl files in `mechanical-design`.
The process of moulding the silicone contact surface is illustrated by the graphic on the right. To keep the "sensor cavity insert" in the right position, fix it with hot glue. It is recommended to attach the "mounting interface" to the fingertip while curing, so that the silicone has less chance to leak.

<BR CLEAR="all">

## Microphone specifics

<img align="left" width="200" height="281" src="https://github.com/RemkoPr/icra2025-SMI-tactile-sensing/blob/main/img/mic_electrical.png">

On the PCB, you must set the microphone to either by left or right channel, this will determine during which phase of the I2S communication cycle the microphone sends its data. Do this by soldering a bridge as indicated on the image (the readout code in this repo was tested for left channel, so with a bridge soldered from the middle pad to the L pad).

The MKR1000 firmware (`code > arduino > microphone`) was used for the paper, the wiring is explained in the top rows of the `I2S_MKR1000.ino` file. The `I2S_ArduinoNanoBLE_16kHz` is a first attempt at a readout implementation for the Arduino Nano 33 BLE, for which the standard I2S library isn't implemented.

<BR CLEAR="all">

## Laser specifics

The readout frequency when read with an Arduino UNO was 3.7kHz, and 18kHz with our [Halberd coupling](https://github.com/RemkoPr/airo-halberd/tree/main) (or equivalently with an Arduino Nano 33 BLE, same microcontroller unit). When using higher readout frequency, turn off the 2kHz anti-aliasing filter with the on-PCB slide switch.


# Contact

For support regarding the design files in this repo, raise an issue or contact me at remko.proesmans@ugent.be.
