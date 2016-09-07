/*
Micromill Firmware for Arduino Nano or similar using unipolar steppers for an experimental 3 axis mill
(c)2015-2016 duisenberg 
 
This software is licensed under GNU GPL V3 

0. Abstract
The Micromill firmware is simple gcode interpreter for Arduino, offering basic G-Code capabilites 
and is meant to be easy accessible, even for beginners. 

It supports three 28BYJ-48 steppers, 3 endswitches and 2 relay switches for spindle and Vac control.
Interpretig commands from USB, it supports the most common GCodes and emulates the grbl communication
protocol. So common grbl senders may be used. Universal G-Code Sender from Will Winder is suggested.

1. Quickstart
To quick start and test the firmware, undefine ENDSWITCHES_ENABLED, load it to the Nano and start the
Universal G-Code Sender. In UGS settings set "Single Step Mode", "Enable status polling" and in its GRBL 
firmware settings unset "Whitespace Remover". Set the Com-Port the Nano is on and 115200 Baud. Open the 
connection, use the jog or load a file, visualize and send it, the virtual mills progress and positions 
are monitored in Machine status. For testing the firmwares STEPPER_INTERVAL may be set to 250 to run the
files commands with higher speed. 

To create a g-file, the workspace is 80x100x-20mm. See Appendix A for the supported commands.

Be sure to define ENDSWITCHES_ENABLED and reset STEPPER_INTERVAL to 1250 when connecting to a real mill. 


2. Small steppers Mill construction
The reference mill is made from most recycled parts, using drawer sliders and a proxxon minimot 40 with
stand. It has a build room of 8x10x2cm, a 5095 halfsteps per mm translation for x/y, 165 for 1mm z.  
See Appendix B for further details. 


3. Program / Firmware overview
In program header all machine dependent defines are set. These may need attention, depending on where the steppers 
are connected and what features are used. The main interrupt routine monitors the endswitches, drives the motors
and grabs bytes from serial stream. It is initialized in setup. The loop then controls the HOMING cycle and 
executes the serial commands once such a command is available.

The command execution function parses the command line, executes the parsed G/M-codes and returns sending a result 
string to the serial line that is ok or error. 

For the machine parameters a semipermanent parameter storage in eeprom is not supported yet, neither a storage
of current position or target. This may change in future versions. 

For the timed interrupt TimerOne library is used. The original concept included an I2C OLED display but this had
to go because it needed too much memory and time. 


4. Operation
The default operation is to drive the steppers into HOME positions, using the endswitches as positional reference.
Once a serial connect is detected by something received in serial, the cycle interrupts and the firmware awaits
commands. If HOMING is enabled, this cyle will not interrupt and there is no boot message until homing is done.

After initialization the firmware sends a boot message and waits for a command from serial line. Such a command has
to be terminated by NL or NLCR, the maximum length for a serial command is 80 chars. Grbl priority commands ?!~^X 
may be thrown into the stream anytime, $-commands must be in a single line, terminated by NL.

Commands are answered with "ok" or "error:[errormessage]". The errormessages may be subject of future changes. If
DEBUGs are defined, some commands produce debug output that shows up on the senders console.

By default X and Y axes do work in positive, the Z-axis works in negative space. For Z axis, a coordinate translation
is available.


5. Sending GCode, UGS Setup
GCode has to be sent line by line, the sender has to monitor the stream for "ok" or "error" before sending the next
command. (Sender Settings: Enable Single Step Mode) Also the sender should not strip whitespaces, this feature is 
available but not recommended to use. 

The UGS version used is the classic interface from nightly build of September 04,2016. The "platform" Version should
work too, since better configurable for inidivual firmware needs like customized jog commands it might be the
future version of choice, if the sender systems monitor is wide enough.

Other senders may work as well if they support single line mode.

 

Appendix A: Gode-Command reference 
Appendix B: Reference Mill details
Appendix C: Mill setup and finetuning 


6. DISCLAIMER
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any 
later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.
If not, see <http://www.gnu.org/licenses/>.







  

*/
