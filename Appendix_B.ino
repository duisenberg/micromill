/* ********************************************************************************************************
Appendix B - Small steppers Mill construction


1 Mechanical construction part
The "reference" mill for the firmware consists of 2 steppers using M5 threaded rods and 19cm drawer 
sliders to operate the XY table, mounted on a 40x40cm wooden shelf. The Z-axis is a Proxxon Minimot 40
Stand with a winded belt coupling, allowing like an 8x12x2cm work area. All in all it was made from
2 20x20cm plywood 4 and 6mm, a meter wooden bar 10x20mm, a M5 threaded rod. A bit of Pattex power 
plasticine (modeling "metal" clay) and M5 females to connect stepper to M5 rod and for the other end
of the rod two ball bearings with 5mm inner diameter from old (very old) harddrives were used. Along
with some small and very small savaged screws. Some additional plywood for the stepper mounts and an
id neck strap for the proxxon stand gear - thats the mechanics part. 3 steppers, 3 microswitches, 
3 led's, a power supply, some high capacity condensators and the hardware is done.

With an XY incline of 0.8mm per turn (M5) and the gear ratio of 63.8:1 a mm needs 5095 steps, per step it
has an advance like 0.0002 mm. This is far beyond the tolerances of the rest of the machine which is fairly
like 0.1 millimeter.

The Z-axis is coupled by the wind up belt with about 12:1 coupling factor to the gear/feather driven stand 
meachnics that operates with about 4cm per turn. So 9 degrees is like 1mm, resulting into 108 degrees for the
stepper, which is like 150 steps. Since the belt coupling wasnt manufatured exactly, the estimated steps per
mm is 165. While winding up, the coupling factor for the second turn decreases to little less steps per mm. 
This currently isnt taken into account by the program. 

Further a control for turning the Micromot on/off is supported, as well as an on/off control for a fan or a
vacuum cleaner. 

The power supply is an old Notebook power supply that supports 12V and 5V 2A, an old desktop supply will do
as well. The proxxon drill needs 15V for max speed, but the mill is slow and needs like an hour for a 2x6cm
engraved name, so 12V for the proxxon is quite ok. A tiny bit of hot glue in the proxxons motor and bearing
fits helps to get rid of its tool height slip.


2 Electronics
The stepper drivers have 3 connectors, 4 pin control, 2 pin power and 2 pin enable. The enable in general is 
shorted by a pinheader. For the overall current needs small lines for power and control are good, the motors 
take like 100mA per control line "on". Since these are motors using a winding, it seems to be a good idea to
place a high capacity condensator like 3000uF or even higher close to the drivers power pins to prevent a 
Voltage break in when the coils are switched. 

The steppers are also available as 12V type, with 200ohm windings they need less amps, so input watts is quite
the same and torque should not differ that much. Power supply is a bit easier since a single 12V supply covers 
everything. 

The endswitches closers are connected to ground an led (with build in resistor) from  +5V----|>-+-./ ----- GND,
the according pin is connected between led and switch.                            Ax------------+ 
The emergency switch is connected the same way. 

This is in main the electronics needed. For tool and fan control a common relay board is used. It switches on HIGH
signal. This can be changed in setup, toolControl and fanControl if necessary.




*/
