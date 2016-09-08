/*
Appendix C
Mill operation

1. Understanding G-Code
There is a standard - DIN66025/ISO6983 - for G-Code, it looks quite simple and handy. As often, in common this
is true, but the standard leaves plenty room, the real world uses it and the devil is in details.

Never use any G-Code file without proper testing. Most automatically created files need manual care before they
can be sent to the machine. Be sure to understand at least the G-Code basics, else the learning curve (like mine)
will be exponential but repair intensive.

Since the mill is very slow, it will easily take some hours for even rather simple objects. The longer a job takes, 
the more likely something is to come in between. To be able to stop a job and continue it somewhat later, the code
producer should use absolute coordinates. This also makes it easy to split the job in advance. Keep an eye on the 
tools motor temperatur, maybe use the TOOL_AUTO_OFF feature to let it cool while moving in G0. 


2. Setup + Finetune Settings
First important point is to make sure that all axes do move in the correct directions. Turning the ULN connectors
is the easiest way here, or set stepper_pins appropriate. Put the axes manually somewhere in middle position before
switching on the power. If endswitches are enabled and homing set true, all axes shall move towards the switches.

Once the table moves correctly, the endswitch settings must be tuned. In normal operation, homing cycle drives the
axes into negative space until the endswitches go high. This is the MIN position. Depending on the switch used, it 
should be 1-3 mm less than the 0 position to make sure the switch is released a bit before 0. 

The third point is experimental and may take some time. Depending on material and tool, the path depth and move
speed has to be be determined. The most important point here is to prevent slip, which is hard to detect. One 
way is to run a gcode test file several times, increasing the feedrate a bit each time. As long as the xy 0 pos
the machine goes to at jobs end is exactly the same it was in the beginning things look good but it is hard to 
detect offets below 0.2mm.

Depending on the material and the tool there will also be aweful vibration and resonance issues. A slim rubber 
pad between table and workpiece may help. 

As rule of thumb a 1mm diameter tool at 10k rpm will have a cut speed of ~32m/min. For a cut depth of ~2mm 
generic cut speeds are 40-120 for steel, 100-500 for aluminium, 100-200 for copper, 50-150 for plastic and
up to 3000 for wood. 
For the max feedrate cut speed, numbers of tools cutting edges, tool rpm and a material specific value are
multiplied. With the given tool at 10krpm there are calculated speeds like 4mm/min for plastic, acryl etc,
1mm/min for light metal and less than 1mm/min for steel. Calculators are on the net. 

In firmware there are two ways to set the feedrate. Using a separate F comman or G94 F, the default feedrate
is set. G0 and G28 always use the max feedrate. All other commands use the default feedrate or the feedrate
given in the command itself. The drill commands have a separate drill feedrate that isnt affected by default
feedrate. $G displays the current work rate. See Appendix A Feedrate issues.

3. G-Code Applications
For first start, F-Engrave is not a bad place for a kickoff. Simply enter the mills limits, be sure to 
click Z working in negative, set arc none in settings and then export your word ir picture. In the next
step it is suggested to run a G-Code simulator like CAMotics. If this shows the path as expected, it 
should be ready for production. Double check the feedrate, then maybe it is time to give the mill a try.
Using check mode ($C) helps to detect errors, maybe use a separate Nano with homing and endswitch disabled
and a 250 timer to see if the file runs correct in senders visualizer. Also a pen attached to the tool may
help to see how things will run.

Having more sophisticated constructions in mind, a quite poerful way to go is FreeCAD. It still is in early
state of development, expect many things work only in their "specific" way, often enough a part file needs 
to be redone from scratch because things went haywired. Especially in Paths workbench. Once the Machine and 
Tool is set up, be sure to click the whole part in project tree before marking a surface and creating a path
object. From Versions 0.14 seems to create paths ok, 0.15 and 0.16 bummer, 0.17 looks like Paths is working
again. Starting with a very simple object like a block with a pocket is strongly suggested. 

As from Freecad - other CAD's may be same - Parts are sometimes created using all coordinate system quadrants.
If defining coordinate axes as part of the construction - some tutorials suggest to do so - it might come to
issues later in path, if the object has to be moved to fit into the build coordinates of the mill. 

In common construction the object is built in positive Z quadrant. To ease things here, M934 can be used to
set a Z coords translation to negative. This way the produced gcode can be used directly. Use M935 to disable
the translation (default setting). If this feature shall be used, in Freecad Machine settings set Z MAX 20mm 
MIN 0mm and place a M934 P-20 into the gcode header.  
A micromill post processor script is available. 



*/
