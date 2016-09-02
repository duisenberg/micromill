/* *********************************************************************************************************
Appendix A - supported G-Commands

A G-Code line may or may not have line numbers. A G-Code line should have commands and parameters seperated
by whitespaces. This firmware is only able to work on ONE gcode line a time. A successful executed command 
line is answered with "ok". If in parsing or execution an issue comes up, the line is answered with 
"error:[errmessage]". 

One command line may contain one or more G-commands, but only one "move" command per line. If a "setting" Gcode
like G90 is lines up with other commands, this ONLY COUNTS FOR THIS SPECIFIC MOVE/line. If such a command is 
issued seperate, it sets the mode for the whole file. See examples below.

WARNING: This still is a GIGO system - put garbage in, get garbage out. Some errors are catched, some are not. 


Each Gcode-command ends with LF (Linefeed, char 10)

G00 Xx Yy Zz         - max speed goto pos 
G01 Xx Yy Zz         - normal speed goto pos
G02 X Y I J F Z      - clockwise arc at from current point to point X Y around center I J Feedrate F 
G03 X Y I J F Z      - counterclockwise arc  Z is helical Z-axis feed (partially supported, see below) 
                       Feedrate F  ... Use small arc segments for small arcs 
                       
G04 Pxxx|Xxxx        - stop / pause Pxxx millis or Xxxx seconds at current position
G17                  - select XY plane (default, cannot be changed)
G18                  - issues an error
G19                  - issues an error
G20                  - prog coords are inch (only partially supported, obsolete system)
G21                  - prog coords are mm
G28                  - Home pos /0 0 0
G82 XYZ RPLF         - Drill a hole     Z-bottom hole, R-retractpos P-dwelltime F-feed L-repeats
G83 XYZ RPQLF        - Drill with Peck -Z-bottom hole, R-retractpos P-dwelltime Q-incrperpeck  L-num repeats
G90                  - all positions absolute (default) - see below
G90.1                - arc centers (IJ) absolute        - see below
G91                  - all further positions relative   - see below
G91.1                - arc centers (IJ) are relative    - see below
----
G92	                 - Reset 0 0 0 to current pos (may damage machine, enable in defines)
G94 Fxxx             - set Feedrate (will be set as close below the requested feedrate as possible)
G162                 - Max pos (set+move) 
  
supported M-Commands
M00 Px                - halt/pause prog (issues an error to force the sender to pause) 
                        optional: P0 -> recalibration P1 back to normal operation : review
M01                   - halt/pause prog switching off the servos and spindle (issues error like M0)
M02                   - end program (reset lineno, goto 0 0 0, reset settings like G90/91 to defaults)
M03                   - Spindle on clockwise
M04                   - Spindle on clounterclockwise (only with spindle/tool pwm control)
M05                   - Spindle off
M06                   - tool change - table is driven to a pos the toolhead is accesible, see below
M10                   - Vaccuum cleaner (Fan) on
M11                   - Vaccuum cleaner (Fan) off
M17                   - enable all steppers
M18                   - disable all steppers
M37                   - check mode, steppers disabled

M120                  - enable endstop detection (not supported, compile time settable only)
M121                  - disable endstop detection (not supported, compile time settable only)

M134                  - write settings to EEPROM (not supported yet)
M930 - M980           - machine specific M-Codes

M934 Pxxx             - Translate Z coordinates for positive Z generators. Without Poffset set, it will
                        multiply all incoming Z coordinates with -1. See Appendix C.3
M935                  - Reset Z translation

M942                  - Tool auto off feature switch - tool&fan need to be on with M03/M04/M10 once

  
Misc commands
Nxxx                  - number in sequence (mostly ignored)
Fxx.xx                - Feedrate, may be parameter in some commands as well
  
Example commands 
N010 G00 X0 Y0 Z0 F010   - max speed to home pos all axes, F - feedrate in mm/min

G90                      - all following command coordinates are absolute
G91.1 G3 X4 Y4 I1 J1     - do ccw arc from current pos to 4,4 with center offset 1,1 to current pos. 
                           relaive center setting valid only in this line. 
G21 G0 X20 Y20           - go G0 max speed to X 20mm Y20mm (well, this still is default ;)
G91.1                    - all following center coordinates are relative

CAVE:
G90 G21 G17              ->would set units to mm (permanent until G20 is sent) but G90 only for this line.
                         ->BETTER PUT EACH COMMAND IN SEPARATE LINE

GRBL-Commands
All grbl commands consist of one up to 4 alphanumeric chars. The priority commands like ?,!,^X can be sent
at any time, even in a command, all $ commands need to be sent in their own single line..

?                        - status report
$                        - send help message
$$                       - send parameter sheet                - semi faked 
$xxx                     - send parameter no xxx (1-3 digits)  - just sends ok
$C                       - enable/disable check mode (in check mode all commands are executed but 
                           no stepper is moved.)
$H                       - not supported, just send ok
$X                       - interrupt current command - may reset the microprocessor (answeres ok atm)           
-----------------------------------------------------------------------------------------------------
Limits and issues

G-Code interpreter
G-Code is not always G-Code. Professional CAD/CAM systems work with a G-Code postprocessor that translates the
Code the CAD's developer think is G-Code to Code the current machine may be able to work with - after only few 
manual corrections. With CAD one often works in all quadrants of an xyz system, the machine is limited to
only one quadrant. One reason to set a G92 origin somewhere in the middle of the build room - a hard to 
control feature. Or a hard way from a CAD drawing to working G-Code. See M934. 

G90/91 handling
Cürrently G90/91 commands sent in a line with a move command are interpreted as valid only for their single line.
This feature may change in future as the G-standard seems to suppose to switch things with each setting command 
"per program" until another command resets the feature. 

G2/G3 issues
G2 and G3 commands are meant to draw arcs and circles. Some programs uses them for that, some are able to 
replace them with lines. To support helical movement - move in circles with xy and feed z in that motion - 
the Z parameter is supported. G2/G3 only works for arcs up to 180 deg, Z-1.8 should drive the Z axis for
0.01mm per degree, but the Z axis's resolution is 0.06mm per step ...
Havin relatively big arcs close to 180 deg, in some cases it may happen that start and endpoint overshoot
for some degree. It may be an issue with the atan2 routine, it may come from using steps instead of mm. 

Tool change feature
One awkward thing for the Mill is to change tools. The machine holder at its stand has to be turned, the 
Z-offset is gone and the toolhead needs to be repositioned. One way is to split the G-Code file according
to the tool changes, readjust the micromot after toolchange and then load n run the according file.
But if tools are prepared carefully before to have same height and insertion - maybe using a piece of scotch
tape - the table can be driven into a position far beyond MAX X. Thus the toolhead becomes accessible, the 
next tool can be mounted - with care but without any change to the Z-axis. The table is driven back to its
prior position then and the job can be resumed.
The tool change sequence currently is 
M00
M06
M06
Each command has to be in a single line. If the ugs issue - at error send next line and then stop - is solved, 
two M6 should work as expected.

Feedrate issue
The steppers speed is controlled by the stepper delays, so it can be set only in discrete portions. The lower a
feedrate is, the more it can be close to the requested value. In effect, the next settable xy feedrate after
max 9.42 is 4.71 and then 3.14 and so on. 
As it reads, the feedrate is calculated from all movement axes. So if there is a combined movement of xy or xyz
all feedrates count together as root of their spare sums. This is currently not calculated.

* *************************************************************************************************************/
