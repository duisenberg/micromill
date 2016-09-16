/*************************************************************************************************************
 *** Micromill for Arduino Nano
 *** (c)2016 diusenberg  
 *** 
 *** This software is licensed under GNU GPL V3
 ***
 *** XYZ-Axis CNC / Engraver firmware using 28BYJ steppers with ULN2003 driver
 ***
 *** G-Code implementation see Appendix A. Supported sender is Universal G-Code Sender from Will Winder.
 *** 
 *** Version history
 *** 0.1 - manual control,oled display
 *** 0.2 - serial control, first gcode parser
 *** 0.3 - some G/M commands, lines, manual control+oled obsolete
 *** 0.4 - basic G/M commands, grbl communication protocol emulation
 *** 0.5 - G2/G3 arcs, G20,G21,G90,G91,M934,M6, homing
 *** 0.6 - M3/4/5/10/11/942,G81,82,83,basic feedrate, halt switch, eeprom
 ***
 *** Planned:
 *** 0.7 - lines/arcs with Z parameter , feedrate review
 *** 0.8 - code optimization/cleanup
 *** 0.9 - reviews, testing
 *** 
 ***
 *** TODO: 
 *** - review: the multiple Gcodes per line feature 
 *** - review: feedrate setting per command line
 *** - review: commands without whitespace
 *** 
 *************************************************************************************************************/
#include <TimerOne.h>
#include <EEPROM.h>
#include <math.h>
#include <avr/pgmspace.h>

#define VERSION "0.6a" 
#define FALSE  B0
#define TRUE   B1
#define OFF    B0
#define ON     B1

//* ************************************
//* General feature settings
//* ************************************
//
#define DEBUG //common debug output on serial ´
//individual debugs
//#define LINE_DEBUG //debug output for lines
//#define ARC_DEBUG   //debug output for Arcs
//#define ARC_DEBUG_MAX //more verbose arc debug


//obsolete: does not fit into flash any more 
//#define GRBL_FAKE_FULL //send complete GRBL params message (some senders expect that)
                        //if undefined, $# answers only for some parameters too

//#define ENDSWITCHES_ENABLED  //undefine for testing

//#define IDLE_STEPPER_POWERSAVE // define to switch off power if choosen pos is reached
                               // this saves like 150mA and keeps stepper cool but reduces
                               // the hold force to gear friction and gives a little less
                               // force on next step so slip may occur easier

//FORCED HOMING - set to TRUE the firmware runs the axes to their endswitches
//                before sending the boot message and accepting commands from serial

#define HOMING FALSE // set TRUE to force initial homing


#define EMERGENCY_SWITCH 42 //define how to use it 0  - just hold things until pressed again
                           //                       1  - like 0 but with alarm sound/led
                           //                       2  - reboot (doing nothing at restart - not supported yet)
                           //                       42 - activate PIN_MISC only
                           
//if defined, the value read from S command * 0.0512 is written to 
//PWM_OUT (A6 by default) to set spindle speed from 10000ccw to 10000cw. 
//Spindle on/off is controlled by relais still. 
#define PWM_SPEED_CONTROL

//* *************************************************
//* Machine specific parameters, construction related
//* *************************************************

#define STEPPER_INTERVAL 250 //1250us for timer intr, 1250us 800Hz -> 100Hz/turn max speed 
                             //may be set down to 250 for testing reasons w/o motors

//max steps per sec = 1/interval 
#define STEPS_PER_SECOND 800 //set manually to enable real FR sim even if interval is set lower for testing

//Stepper pins 
byte stepperX_pins[4]={2,3,4,5};  //maps to ULN 1-4 
byte stepperY_pins[4]={6,7,8,9};  //maps to ULN 4-1
byte stepperZ_pins[4]={10,11,12,13}; //maps to ULN1-4  


#define ENDSWITCH_X     A1  
#define ENDSWITCH_Y     A3
#define ENDSWITCH_Z     A0
  
#define PIN_MISC        A2  //used for a signal led or something like that

//analog only pins
#define PWM_OUT         A6   //PWM output

#define EM_SWITCH       A7   //emergency switch
  
//Tool and fan control using one of those relay boards...these
//are active high, so need to go low to switch something on. 
#define TOOL_CONTROL    A4
#define FAN_CONTROL     A5

//not yet supported  
#define TOOL_CONTROL_PWM 0
#define FAN_CONTROL_PWM  0

#define SERIAL_SPEED 115200 

//see Appendix B, Mill construction - used by G21
#define X_STEPS_permm  5095 
#define Y_STEPS_permm  5095
#define Z_STEPS_permm  165

//outdated inches, still used, G20
#define X_STEPS_perInch 129413
#define Y_STEPS_perInch 129413
#define Z_STEPS_perInch   4191

//build room 8x10x-2cm
//#define X_MAX_POS 407600 //80mm
//#define Y_MAX_POS 509500 //100mm
#define X_MAX_POS 510000 //110mm
#define Y_MAX_POS 610000 //120mm
#define Z_MIN_POS -3300  //20mm (!Z inverted)

//this is where the endswitch is located
#define X_MIN_POS -10000 //2mm
#define Y_MIN_POS -10000 //2mm
#define Z_MAX_POS 200    //1.5mm 

#define X_TOOLCHANGE_POS 600000 //120 mm

//granularity for lines and arcs
#define STEPSIZE 100 //~0.02 mm per cycle  //review
  

//if there is slip (missing steps) with G0 commands, make sure there is no 
//mechnical issue like too much friction. If everything looks easy and there
//still is slip increase min_delay by one. 
#define STEPPERX_MIN_DELAY 0
#define STEPPERY_MIN_DELAY 0
#define STEPPERZ_MIN_DELAY 9

//max delays - see Appendix C for considerations
//Z delay 200 means 41 sec per 1mm ~0.024mm/sec FR 1.4
//XY delay 40 means 254 sec per 1mm ~0.004 mm/sec, FR 0.235
//so a combined XY MIN FR would be 0.33 
#define STEPPERX_MAX_DELAY 40
#define STEPPERY_MAX_DELAY 40
#define STEPPERZ_MAX_DELAY 200

//default drill delay / speed 0.06mm/sec FR ~3.6
#define Z_DRILL_SPEED 100

//feedrate per axis in mm/min 
//one step is 1250us which is 800 steps/sec or 48k/min
//resulting in a feedrate of 9.42 mm/min at max speed. 
#define MAX_X_FEEDRATE 9.42  //these are the mechanical
#define MAX_Y_FEEDRATE 9.42  //limits due to the mills
#define MAX_Z_FEEDRATE 29.09 //construction 
 
//table max feedrate in mm/min. 
#define MAX_XY_FEEDRATE 13.32 


//G92 modifies the 0 0 0 position setting
#define G92enable FALSE //setting this TRUE may save time in some cases but bears
                        //risk to damage the machine. If it drives into endswitches, 
                        //they fire and reset pos 0, if it drives above the max pos
                        //the tool is in air then.
                       
                      

/*********************************************************************************
 *** No general user definable settings below
 *********************************************************************************/

volatile byte buf[80];
volatile byte ptr=0;
volatile int maxval;

//misc globals, manual control, vcc monitor etc.
unsigned long time_main,time_disposition,time_display;


  
//stepper sequence cw / ccw  halfstep mode 
byte stepper_sequence_clw[8] = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};
byte stepper_sequence_ccw[8] = {B00001, B00011, B00010, B00110, B00100, B01100, B01000, B01001};
byte stepperX_spos=0,stepperY_spos=0,stepperZ_spos=0,steps_X_step=0,steps_Y_step=0,steps_Z_step=0;
long current_steps_X_pos = 0,current_steps_Z_pos=0;
boolean tool_status=0,fan_status=0;


boolean stepperX_enabled=0;
boolean stepperX_endswitch=0;
boolean stepperY_enabled=0;
boolean stepperY_endswitch=0;
boolean stepperZ_enabled=0;
boolean stepperZ_endswitch=0;
  
boolean steppers_enable =TRUE; 
boolean machine_idle   = TRUE; 
 
volatile long int steps_X_goto = 0;
volatile long int steps_Y_goto = 0;
volatile long int steps_Z_goto = 0;
volatile long int steps_X_pos = 0;
volatile long int steps_Y_pos = 0;
volatile long int steps_Z_pos = 0;
  
#define CONTROL_SERIAL 0
#define CONTROL_INTERNAL 1
byte control_mode = CONTROL_INTERNAL; 

#define INCH 0
#define MM   1
unsigned int steps_X_perunit = X_STEPS_permm; 
unsigned int steps_Y_perunit = Y_STEPS_permm;
unsigned int steps_Z_perunit = Z_STEPS_permm; 
  
//review - obsolete - relace by #define
unsigned long X_max_steps = X_MAX_POS; //80mm 
unsigned long Y_max_steps = Y_MAX_POS; //100mm
unsigned long Z_max_steps = Z_MAX_POS;   //20m

//delay is is set by each move command.   
byte stepperX_delay = STEPPERX_MIN_DELAY; 
byte stepperY_delay = STEPPERY_MIN_DELAY;
byte stepperZ_delay = STEPPERZ_MIN_DELAY;
  
//the steppers max speed is fixed to steppers min delay 
//since it costs 3 bytes it stays for better reading
byte X_max_speed = STEPPERX_MIN_DELAY;
byte Y_max_speed = STEPPERY_MIN_DELAY;
byte Z_max_speed = STEPPERZ_MIN_DELAY;

//default work speeds for G1,2,3 
//if F param is given in a G, this overrides only for current line
//if F is given with XYZ coords it affects all three speeds, if only
//Z is given only Z is affected.  - review: feedrates from producers 
byte X_work_speed = stepperX_delay + 3; 
byte Y_work_speed = stepperY_delay + 3;  // 0.05mm/sec FR 3.1
byte Z_work_speed = stepperZ_delay + 11; //~0.25mm/sec FR 15
  
float feedrate = MAX_X_FEEDRATE; //obsolete, stats only
  
//G-Code defaults (set by single G90/91 or G90.1/91.1)
//be aware that each jobs header should set these! 
boolean dimension_absolute = TRUE;   //default G90/91
boolean cp_absolute = TRUE;          //default G90.1/91.1
  
//Units setting
boolean unit_is_mm = TRUE; //the default unit is mm
  
volatile unsigned int machine_init_delay = 0;
double  translate_Z_offset=0; //with translation, add/sub the offset from 
                            //the incoming Z coordinate to get it into bounds
boolean toolchange_procedure = FALSE;
boolean tool_motor = OFF;
boolean fan_motor = OFF;
boolean tool_auto_off = FALSE;

  //misc
#define WPOS_X  steps_X_goto/(float)steps_X_perunit
#define WPOS_Y  steps_Y_goto/(float)steps_Y_perunit
#define WPOS_Z  steps_Z_goto/(float)steps_Z_perunit
 
#define MPOS_X steps_X_pos/(float)steps_X_perunit
#define MPOS_Y steps_Y_pos/(float)steps_Y_perunit
#define MPOS_Z steps_Z_pos/(float)steps_Z_perunit

#define EMERGENCY_HOMING_SIGNAL 252

boolean homing_done = !HOMING;
boolean emergency_hold = FALSE;

int command_last_line = 0;
char enable_string[8] = {"enabled"};
char disable_string[12] ={"disabled"};

//communications 
char serial_command[80]; //serial command string
volatile boolean serial_command_available;
volatile boolean ready_for_serial;
volatile boolean serial_enabled = TRUE;
volatile boolean status_request = FALSE;
  
void setup(void) {
      
  analogReference(DEFAULT);

//PINS SETUP
  pinMode(ENDSWITCH_Z,INPUT); //A0 ENDSWITCH_Z
  pinMode(ENDSWITCH_X,INPUT); //A1 ENDSWITCH_X
  pinMode(ENDSWITCH_Y,INPUT); //A3 ENDSWITCH_Y
    

  pinMode(TOOL_CONTROL,OUTPUT);  //A4  - TOOL_CONTROL
  pinMode(FAN_CONTROL,OUTPUT);   //A5  - FAN_CONTROL

  digitalWrite(TOOL_CONTROL,HIGH); //TOOL_CONTROL OFF
  digitalWrite(FAN_CONTROL,HIGH);  //FAN_CONTROL OFF
  
  pinMode(PWM_OUT,OUTPUT);          
  pinMode(EM_SWITCH,INPUT);        
  pinMode(PIN_MISC,OUTPUT);   
 
  
  //set the steppers pins
  for(int i=0;i<4;i++) pinMode(stepperX_pins[i],OUTPUT);
  for(int i=0;i<4;i++) pinMode(stepperY_pins[i],OUTPUT);
  for(int i=0;i<4;i++) pinMode(stepperZ_pins[i],OUTPUT);
  
  //set all positions and gotos to 0 - review: em switch
  stepperX_spos=0; stepperY_spos=0; stepperZ_spos=0;
  steps_X_step=0;  steps_Y_step=0;  steps_Z_step=0;
  
  Serial.begin(SERIAL_SPEED);
  
  //without endswitches homing is useless
  #ifndef ENDSWITCHES_ENABLED
  homing_done = TRUE;
  #endif
  
 #ifdef EMERGENCY_SWITCH
   byte restart_from_emergency = EEPROM.read(42);
   if(EMERGENCY_SWITCH==3 && restart_from_emergency == EMERGENCY_HOMING_SIGNAL){
      homing_done = TRUE; //we suppress homing cycle - review: saving last pos in eeprom
      EEPROM.write(42,0); //unset the homing suppresion
      } 
 #endif
 
  delay(1000); //burn a second to honor the lost and gone ones

  //former display cycle, now this is used for pausing  
  time_display = millis();
  
  Timer1.initialize(STEPPER_INTERVAL) ; //1250us 800Hz -> 100Hz/stepper wheel turn 1:63.8 turns from gear
                                        //makes with 0.8mm incline 5095 steps per mm
  Timer1.attachInterrupt(core);
  sei(); //just in case - enable interrupts
 
} //setup done

//the interrupt routine 
//thinge done here should be as short as possible. Currently the 
//core routines take like ~200us so interval should not be less 
//than 250. 

void core(void){
    doSerialComm();
    doSteps();
    //if(status_request) sendStatus();    
    //review: sending status from doDisplay may help
    //        to avoid fuzzy breaks in steppers move 
}
 
 //the main loop 
 void loop(void) {

#ifdef EMERGENCY_SWITCH
       emergencySwitch();
#endif



#ifdef IDLE_STEPPER_POWERSAVE
       steppersPowersave();
#endif  
       
//homing is first - wait until steppers have reached endswitches
if(control_mode == CONTROL_INTERNAL){ 

   // Z works in negative space only
   if(stepperZ_enabled==FALSE ) steps_Z_goto++; //slowly drive to endswitch pos
  
   if(stepperX_enabled==FALSE) steps_X_goto=steps_X_goto -10; //drive to endswitch pos
        
   if(stepperY_enabled==FALSE) steps_Y_goto = steps_Y_goto -10; //drive to endswitch pos
 
 
//Rearm endswitches - not time critical. There always is a gap between on / off due to switch mechanics.
//the endswitch "pressed" means we reached MIN (Z MAX) position. Do not go beyond, machine must hard stop here
//which is done in doSteps()

//here we "rearm" the endswitches and go to 0 position if the current position is lower than 0. 
//so this should be the "true" 0 position, but better 0 is a little more of so the swithes are 
//positively rearmed at 0 position.  Set X_MIN/Y_MIN/Z_MAX appropriate.

   if(stepperZ_endswitch==TRUE){
         if(digitalRead(ENDSWITCH_Z)==HIGH) stepperZ_endswitch=FALSE; // normal operation
         if(steps_Z_goto > 0) steps_Z_goto = 0; //go to 0 position
         } 
  //the x switch may need a replacement
  if(stepperX_endswitch==TRUE){
         if(digitalRead(ENDSWITCH_X)==HIGH) stepperX_endswitch=FALSE; // normal operation
         if(steps_X_goto < 0 ) steps_X_goto = 0;
         } 
  if(stepperY_endswitch==TRUE){
         if(digitalRead(ENDSWITCH_Y)==HIGH) stepperY_endswitch=FALSE; // normal operation
         if(steps_Y_goto < 0) steps_Y_goto = 0;
         } 
 //Control mode INTERNAL done
 } else if(serial_command_available == 1) executeSerialCommand();

 doDisplay(); //some millis delay
 
if(!homing_done)
  if(steps_X_goto == 0 && steps_X_pos==0 
     && steps_Y_goto==0 && steps_Y_pos==0 
     && steps_Z_goto==0 && steps_Z_pos ==0){ homing_done = TRUE; }

 
                                   
} //main loop done

//****************************************************************************************************************
//**

 void doDisplay() {

 if(status_request) sendStatus();
//review: reading switch in core
#ifdef EMERGENCY_SWITCH
 emergencySwitch();
#endif

  if(time_display > millis()) return;
  time_display = millis() + 10; 
 
 }

 #ifdef EMERGENCY_SWITCH
 //force a program restart 
 void restart(){ asm volatile ("  jmp 0");  }
 
 void emergencySwitch(){

   if(analogRead(EM_SWITCH)<500) return; 
  
  if(EMERGENCY_SWITCH == 0||EMERGENCY_SWITCH == 1) emergency_hold= TRUE;
  
  //restart the firmware, issue error to sender to hold transmission
  if(EMERGENCY_SWITCH == 2){ cli(); Serial.println(F("error:EMERGENCY SITUATION]")); EEPROM.write(42,EMERGENCY_HOMING_SIGNAL); restart(); }
  
  //just switch on the alarm          
  if(EMERGENCY_SWITCH == 42) { alarmSignal(TRUE); delayMicroseconds(1000); return; }

  //hold current position until em_switch is pressed again
  if(emergency_hold == TRUE){
               cli(); 
               if(EMERGENCY_SWITCH==1) alarmSignal(TRUE);
               delay(50); //debounce interval
               //wait until release
               for(;;){ if(analogRead(EM_SWITCH)<100) break; delay(50); }
               for(;;){
                  delay(1000);
                  if(analogRead(EM_SWITCH)>1000){emergency_hold=FALSE;  break;} 
                  }
               if(EMERGENCY_SWITCH==1) alarmSignal(FALSE);
               delay(500); //just to be sure
               sei(); 
               }
 
 }
#endif

//send a pwm signal to PIN_MISC
void alarmSignal(boolean onoff){
  if(onoff) analogWrite(PIN_MISC,666); //set a pwm signal
     else   analogWrite(PIN_MISC,0);
}


void doSteps(){

#ifdef ENDSWITCHES_ENABLED
     //Homing 
    if(stepperX_endswitch==0)
       if(digitalRead(ENDSWITCH_X)==LOW){
                  stepperX_endswitch=TRUE;//disarm endswitch
                  steps_X_goto = X_MIN_POS; //set position to -2mm
                  steps_X_pos  = steps_X_goto;
                  stepperX_enabled = 1;
                  }
    if(stepperY_endswitch==0)
       if(digitalRead(ENDSWITCH_Y)==LOW){
                  stepperY_endswitch=TRUE;
                  steps_Y_goto = Y_MIN_POS;
                  steps_Y_pos  = steps_Y_goto;
                  stepperY_enabled = 1;
                  }
    if(stepperZ_endswitch==0)
       if(digitalRead(ENDSWITCH_Z)==LOW){
                  stepperZ_endswitch=TRUE;
                  steps_Z_goto = Z_MAX_POS;
                  steps_Z_pos  = steps_Z_goto;
                  stepperZ_enabled = 1;
                  }
    
    //for the homing run we need to go under min pos. the endswitches save it here
    //and reference the min pos as exact as they can.
    if(stepperX_endswitch && steps_X_goto < X_MIN_POS) steps_X_goto = X_MIN_POS; 
    if(stepperY_endswitch && steps_Y_goto < Y_MIN_POS) steps_Y_goto = Y_MIN_POS;
    if(stepperZ_endswitch && steps_Z_goto > Z_MAX_POS) steps_Z_goto = Z_MAX_POS;
 #endif
 
    if(steps_X_goto > X_MAX_POS){
            if(toolchange_procedure){ if(steps_X_goto > X_TOOLCHANGE_POS) steps_X_goto = X_TOOLCHANGE_POS;}
               else  { steps_X_goto=X_MAX_POS; }
            }
    if(steps_Y_goto > Y_MAX_POS) steps_Y_goto=Y_MAX_POS;
    if(steps_Z_goto < Z_MIN_POS) steps_Z_goto=Z_MIN_POS;

     if(steps_Z_goto > steps_Z_pos) doStepZ(1,1);
     if(steps_Z_goto < steps_Z_pos) doStepZ(1,0); 
      
     if(steps_X_goto > steps_X_pos) doStepX(1,1);
     if(steps_X_goto < steps_X_pos) doStepX(1,0);  
  
     if(steps_Y_goto > steps_Y_pos) doStepY(1,1);
     if(steps_Y_goto < steps_Y_pos) doStepY(1,0);

     if(steps_X_goto == steps_X_pos && steps_Y_goto == steps_Y_pos && steps_Z_goto == steps_Z_pos) machine_idle = TRUE;
  
       
  }

void doStepX(boolean s_step, boolean s_direction){
  if(!s_step) return;
  if(++steps_X_step < stepperX_delay) return;

  if(s_direction){
    if(++stepperX_spos>7) stepperX_spos=0;
    for(byte i=0;i<4;i++)
      if(steppers_enable) digitalWrite(stepperX_pins[i],bitRead(stepper_sequence_ccw[stepperX_spos],i));
    steps_X_pos++;   
    } else {
    if(++stepperX_spos>7) stepperX_spos=0;
    for(byte i=0;i<4;i++)
      if(steppers_enable) digitalWrite(stepperX_pins[i],bitRead(stepper_sequence_clw[stepperX_spos],i));
    steps_X_pos--;
    }
  steps_X_step = 0; 
    
  }
void doStepY(boolean s_step, boolean s_direction){
  if(!s_step) return;
  if(++steps_Y_step < stepperY_delay) return;
  
  if(s_direction){
    if(++stepperY_spos>7) stepperY_spos=0;
    for(byte i=0;i<4;i++)
      if(steppers_enable) digitalWrite(stepperY_pins[i],bitRead(stepper_sequence_ccw[stepperY_spos],i));
    steps_Y_pos++;
    
    } else {
    if(++stepperY_spos>7) stepperY_spos=0;
    for(byte i=0;i<4;i++)
      if(steppers_enable) digitalWrite(stepperY_pins[i],bitRead(stepper_sequence_clw[stepperY_spos],i));
    steps_Y_pos--;
    }
 steps_Y_step=0;   
  }
  
void doStepZ(boolean s_step, boolean s_direction){
  if(!s_step) return;
  if(++steps_Z_step < stepperZ_delay) return;
  
  if(s_direction){
    if(++stepperZ_spos>7) stepperZ_spos=0;
    for(byte i=0;i<4;i++)
      if(steppers_enable) digitalWrite(stepperZ_pins[i],bitRead(stepper_sequence_ccw[stepperZ_spos],i));
    steps_Z_pos++;
    
    } else {
    if(++stepperZ_spos>7) stepperZ_spos=0;
    for(byte i=0;i<4;i++)
      if(steppers_enable) digitalWrite(stepperZ_pins[i],bitRead(stepper_sequence_clw[stepperZ_spos],i));
    steps_Z_pos--;
    }
 steps_Z_step=0;   
  }
  
//set all stepper pins to off - saving power  
void steppersPowersave() {
  if(steps_X_goto == steps_X_pos ) for(byte i=0;i<4;i++) digitalWrite(stepperX_pins[i],0);
  if(steps_Y_goto == steps_Y_pos ) for(byte i=0;i<4;i++) digitalWrite(stepperY_pins[i],0);
  if(steps_Z_goto == steps_Z_pos ) for(byte i=0;i<4;i++) digitalWrite(stepperZ_pins[i],0);
  }
  
//need to wait until the set goto is reached
void waitForDisposition(){
 
 while(steps_X_pos != steps_X_goto || steps_Y_pos != steps_Y_goto || steps_Z_pos != steps_Z_goto ){
     machine_idle = FALSE;
     doDisplay();
     }
 
  }
 
 
//Tool & Fan control - supporting ON/OFF only, may change to support PWM
void toolControl(boolean mode){
  tool_motor = mode;
  if(mode) digitalWrite(TOOL_CONTROL,LOW); //ON
    else   digitalWrite(TOOL_CONTROL,HIGH);//OFF
  }
void fanControl(boolean mode){
  fan_motor = mode;
  if(mode) digitalWrite(FAN_CONTROL,LOW); 
    else   digitalWrite(FAN_CONTROL,HIGH);
  }

//grab chars from stream 
void doSerialComm(){
   
   if(!ready_for_serial) { machineInit(); return;} 
   
   if(Serial.available()>0){
     cli();
     buf[ptr] = Serial.read();
     
     //preprocess some special chars
     if(buf[ptr] == 13) { buf[ptr]=0; sei(); return; }//ignore CR
     //grbl status request  - answer with wpos mpos in next intr
     if(buf[ptr] =='?'){ buf[ptr]=0; status_request = TRUE; sei(); return ; }
     //other priority signals: ~ cycle start/resume, !feed hold, ^X soft reset
     if(buf[ptr]=='~') { buf[ptr]=0; sei(); return; }//cycle start (reset machine with endswitches)
     if(buf[ptr]=='!') { buf[ptr]=0; sei(); return; }//halt feed 
     if(buf[ptr]==24)  { buf[ptr]=0; sei(); return; }//^X reset (asm jmp 0)  
      //Serial.available
   
     if(buf[ptr] == 10){
       if(ptr<2) { buf[ptr]=0; ptr=0; sei();  return ; } //short line, ignore
       byte i;
       for(i=0;i<ptr;i++) { serial_command[i] = buf[i]; buf[i]=0; }
       serial_command[i]=0; buf[ptr]=0; ptr=0; 
       serial_command_available = TRUE;
       sei();
       return;
       }
     if(ptr++>78) ptr=0;
     sei();
     }   

 }


void machineInit(){//test the homing condition
  
  if(machine_init_delay++<250) return;
  
 //if homing is done (or disabled) send boot message (grbl mimics)
 if(homing_done){
       ready_for_serial = TRUE;
       control_mode = CONTROL_SERIAL;
       sendVersion(); 
       sendHelp();
       return; 
     }
  
  machine_init_delay = 0; 
 
 }
   
 //we answer with ok meaning command is parsed, accepted and executed
 //error:[...] for anything failed
 void executeSerialCommand(){
  if(!serial_command_available) return;
  
  machine_idle = FALSE;
  
  int res = parseSerialCommand();
  while(machine_idle==FALSE) doDisplay();
  
  if(res>=0) {   sendOK();  return;    }
  
  if(res==-3) { sendError(F("parameter wrong / missing]")); return; }
  if(res==-10) { sendError(F("Coordinate out of Bounds]")); return; }
  if(res==-21) { sendError(F("Feedrate error]")); return; }
  if(res==-31) { sendError(F("still HOMING]")); return; }
  if(res==-101) { sendError(F("HALT mode activated]")); return; }
  if(res==-121) { sendError(F("toolchange mode - no G commands]")); return; }
  

 sendError(F("error:[unknown command or parameter]"));
 }
 
 //this is the main control routine, we parse our command, split it into parts and
 //in second step the command is routed/executed. Not resource effective, but easy to
 //understand where what is happening and even more easy to expand
 //called by executeSerialCommand, it returns the appropriate error code that is
 //translated into a error message by executeSerialCommand.
 
 int parseSerialCommand(){
   char cmd[80];
   cli(); //do not interrupt here
   strcpy(cmd,serial_command);
   serial_command[0] = 0;
   serial_command_available = 0 ;
   sei();
  
  //do not process anything until serial control is enabled
  //review: this should never happen
  if(control_mode == CONTROL_INTERNAL) {  return -31; }
  
 #ifdef DEBUG
   Serial.print(F("DEBUG: CMD="));Serial.print(cmd);Serial.print("(");Serial.print(strlen(cmd));Serial.println(").");
 #endif
   
   //the standard gcode ISO 6983 - no :% chars
   //                            - dimension chars: XYZUVWPQRABC
   //                            - interpolation/thread cutting IJK
   //                            - decimal sign . leading 0 or decimal may by omitted 10 10.00mm .1 0.1mm
   //                            - G90/G91 absolute or incremental argument following
   //                            - degrees in decimal or part of decimal
   //                            - +- is part of the dimension word
   //review: all numbers are translated using the ato functions. 
   
   char cmd_part[8][16],*tmp; //cmdno cmd x y z part 
   byte j=0,k=0,n_parts=0,ng=0;
   boolean has_whitespace=FALSE; //some grbl senders strip spaces  
   int ret=-1; //default return value
   
   for(byte i=0;i<strlen(cmd);i++){
     if(cmd[i]=='G') ng++; //count the number of G's in the command
     if(cmd[i]==32) { has_whitespace = TRUE; }
     }

   if(!has_whitespace && strlen(cmd)>2){ //do not parse short cmds here
     for(byte i=0;i<strlen(cmd)+1;i++){//no ws so we need to use 0 for last arg
       cmd_part[j][k]=0;
       if(cmd[i] < 33) { j++; k=0; continue;} //next part
       if(cmd[i] > 64 && cmd[i] < 91 && i>0){ j++;k=0;cmd_part[j][k]=cmd[i] ; continue; }
       cmd_part[j][k++] = cmd[i];
       if(j>6||k>14) break;
       } 
     } else {
      for(byte i=0;i<=strlen(cmd);i++){
         cmd_part[j][k]=0;
         if(cmd[i]==32&&cmd[i+1]==32) continue; //strip multiple blanks
         if(cmd[i] < 33) { j++; k=0; continue;} //next part
         cmd_part[j][k++] = cmd[i];
         if(j>6||k>14) break;
         } 
     }
   cmd_part[j][k]=0;
   n_parts = j;
   
   int N=-1,M=-1,S=0,T=0;
   float G=-1.0;
   double X=9999.0,Y=9999.0,Z=9999.0,I=9999.0,J=9999.0,K=9999.0,R=9999.0,FR=0.0,E=0.0,L=0.0,P=0.0,Q=0.0; 
   
   //load the abs/rel settings from defaults
   boolean this_dimension_absolute = dimension_absolute; // G90/91 ->G0G1G2G3
   boolean this_cp_absolute = cp_absolute; //G90.1/91.1 ->IJ G2G3 
   
   for(byte i=0;i<n_parts;i++){
     if(cmd_part[i][0]==0) { n_parts=i-1; break; }
     tmp=&cmd_part[i][1];

  //command preprocessor part 
  
     //line numbers are monitored but mostly ignored.
     if(cmd_part[i][0] == 'N'){
       N = atoi(tmp);
       if(N <= command_last_line) { continue;} //ignore the line count
       command_last_line = N;
       continue;
       }
           
      if(cmd_part[i][0] == '$'){ return grblCommand(tmp); }
 
      if(cmd_part[i][0] == '(') { return 0; } //this is comment - should not get here
      if(cmd_part[i][0] == '%') { return 0; } //this is comment - should not get here

#ifdef PWM_SPEED_CONTROL
      //spindle on/off is controlled by relais ... so we write a permanent pwm to the 
      //pwm pin. The value area is -10000 to +10000 * 0,0512 -> Analog pin
      if(cmd_part[i][0] == 'S') { 
            if(S>10000||S<-10000) return -1;
            analogWrite(PWM_OUT,S*0.0512);
            continue;
            } 
#else
      if(cmd_part[i][0] == 'S') { return 0; } //spindle speed - ignored
#endif
      
     //some senders send things like G91 G21 as the command, we do not want this to be an error 
     //but the G91 and G91.1 setting would affect the current line only
    
     if(ng>1){ //we have multiple G in this line so the "setting" G's only count for this command

       float arg = atof(tmp);
       if(cmd_part[i][0] == 'G' && arg==90){ this_dimension_absolute=TRUE; continue; } 
       if(cmd_part[i][0] == 'G' && arg==90.1){ this_cp_absolute=TRUE; continue; } //current cmd incremental 
       if(cmd_part[i][0] == 'G' && arg==91){ this_dimension_absolute=FALSE; continue; } //current cmd incremental 
       if(cmd_part[i][0] == 'G' && arg==91.1){ this_cp_absolute=FALSE; continue; } //current cmd incremental 
       if(cmd_part[i][0] == 'G' && arg==17){ ret=0; continue; } //select XY plane (ignored, may need review)
       if(cmd_part[i][0] == 'G' && arg==20){ unit_is_mm=FALSE; setStepsPerUnit(INCH); ret=0; continue; } //select unit inch
       if(cmd_part[i][0] == 'G' && arg==21){ unit_is_mm=TRUE; setStepsPerUnit(MM); ret=0; continue; } //select unit mm 
       }  
 
 //command and arg processing
     if(cmd_part[i][0] == 'G' && G<0){ G = atof(tmp); continue; } //accept only one G
     if(cmd_part[i][0] == 'M'){ M = atoi(tmp); continue; }
     if(cmd_part[i][0] == 'X'){ X = atof(tmp); continue; }
     if(cmd_part[i][0] == 'Y'){ Y = atof(tmp); continue; }
     if(cmd_part[i][0] == 'Z'){ Z = atof(tmp); continue; }
     if(cmd_part[i][0] == 'F'){ FR = atof(tmp); continue; }
     if(cmd_part[i][0] == 'I'){ I = atof(tmp); continue; }
     if(cmd_part[i][0] == 'J'){ J = atof(tmp); continue; }
     if(cmd_part[i][0] == 'K'){ K = atof(tmp); continue; }
     if(cmd_part[i][0] == 'L'){ L = atof(tmp); continue; }
     if(cmd_part[i][0] == 'P'){ P = atoi(tmp); continue; }
     if(cmd_part[i][0] == 'Q'){ Q = atof(tmp); continue; }
     if(cmd_part[i][0] == 'S'){ S = atoi(tmp); continue; }
     if(cmd_part[i][0] == 'E'){ E = atof(tmp); continue; }
     if(cmd_part[i][0] == 'R'){ R = atof(tmp); continue; }
     if(cmd_part[i][0] == 'T'){ T = atoi(tmp); continue; }
         
    }//for parts

    //Z translation - offset needs to be set 
    if(Z<9999 && translate_Z_offset!=0 ) { Z=Z+translate_Z_offset;  }

#ifdef DEBUG
    Serial.print(F("DEBUG:")); Serial.print(N);Serial.print(":");
    Serial.print("G:");Serial.print(G);Serial.print(":");
    Serial.print("M:");Serial.print(M);Serial.print(":");
    Serial.print("X:");Serial.print(X);Serial.print(":");
    Serial.print("Y:");Serial.print(Y);Serial.print(":");
    Serial.print("Z:");Serial.print(Z);Serial.print(":");
    Serial.print("FR:");Serial.print(FR);Serial.print(":");
    Serial.print("P:");Serial.print(P);Serial.print(":");
    Serial.print("Q:");Serial.print(Q);Serial.print(":");
    Serial.print("I:");Serial.print(I);Serial.print(":");
    Serial.print("J:");Serial.print(J);Serial.print(":");
    Serial.print("K:");Serial.print(K);Serial.print(":");
    Serial.print("L:");Serial.print(L);Serial.print(":");
    Serial.print("E:");Serial.print(E);Serial.print(":");
    Serial.print("R:");Serial.print(R);Serial.print(":");
    Serial.print("T:");Serial.print(T);Serial.print(":");
    
    Serial.println(".");
#endif
    
   //parsing done, see what we have

    //M00   - halt prog  
    // M0 is not really necessary here since we operate on single command mode, 
    //    the sender has to handle the pause.  
   if( M == 0 ) { //resume serial control with M00 P01 - review: obsolete
      waitForDisposition();
      if(P==0) { control_mode = CONTROL_INTERNAL; }
      if(P==1) { control_mode = CONTROL_SERIAL;   }
     return -101; //we return an error here to force the sender into pause mode 
    }
   if(M == 1) { //pause with powersave
     waitForDisposition();
     steppersPowersave();
     toolControl(OFF);
     fanControl(OFF);
     return -101;
     }
 
   if(M == 2|| M == 30) { //end program - reset settings, turn off things - review
     setStepperDelays(X_max_speed,Y_max_speed,Z_max_speed);
     steps_Z_goto = 0; 
     Serial.println(F("[***END PROGRAM** - resume Z]"));
     toolControl(OFF);
     fanControl(OFF);
     tool_auto_off = OFF;
     waitForDisposition();
     steps_X_goto = 0; steps_Y_goto = 0; 
     waitForDisposition();
     dimension_absolute = TRUE; //reset to default
     cp_absolute = TRUE; //reset to default
     command_last_line = 0;
     unit_is_mm = MM; 
     translate_Z_offset = 0.0;
     
#ifdef DEBUG
     Serial.println(F("[M02 ***PROGRAM ENDED***]"));
#endif
     return 0;
     }
   
   //spindle cw/ccw needs PWM support
   if(M==3||M==4) { toolControl(ON); return 0; }
   if(M==5)       { toolControl(OFF); return 0; }
     
   if(M==6){ //Toolchange procedure
      //this will drive the X table into a proper position 
      //and stay in toolchange mode until another M6 command comes in.
      //so the user has sufficient time to change the tool. 
      //in toolchange mode, only M commands are processed. 
      //to force ugs to pause transmission, we issue an error 
      
      if(!toolchange_procedure){
            waitForDisposition();
            sendError(F("TOOLCHANGE OPERATION]"));
            current_steps_Z_pos=steps_Z_pos;
            steps_Z_goto = 0;
            waitForDisposition();
            tool_status= tool_motor;
            fan_status = fan_motor;
            toolControl(OFF);
            fanControl(OFF);
            current_steps_X_pos = steps_X_pos;
            toolchange_procedure = TRUE;
            steps_X_goto = X_TOOLCHANGE_POS;
            waitForDisposition();
            }else {
            steps_X_goto = current_steps_X_pos;
            waitForDisposition();
            toolControl(tool_motor);
            fanControl(fan_motor);
            steps_Z_goto = current_steps_Z_pos;
            waitForDisposition();
            toolchange_procedure = FALSE;
            return 0;
            }
      }
   
   if(M==10) {fanControl(ON); return 0;} 
   if(M==11) {fanControl(OFF); return 0;} 
        
   if(M==17) { steppers_enable = TRUE; return 0; }//enable steppers
   if(M==18) { steppers_enable = FALSE; steppersPowersave(); return 0; }
   if(M==37) { checkMode(); steppersPowersave(); return 0; } //simulation mode like $C
   

   if(M==114) return 0;
     
    
    //Z-translator - if Z is from 20mm to 0 M934 P-20.0 - review: no sanity check for values!
    if(M==934) {  if(P==0) return -1;  translate_Z_offset=P; return 0; }
        
    if(M==935) {  translate_Z_offset = 0.0; return 0;       }

    //if a file is canceled and then resent, the toggle toggled wrong. 
    if(M==942) { tool_auto_off = ON; return 0; }
    if(M==943) { tool_auto_off = OFF; return 0; }
       
    //M-commands done
    
    if(toolchange_procedure) return(-121); //do not proceed G commands while active
    
    if(FR>0) return setFeedrate(FR); //single F-command - set new feedrate 

    //see if we have a limit violation
    if(!check_X_limit(getSteps(steps_X_pos,steps_X_perunit,X,this_dimension_absolute))) return -10;
    if(!check_Y_limit(getSteps(steps_Y_pos,steps_Y_perunit,Y,this_dimension_absolute))) return -10;
    //use the Z translation feature with care 
    //Serial.print(" Z:");Serial.print(Z);Serial.print(" sZ:");Serial.println(abs(Z)* steps_Z_perunit);
    if(!check_Z_limit(getSteps(steps_Z_pos,steps_Z_perunit,Z,this_dimension_absolute))) return -10;  //Z works only in negative Quadrant
    //the circle center may be out of bounds

    
    if(G == 0){ 
      setStepperDelays(X_max_speed,Y_max_speed,Z_max_speed);

      if(Z<9999) steps_Z_goto=getSteps(steps_Z_pos,steps_Z_perunit,Z,this_dimension_absolute);   
      waitForDisposition();  

      if(tool_auto_off) toolControl(OFF);

      if(X<9999) steps_X_goto=getSteps(steps_X_pos,steps_X_perunit,X,this_dimension_absolute);
      if(Y<9999) steps_Y_goto=getSteps(steps_Y_pos,steps_Y_perunit,Y,this_dimension_absolute);
#ifdef DEBUG
      Serial.print("G0 X:");Serial.print(steps_X_goto);Serial.print(" Y:");Serial.print(steps_Y_goto);
      Serial.print(" Z:");Serial.println(steps_Z_goto);
      sendSettings();
#endif
      waitForDisposition();
      return 0;
     } 
     
   //G1 running in material command. 
   if(G == 1){ 
      waitForDisposition(); //make sure the last command is done 
      long x=steps_X_pos;
      long y=steps_Y_pos;
      long z=steps_Z_pos;
      setStepperDelays(X_work_speed,Y_work_speed,Z_work_speed); //set default work speeds
     if(Z < 9999) { 
        if(FR>0) stepperZ_delay = calculateDelay(FR, steps_Z_perunit,STEPPERZ_MAX_DELAY);
        z=getSteps(steps_Z_pos,steps_Z_perunit,Z,this_dimension_absolute);
        }
      if(X < 9999) {
       if(FR>0) stepperX_delay = calculateDelay(FR, steps_X_perunit,STEPPERX_MAX_DELAY);
       x=getSteps(steps_X_pos,steps_X_perunit,X,this_dimension_absolute);
       }
      if(Y < 9999){
       if(FR>0) stepperY_delay = calculateDelay(FR, steps_Y_perunit,STEPPERY_MAX_DELAY);
       y=getSteps(steps_Y_pos,steps_Y_perunit,Y,this_dimension_absolute);
       }
      
#ifdef DEBUG
      Serial.print("G1 goto X:");;Serial.print(x);Serial.print(" Y:");Serial.print(y);Serial.print(" Z:");Serial.println(z);
      sendSettings();
#endif
      if(tool_auto_off==TRUE) toolControl(ON);
      return doLine(x,y,z,FR);
     }

   if(G == 2||G==3){
      long  stepsX=steps_X_pos,stepsY=steps_X_pos,stepsZ=steps_Z_pos,stepsI=steps_X_pos,stepsJ=steps_Y_pos;
      stepsX=getSteps(stepsX,steps_X_perunit,X,this_dimension_absolute);
      stepsY=getSteps(stepsY,steps_Y_perunit,Y,this_dimension_absolute);
      stepsZ=getSteps(stepsZ,steps_Z_perunit,Z,this_dimension_absolute);
 
      stepsI = getSteps(steps_X_pos,steps_X_perunit,I,this_cp_absolute);
      stepsJ = getSteps(steps_Y_pos,steps_Y_perunit,J,this_cp_absolute);
      
      if(X==9999||Y==9999||I==9999||J==9999) return -10;     
      waitForDisposition();
      if(tool_auto_off==TRUE) toolControl(ON);
      if(G==2) { return doArc(stepsI,stepsJ,stepsX,stepsY,stepsZ,0,FR); }
       else    { return doArc(stepsI,stepsJ,stepsX,stepsY,stepsZ,1,FR); }
     
     return -1;
     }
   //Dwell - pause for P millis S secs
   if(G== 4){
      int pause_millis;
      if(P>0) pause_millis = P;
      if(S>0) pause_millis = S*1000;
      long t = millis() + pause_millis;
      while(t>millis());
      return 0;
      }
      
    //halt at the last given coords
    if(G==9){ waitForDisposition(); return 0;}
     
    if(G==17) return 0;  //select XY plane, default and only setting
    if(G==18) return -1; //no other plane selectable
    if(G==19) return -1;
    
    if(G==20) { unit_is_mm=FALSE; setStepsPerUnit(INCH); return 0; }//set outdated inches (still happens)
    if(G==21) { unit_is_mm=TRUE;  setStepsPerUnit(MM); return 0; }//co-ordinates mm
     
   if(G == 28){ //review: G28 with intermediate point?
      stepperX_delay = X_max_speed;
      stepperY_delay = Y_max_speed;
      stepperZ_delay = Z_max_speed;
      if(steps_Z_pos < 0) { steps_Z_goto = 0; waitForDisposition();} //move Z first
       if(tool_auto_off==TRUE) toolControl(OFF);
      steps_X_goto = 0;
      steps_Y_goto = 0;
      steps_Z_goto = 0;
      waitForDisposition();
      return 0;
     }

   if(G == 80) return 0; //drill cycle parameter - ignore (injected by Freecad)
   
   //review: before issuing a drill cmd Z should be at a safe position. 
   //review: parameter needed check
   //we then move to xy and than z to retract pos.      
   if(G == 81||G == 82 ) { // Drill mode G82 XYZ Rretract Pdwell Ffeed Lrepeats
     long bottom_hole = 0.0;
     if(Z<9999) { //review: this should always be absolute here, shouldnt it?
        stepperZ_delay =  Z_DRILL_SPEED ; 
        if(FR>0) stepperZ_delay = calculateDelay(FR, steps_Z_perunit,STEPPERZ_MAX_DELAY);
        bottom_hole=getSteps(steps_Z_pos,steps_Z_perunit,Z,this_dimension_absolute);
        } else return -1;
        
     long retract_pos = 0;
     if(R<9999) { 
        retract_pos=getSteps(steps_Z_pos,steps_Z_perunit,R,this_dimension_absolute);
        if(!check_Z_limit(retract_pos)) return -10;

        }
      long stepsX=0,stepsY=0;
      stepsX=getSteps(steps_X_pos,steps_X_perunit,X,this_dimension_absolute);
      stepsY=getSteps(steps_Y_pos,steps_Y_perunit,Y,this_dimension_absolute);
     
     stepperX_delay = X_max_speed;
     stepperY_delay = Y_max_speed;
     if(stepsX!=0x80000000) steps_X_goto = stepsX; 
     if(stepsY!=0x80000000) steps_Y_goto = stepsY;
     waitForDisposition();

     steps_Z_goto = retract_pos;
     waitForDisposition();  
     //minimum dwell time for G81
     int dwell_time = 10; if(P>0.1) dwell_time = P * 1000; //ms
     int repeats=0; if(L>0) repeats=L;
     
     if(tool_auto_off==TRUE) toolControl(ON);
     return doDrill(bottom_hole,retract_pos,dwell_time,repeats);
    }//G82
   
   if(G == 83 ) { //Drill with peck G83 XYZ R-retractpos Pdwelltime Qincrperpeck  Lnum repeats
     long bottom_hole = 0.0;
     if(Z<9999) { //this should always be absolute here, shouldnt it?
         stepperZ_delay =  Z_DRILL_SPEED ; 
        if(FR>0) stepperZ_delay = calculateDelay(FR, steps_Z_perunit,STEPPERZ_MAX_DELAY);
        
        bottom_hole=getSteps(steps_Z_pos,steps_Z_perunit,Z,this_dimension_absolute);
        } else return -1; //if not there this is an error 
        
     long retract_pos = 0;
     if(R<9999) { 
        retract_pos=getSteps(steps_Z_pos,steps_Z_perunit,R,this_dimension_absolute);
        if(!check_Z_limit(retract_pos)) return -10;
        } else return -1; 
        
      long stepsX=-1,stepsY=-1;
      stepsX=getSteps(steps_X_pos,steps_X_perunit,X,this_dimension_absolute);
      stepsY=getSteps(steps_Y_pos,steps_Y_perunit,Y,this_dimension_absolute);
     
     stepperX_delay = X_max_speed;
     stepperY_delay = Y_max_speed;
     if(stepsX!=0x80000000) steps_X_goto = stepsX; 
     if(stepsY!=0x80000000) steps_Y_goto = stepsY;
     waitForDisposition();

     steps_Z_goto = retract_pos;
     waitForDisposition();  
     
     if(tool_auto_off==TRUE) toolControl(ON);
     
     int dwell_time = 500; if(P>0.1) dwell_time = P * 1000; //ms
     int incpp = 1; if(Q>0) incpp=Q*steps_Z_perunit; 
     int repeats = 2; if(L>2) repeats=L;
     
      //G83 XYZ R-retractpos Pdwelltime Qincrperpeck  Lnum repeats
      return doPeck( bottom_hole, retract_pos, dwell_time, incpp, repeats);
      }
     
   //set the rel/abs interpretation - query with $G 

   if(G==90)  { dimension_absolute = TRUE; return 0; }
   if(G==90.1){ cp_absolute = TRUE; return 0; }
   if(G==91)  { dimension_absolute = FALSE; return 0; }
   if(G==91.1){ cp_absolute = FALSE; return 0; }

   //this manipulates the 0 position for one or more axes, 
   //setting current position as 0 with no arguments.
   if(G == 92 && G92enable){ //review for better error feedback
      waitForDisposition();
      if(X<9999) steps_X_goto = getSteps(steps_X_pos,steps_X_perunit,X,this_dimension_absolute);
      if(Y<9999) steps_Y_goto = getSteps(steps_Y_pos,steps_Y_perunit,Y,this_dimension_absolute);
      if(Z<9999) steps_Z_goto = getSteps(steps_Z_pos,steps_Z_perunit,Z,this_dimension_absolute);       
      
      waitForDisposition();
      
      if(X<9999) { steps_X_pos=0; steps_X_goto=0; }
      if(Y<9999) { steps_Y_pos=0; steps_Y_goto=0; }
      if(Z<9999) { steps_Z_pos=0; steps_Z_goto=0; }
      
     return 0;
     }
     
   if(G==94){ //set feed work speed
     feedrate = FR;
     if(FR>0) return setFeedrate(FR); 
     return -3; //parameter wrong or missing
     }
     
   if(G==97){ //Spindle RPM
      //S for spindle RPM
      return 0; //currently this can be done manually only
     }

  //G commands done
    
  return ret;
 }
 
long getSteps(long offset,long steps_perunit,float units,boolean absolute){
  if(units == 9999) return 0x80000000; //max long
  if(absolute) return units * steps_perunit;  
       else    return offset + units * steps_perunit;
 }
 
void setStepsPerUnit(boolean unit){
#ifdef DEBUG_UNIT
 Serial.print("SET: "); printUnit();
#endif
  if(unit == MM){
    steps_X_perunit = X_STEPS_permm;
    steps_Y_perunit = Y_STEPS_permm;
    steps_Z_perunit = Z_STEPS_permm;
    } else {
    steps_X_perunit = X_STEPS_perInch;
    steps_Y_perunit = Y_STEPS_perInch;
    steps_Z_perunit = Z_STEPS_perInch;    
    }
}

void printUnit(){
  Serial.print(F("Unit "));
  if(unit_is_mm) Serial.println(F("MM"));
  else           Serial.println(F("INCH"));
 }

//
// this works off the grbl specific commands $$ etc.
int grblCommand(char *n){

        if(n[0]=='G') { sendSettings(); return 0; } 
        if(n[0]=='$') { sendParameters(255);  return 0; }
        if(n[0]==0)   { sendHelp(); return 0; } //not ideal here
        if(n[0]=='C') { checkMode() ; steppersPowersave(); return 0;}
        if(n[0]=='I') { sendVersion() ; return 0;} //view build info
        if(n[0]=='X') { return 0; }  //deactivate alarm lock, currently not supported
        if(n[0]=='H') { return 0; } //force homing, currently not supported
        byte param_nr; 
        if(n[0]>47 && n[0]<58) { //we do not really have all this params
             param_nr=(byte)atoi(n);
             sendParameters(param_nr);
             }
         
  return 0;
 }

//check hard limits in mm or steps, return TRUE if in limit
 boolean check_X_limit(long x_pos){ //pos in steps
    if(x_pos==0x80000000) return TRUE; //not set 
    if(x_pos>0 && x_pos>X_MAX_POS) return FALSE;
    if(x_pos<0 && x_pos<X_MIN_POS) return FALSE;
   return TRUE;
   }
 boolean check_Y_limit(long y_pos){ //pos in steps 
    if(y_pos==0x80000000) return TRUE;
    if(y_pos>0 && y_pos>Y_MAX_POS) return FALSE;
    if(y_pos<0 && y_pos<Y_MIN_POS) return FALSE;
   return TRUE;
   }
 boolean check_Z_limit(long z_pos){ //pos in steps 
    if(z_pos==0x80000000) return TRUE;
    if(z_pos>0 && z_pos>Z_MAX_POS) return FALSE;
    if(z_pos<0 && z_pos<Z_MIN_POS) return FALSE;
   return TRUE;
   }

void  setStepperDelays(byte x,byte y,byte z){
      stepperX_delay =  x;
      stepperY_delay =  y;
      stepperZ_delay =  z;
  }
  
//this is a bit stupid command since the individual G commands better should
//set the feedrate to support complex movement. Since we might want to change
//our work speed here though, this rate is set for all axes. 
 int setFeedrate(float fr){
     if(fr <= 0 || fr > MAX_XY_FEEDRATE) return -21;
   
   X_work_speed = calculateDelay(fr,steps_X_perunit,STEPPERX_MAX_DELAY);
   Y_work_speed = calculateDelay(fr,steps_Y_perunit,STEPPERY_MAX_DELAY);
   Z_work_speed = calculateDelay(fr,steps_Z_perunit,STEPPERZ_MAX_DELAY);
   feedrate = fr;
#ifdef DEBUG
  sendSettings();
#endif
 return 0;
 }
 
 
 //calculate feedrate mm/min for xy distance from current stepper delay 
 //assumed x/y_stepspermm are same 
 float calculateXYFeedrate(unsigned long dx,unsigned long dy){
   //distance in steps
   double steps_xy=sqrt(dx*dx+dy*dy);
   double steps_per_sec = (STEPS_PER_SECOND/(stepperX_delay+1)) + (STEPS_PER_SECOND/(stepperY_delay+1)); //1/0.00125 800 steps per sec
   double secs = steps_xy / steps_per_sec;
   double mm=steps_xy / steps_X_perunit; 
   float fr = (mm / secs)*60;
#ifdef DEBUG
   Serial.print("cFR:");Serial.println(fr,3);
#endif
   return fr;
  }
  
//calculate the feedrate from given delay and stepsperunit
 float calculateFeedrate(byte s_delay, long steps_perunit){
  return(float)STEPS_PER_SECOND/(float)steps_perunit*60.0/(float)(s_delay+1);
 }
 
//returns the delay from a given feedrate  
 byte calculateDelay(float fr, long steps_perunit,byte max_delay){

byte s_delay = ceil(STEPS_PER_SECOND/((fr/60) * steps_perunit));

 if(s_delay>max_delay) { return 255; }

 if(s_delay>0) s_delay--; 
  
 return s_delay;
 }

//set XY delays according to the current feedrate 
//since each delay increase halves the stepper speed, setting a feedrate
//can not be done really exact. So we do not calculate the steps needed, 
//just set the higher the delay the lower the feedrate is.
//review 
void setXYDelay(unsigned long dx,unsigned long dy){
    
    double sps = 1/((feedrate / 60.0)*steps_X_perunit); //steps per second
         
    int s_delay = ceil(sps/0.00125)-1;
    if(s_delay < 0) s_delay = 0;
    if(s_delay > STEPPERX_MAX_DELAY) s_delay = STEPPERX_MAX_DELAY;
    stepperX_delay = s_delay;
    stepperY_delay = s_delay;
#ifdef DEBUG
    Serial.print("FR:");Serial.print(feedrate,3);Serial.print(" SD:");Serial.println(s_delay);
#endif
  }
  

//the besenhams do not work well here, the large step numbers
//prevent the usage of the error feature. 
//For testing XYZ may be provided, moving Z in steps relative
//to XY increase - use with care
//review: feedrate
int doLine(long x1, long y1,long z1,float fr){
  
  waitForDisposition();
  machine_idle = FALSE;

  if(x1==0x80000000) x1=steps_X_pos;
  if(y1==0x80000000) y1=steps_Y_pos;
  if(z1==0x80000000) z1=steps_Z_pos;
  
  //if its a single Z move just move it 
  if(x1==steps_X_pos&&y1==steps_Y_pos&&z1!=steps_Z_pos) {steps_Z_goto=z1; waitForDisposition(); return 0; }
  
  long x0 = steps_X_pos;
  long y0 = steps_Y_pos;
  long z0 = steps_Z_pos;
  long dx=0,dy=0,dz=0;
  
  int sx=0,sy=0,sz=0;
  
  if(x0>x1) { dx = x0 - x1; sx=-1;}
    else    { dx = x1 - x0; sx=1; }
  if(y0>y1) { dy = y0 - y1; sy=-1;}
    else    { dy = y1 - y0; sy=1;}
  if(z0<z1) { dz = z0 + z1; sz=-1;} //negative space, go in
    else    { dz = z1 - z0; sz=1;} // and out

  //if only very small move to go we go there with workspeed, 10 steps
  //off (0.002mm) seems to be far in machine max precision 
  if(dz==0&&abs(dx)<10 && abs(dy)<10){ steps_X_goto = x1; steps_Y_goto = y1; waitForDisposition(); return 0;}
  
  double xf = (double)dx/(double)STEPSIZE;
  double yf = (double)dy/(double)STEPSIZE;
  int    zf = dz;
  int stepsx = ceil(xf) * sx;
  int stepsy = ceil(yf) * sy;
  int stepsz = ceil(abs((dx/stepsx + dy/stepsy))/dz); //all stepsz move z for one
//review: feedrate
  
#ifdef LINE_DEBUG
  Serial.print("doLine: x=");Serial.print(x0);Serial.print(" y=");Serial.print(y0);
        Serial.print(" x1=");Serial.print(x1);Serial.print(" y1=");Serial.print(y1);
        Serial.print(" dx=");Serial.print(dx);Serial.print(" dy=");Serial.print(dy);
        Serial.print("Z0:"); Serial.print(z0);Serial.print(" Z1:"); Serial.print(z1);
        Serial.print(" dz:");Serial.print(dz);Serial.print(" SZ:");Serial.print(stepsz);
        Serial.print(" xf=");Serial.print(xf,8);Serial.print(" yf=");Serial.print(yf,8);
        Serial.print(" stepsx=");Serial.print(stepsx);Serial.print(" stepsy=");Serial.print(stepsy);
        Serial.print(" stepsz=");Serial.println(stepsz);
#endif
 
  for(;;){
    int zc=0;
    steps_X_goto = steps_X_goto + stepsx;
    steps_Y_goto = steps_Y_goto + stepsy;
    if(dz !=0 && zc++>=stepsz){zc=0; steps_Z_goto+=sz; }
#ifdef LINE_DEBUG
    Serial.print("L: x:");Serial.print(steps_X_goto);
    Serial.print(" y:");Serial.print(steps_Y_goto);
    Serial.print(" z:");Serial.println(steps_Z_goto);
#endif
    waitForDisposition();
 
    if(x0>x1 && steps_X_pos <= x1)  break;
    if(x0<x1 && steps_X_pos >= x1)  break; 
    if(y0>y1 && steps_Y_pos <= y1)  break; 
    if(y0<y1 && steps_Y_pos >= y1)  break;
    

   }
 #ifdef LINE_DEBUG
 Serial.print("doLine done:x1=");Serial.print(x1);
            Serial.print(" y1=");Serial.println(y1); 
            Serial.print(" z1=");Serial.println(z1);
 #endif
 steps_X_goto = x1; //be sure to set the correct position
 steps_Y_goto = y1; 
 waitForDisposition();
 
return 0; 
}

//ARC procedure for cw/ccw arcs max 180 deg per command 
//if the arc behavior looks weird, I,J might be relative

//there is no feedrate calculation done here, doline will 
//keep track of it

//review:
//- cw doesnt always go cw 
//- movement in xz/yz axis (helical)
//- stepsize mm instead of deg

//this currently is still work in progress. 

int doArc(long cx,long cy,long x,long y,long k,boolean ccw,float fr) {
#ifdef ARC_DEBUG
Serial.println("ARC PROC.");
#endif
  waitForDisposition();
  //see from to
  long px = steps_X_pos;
  long py = steps_Y_pos;

  // get radius
  double dx = (double)x - (double)cx;
  double dy = (double)y - (double)cy;
  double radius = sqrt(dx*dx+dy*dy);

  double from_angle = abs(atan2(px-cx,py-cy)); 
  double to_angle   = abs(atan2(x-cx,y-cy));
   
  double theta=to_angle-from_angle;
 
  long nx=steps_X_pos, ny=steps_Y_pos,nk=steps_Z_pos;
  double angle=from_angle;
  double dt=(theta/180.0)*0.5; // 0.5 deg steps 
  long dk =0;
  if(k!=0x80000000&&k!=0) dk=k/(theta/dt); //steps to go / number of steps 
  int steps; 

#ifdef ARC_DEBUG
 Serial.print("cx:");Serial.println(cx);
 Serial.print("cy:");Serial.println(cy);
 Serial.print(" x:");Serial.println(x);
 Serial.print(" y:");Serial.println(y);
 Serial.print(" k:");Serial.println(k);
 Serial.print("radius:");Serial.println(radius);
 Serial.print("from r:");Serial.print(from_angle);
 Serial.print("   deg:");Serial.println(from_angle*180.0/PI);
 Serial.print("  to r:");Serial.print(to_angle);
 Serial.print("   deg:");Serial.println(to_angle*180.0/PI);
 Serial.print(" theta:");Serial.println(theta,8);
 Serial.print("dtheta:");Serial.println(dt,8);
 Serial.print("dk:");Serial.println(dk,8);
#endif
  
  //the default circle is like -90 deg turned and mirrored
  if(ccw){ 
    while((theta>0 && angle <= to_angle)||(theta<0 && angle >= to_angle)){ //from angle to angle in positive steps cw
             
      if(steps++>3600) break;  //emergency exit
      
     double da=abs(angle*180.0/PI);  //making sure to hit a quadrant
     
    //as max arc for G2/3 is 180 deg, if we go from low to high values we have to add 180
    //to run in upside down direction. 
   
     if(from_angle > to_angle) da=da+180; //going from -180 to 0 should be like 180-360
     if (da > 360) da=da - 360;
     int q=-1;
     
    if(da>=0 && da<= 90){ 
      q=0;
      nx =-(sin(angle) * radius)+cx;
      ny =(cos(angle) * radius)+cy;
      }
    if(da> 90 && da<=180){
      q=1;
      nx =-(sin(angle) * radius)+cx;
      ny =(cos(angle) * radius)+cy;
      }
    if(da>180 && da<=270){
      q=2;
      nx =(sin(angle) * radius)+cx;
      ny =(cos(angle) * radius)+cy;
      }
    if(da>270 ){ 
      q=3;
      nx =(sin(angle) * radius)+cx;
      ny =(cos(angle) * radius)+cy;
      }
#ifdef ARC_DEBUG_MAX
      Serial.print("ARCccw:");
      Serial.print(angle);Serial.print(" Deg:");
      Serial.print(da);Serial.print(" Q:");Serial.print(q);
      Serial.print(" sinx:");Serial.print(sin(angle));
      Serial.print(" cosy:");Serial.print(cos(angle));
      Serial.print(" nX:");Serial.print(nx);
      Serial.print(" nY:"); Serial.print(ny);Serial.println(")");
#endif
      angle=angle+dt;
      
      nk=nk+dk;
      doLine(nx,ny,nk,fr);
      waitForDisposition();
      }
    } else {//cw angles
    
    if(theta>=PI) { angle+=(2.0*PI); dt=dt*-1; }
    
    while((theta>0 && angle <= to_angle)||(theta<0 && angle >= to_angle)){ 
    
     double da=abs(angle*180.0/PI);  
     
     if(from_angle > to_angle) da=da+180; 
     if (da > 360) da=da - 360;   
      
      if(steps++>3600) break;  //emergency exit
    
      int q=-1;               //used for debug only
      if(da>=0 && da<=90){
            q=0;
            nx =(sin(angle) * radius)+cx;
            ny =(cos(angle) * radius)+cy;
            }
      if(da>90 && da <=180){ 
            q=1;
            nx =(sin(angle) * radius)+cx;
            ny =(cos(angle) * radius)+cy;
            }
      if(da>180 && da <=270){
            q=2;
            nx =-(sin(angle) * radius)+cx;
            ny =(cos(angle) * radius)+cy;
            }
      if(da>270 ){
            q=3;
            nx =-(sin(angle) * radius)+cx;
            ny =(cos(angle) * radius)+cy;
            }
 #ifdef ARC_DEBUG_MAX     
      Serial.print("ARCcw:");
      Serial.print(angle);Serial.print(" Deg:");
      Serial.print(da);Serial.print(" Q:");Serial.print(q);
      Serial.print(" sinx:");Serial.print(sin(angle));
      Serial.print(" cosy:");Serial.print(cos(angle));
      Serial.print(" nX:");Serial.print(nx);
      Serial.print(" nY:"); Serial.print(ny);Serial.println(")");
 #endif
      angle=angle+dt;    
      
      nk=nk+dk;
      doLine(nx,ny,nk,fr);
      waitForDisposition();
      }
    }
 
 doLine(x,y,k,fr);
 waitForDisposition();

return 0;
}
//Drill mode - G82 XYZ Rretract Pdwell Ffeed Lrepeats
int doDrill(long bottom_hole,long retract_pos,int dwell_time,int repeats){
  long dwell_timer;
  steps_Z_goto=retract_pos;
  waitForDisposition();

  while(repeats-->0){
    
    steps_Z_goto = bottom_hole;
    waitForDisposition();
    dwell_timer = millis() + dwell_time;
    while(dwell_timer > millis()) doDisplay();

    byte zd=stepperZ_delay;
    stepperZ_delay = Z_max_speed;
    steps_Z_goto = retract_pos;
    waitForDisposition();
    stepperZ_delay = zd;
    
    }
return 0;
}

//Peck mode -  G83 XYZ R-retractpos Pdwelltime Qincrperpeck  Lnum repeats
//review: incpp and repeats parameters
int doPeck(long bottom_hole,long retract_pos, int dwell_time, int incpp, int repeats){
  long delta = ceil(abs(bottom_hole)-abs(retract_pos));
  long steps_per_peck = ceil(delta / repeats);
  Serial.print("Peck-BH:");Serial.print(bottom_hole);Serial.print(" St:");Serial.print(steps_per_peck);
  Serial.print(" incpp:");Serial.print(incpp);Serial.print(" Delta:");Serial.print(delta);
  Serial.print(" rep:");Serial.println(repeats);
  while(repeats-->1){
        Serial.print("Peck drill to Z:");Serial.println(bottom_hole+(steps_per_peck*repeats));
        doDrill(bottom_hole+(steps_per_peck*repeats),retract_pos,dwell_time,0);
        }
  doDrill(bottom_hole,retract_pos,dwell_time,0);
  
return 0;
}


void sendHelp(){
       Serial.print(F("$$ (view Grbl settings)\r\n"
                        "$# (view # parameters)\r\n"
                        "$G (view parser state)\r\n"
                        "$I (view build info)\r\n"
                        "$N (view startup blocks)\r\n"
                        "$x=value (save Grbl setting)\r\n"
                        "$Nx=line (save startup block)\r\n"
                        "$C (check gcode mode)\r\n"
                        "$X (kill alarm lock)\r\n"
                        "$H (run homing cycle)\r\n"
                        "~ (cycle start)\r\n"
                        "! (feed hold)\r\n"
                        "? (current status)\r\n"
                        "ctrl-x (reset Grbl)\r\n"));
  }
  
 //fake grbl boot message, using the F macro to have the text stored in progmem
 //might be relocated to eeprom in future
  void sendVersion(){ 
     Serial.print(F("MicroMill ")); Serial.print(VERSION); 
     Serial.println(F(" - emulating Grbl comminucation protocol"));
     Serial.println(F("Grbl 0.9j")); //needed by ugs and orthers to detect the grbl boot
    }
    
 void sendParameters(byte n){ //semi fake grbl parameters
  
 #ifdef GRBL_FAKE_FULL //no real need to post features that make no sense here, saves us some byte
      if(n==0||n==255) Serial.println(F("$0=10 (step pulse, usec)")); //doesnt apply
      if(n==1||n==255) Serial.println(F("$1=25 (step idle delay, msec)")); //dto. 
      if(n==2||n==255) Serial.println(F("$2=0 (step port invert mask:00000000)")); //dto
      if(n==3||n==255) Serial.println(F("$3=0 (dir port invert mask:00000000)"));  //dto
      if(n==4||n==255) Serial.println(F("$4=0 (step enable invert, bool)")); //always 0
      if(n==5||n==255) Serial.println(F("$5=0 (limit pins invert, bool)")); //always 0
      if(n==6||n==255) Serial.println(F("$6=0 (probe pin invert, bool)")); //always 0
      if(n==10||n==255) Serial.println(F("$10=3 (status report mask:00000011)")); //always 3
      if(n==11||n==255) Serial.println(F("$11=0.010 (junction deviation, mm)")); //no junction 
#endif
      if(n==12||n==255) Serial.println(F("$12=0.02 (arc tolerance, mm)")); //depends on arc size, 1 deg lines currenly
      if(n==13||n==255) Serial.println(F("$13=0 (report inches, bool)")); //always 0
      if(n==20||n==255) Serial.println(F("$20=1 (soft limits, bool)")); //soft limits are applicable
      if(n==21||n==255) Serial.println(F("$21=1 (hard limits, bool)")); //hard limits are provided
      if(n==22||n==255) { Serial.print(F("$22="));Serial.print(HOMING);Serial.println(F(" (homing cycle, bool)"));} //see homing 
#ifdef GRBL_FAKE_FULL
      if(n==23||n==255) Serial.println(F("$23=0 (homing dir invert mask:00000000)")); //always 0
      if(n==24||n==255) Serial.println(F("$24=13.320 (homing feed, mm/min)")); //hardcoded
      if(n==25||n==255) Serial.println(F("$25=100.000 (homing seek, mm/min)")); //hardcoded
      if(n==26||n==255) Serial.println(F("$26=250 (homing debounce, msec)")); //not applicable 
      if(n==27||n==255) Serial.println(F("$27=1.000 (homing pull-off, mm)")); //hardcoded
#endif
      if(n==100||n==255) { Serial.print(F("$100="));Serial.print((float)steps_X_perunit,3);Serial.println(F(" (x, step/mm)")); }
      if(n==101||n==255) { Serial.print(F("$101="));Serial.print((float)steps_Y_perunit,3);Serial.println(F(" (y, step/mm)")); }
      if(n==102||n==255) { Serial.print(F("$102="));Serial.print((float)steps_Z_perunit,3);Serial.println(F(" (z, step/mm)")); }
      if(n==110||n==255) { Serial.print(F("$110="));Serial.print(MAX_X_FEEDRATE);Serial.println(F(" (x max rate, mm/min)")); }
      if(n==111||n==255) { Serial.print(F("$111="));Serial.print(MAX_Y_FEEDRATE);Serial.println(F(" (y max rate, mm/min)")); }
      if(n==112||n==255) { Serial.print(F("$112="));Serial.print(MAX_Z_FEEDRATE);Serial.println(F(" (z max rate, mm/min)")); }
#ifdef GRBL_FAKE_FULL
      if(n==120||n==255) Serial.println(F("$120=1.000 (x accel, mm/sec^2)")); //not applicable - fixed speeds
      if(n==121||n==255) Serial.println(F("$121=1.000 (y accel, mm/sec^2)")); //not applicable
      if(n==122||n==255) Serial.println(F("$122=1.000 (z accel, mm/sec^2)")); //not applicable
#endif
      setStepsPerUnit(MM); //force units to be mm
      if(n==130||n==255) { Serial.print(F("$130="));Serial.print((float)X_max_steps/(float)steps_X_perunit,3);Serial.println(F(" (x max travel, mm)")); }
      if(n==131||n==255) { Serial.print(F("$131="));Serial.print((float)Y_max_steps/(float)steps_Y_perunit,3);Serial.println(F(" (y max travel, mm)")); }
      if(n==132||n==255) { Serial.print(F("$132="));Serial.print((float)Z_max_steps/(float)steps_Z_perunit,3);Serial.println(F(" (z max travel, mm)")); }
      setStepsPerUnit(unit_is_mm); //reset units to what it was
}
 
void sendStatus(){
     Serial.print("<"); 
     if(machine_idle==TRUE) Serial.print("Idle,");
                  else      Serial.print("Busy,");
     Serial.print("MPos:"); Serial.print(MPOS_X,4);
          Serial.print(",");Serial.print(MPOS_Y,4);
          Serial.print(",");Serial.print(MPOS_Z,4);
          Serial.print(",");
        Serial.print("WPos:"); Serial.print(WPOS_X,4);
          Serial.print(",");Serial.print(WPOS_Y,4);
          Serial.print(",");Serial.print(WPOS_Z,4);
        Serial.println(">");   
      status_request = FALSE;
  }
  
void sendOK(){
  Serial.println(F("ok"));
  }
void sendError(const __FlashStringHelper* msg){
  Serial.print(F("error:["));
  Serial.println(msg);
}
  
//changes mode, current mode is shown in $G
void checkMode(){
  steppers_enable = !steppers_enable;
  }

//$G print out some status data that isnt covered elsewhere
 void sendSettings(){
    printUnit();
    Serial.print(F("FR "));
    Serial.print(F(" X:"));Serial.print(calculateFeedrate(X_work_speed, steps_X_perunit));
    Serial.print(F(" Y:"));Serial.print(calculateFeedrate(Y_work_speed, steps_Y_perunit));
    Serial.print(F(" Z:"));Serial.println(calculateFeedrate(Z_work_speed, steps_Z_perunit));
    Serial.print(F("XY coords "));
    if(dimension_absolute) Serial.println(F("abs"));
      else                 Serial.println(F("rel"));
    Serial.print(F("CP coords "));
    if(cp_absolute)        Serial.println(F("abs"));
      else                 Serial.println(F("rel"));
    if(toolchange_procedure) Serial.println(F("toolchange active"));
    Serial.print(F("Check mode "));
    if(!steppers_enable) Serial.println(enable_string);
      else               Serial.println(disable_string);
    if(tool_auto_off==TRUE) Serial.println(F("Tool auto on"));
    if(translate_Z_offset != 0) { Serial.print(F("Z Translation:")); Serial.println(translate_Z_offset);}

    
  }

