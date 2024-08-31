

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ***** Uncomment to use the PinChangeInterrupts in stead of External Interrupts *****
// #define PIN_CHANGE_INT
//
// ******** REMOVE THE "//" IN THE NEXT LINE UNLESS YOU WANT ALL CV'S RESET ON EVERY POWER-UP
// #define DECODER_LOADED
//
// ******** REMOVE THE "//" IN THE NEXT LINE IF YOU WANT TO RUN A TEST DURING SETUP SEQUENCES
#define TESTRUN
//
// ******** REMOVE THE "//" IN THE FOLLOWING LINE TO SEND DEBUGGING INFO TO THE SERIAL OUTPUT
#define _DEBUG_
//


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* don't print warnings of unused functions from here */
#pragma GCC diagnostic push

#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"


#include <Arduino.h>   /*  Needed for C++ conversion of INOfile.  */

#include <avr/wdt.h>   /*  Needed for automatic reset functions.  */

#include <avr/io.h>    /*  AVR device-specific  IO  definitions.  */

#include <NmraDcc.h>   /*  Needed for computing the DCC signals.  */

NmraDcc     Dcc ;
DCC_MSG  Packet ;

#define This_Decoder_Address     40
#define NMRADCC_SIMPLE_RESET_CV 251
#define NMRADCC_MASTER_RESET_CV 252
#define FunctionPinDcc            2  /* Inputpin DCCsignal */

uint8_t thisDecoderdirection = DCC_DIR_FWD;
uint8_t prevDecoderdirection = DCC_DIR_REV;

#define baseAddressServos1  0x40   //  Base address of the SERVOS part
#define baseAddressJuicer1  0x20   //  Base address of the JUICER part
#define baseAddressSignal1  0x21   //  Base address of the SIGNAL part
#define baseAddressSwitch1  0x24   //  Base address of the SWITCH part
#define baseAddressSwitch2  0x26   //  Base address of the SWITCH part


#pragma GCC diagnostic pop
/* don't print warnings of unused functions till here */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* printing of debug and test options */

#if defined( _DEBUG_ ) || defined(TESTRUN)
   #define _PP( a ) Serial.print(     a );
   #define _PL( a ) Serial.println(   a );
   #define _2P(a,b) Serial.print(  a, b );
   #define _2L(a,b) Serial.println(a, b );
#else
   #define _PP(  a)
   #define _PL(  a)
   #define _2P(a,b)
   #define _2L(a,b)
#endif


char bomMarker  =   '<' ;    /*  Begin of message marker.  */
char eomMarker  =   '>' ;    /*  End   of message marker.  */
char commandString[ 32] ;    /*  Max length for a buffer.  */
char sprintfBuffer[ 32] ;    /*  Max length for a buffer.  */
bool foundBom   = false ;    /*  Found begin of messages.  */
bool foundEom   = false ;    /*  Found  end  of messages.  */

volatile unsigned long currentMillis; /*  used for the timekeeping  */


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  *****  small macro used to create all the timed events  *****
#define runEvery( n ) for ( static unsigned long lasttime; millis() - lasttime > ( unsigned long )( n ); lasttime = millis() )


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// =====================================================================

// servo limits min 110, max 665 
const int SERVOMIN = 125;   // min pulse value for full left servo throw (zero degrees)
const int SERVOMAX = 625;   // max pulse value for full right servo throw (180 degrees)
const int SERVOOFF = 4096;  // pulse value that turns the servo pin off (wear and tear)


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(baseAddressServos1);

int servoPos = 90;
int servoLft = 90;
int servoRgt = 90;
bool thrown  = false; // turnout is in normal position

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pwm.begin();

  pwm.setPWMFreq(60);
  pwm.setPWM(0, 0, setServoAngle(90));   //set servo to center (90 degrees)

}

void loop() {
  // put your main code here, to run repeatedly:
  char inChar = (char)Serial.read();
	 
 if (inChar == 'w') {  //center the servo
    servoPos = 90;
    pwm.setPWM(0, 0, setServoAngle(servoPos));
    Serial.println("Set servo center ...");
 }
 
  if (inChar == 'q') {  //adjust servo left 5 degrees at a time
    servoPos = servoPos + 5;
	  if (servoPos > 180) {servoPos = 180;} // prevent going beyond 180
    pwm.setPWM(0, 0, setServoAngle(servoPos));
    Serial.println("New servo left ...");
	  servoLft = servoPos;
 }

 if (inChar == 'e') { //adjust servo right 5 degrees at a time
    servoPos = servoPos - 5;
    if (servoPos < 0) {servoPos = 0;} // prevent overdriving servo right
    pwm.setPWM(0, 0, setServoAngle(servoPos));
    Serial.println("New servo right ...");
	  servoRgt = servoPos;
 }
 
 if (inChar == 't') {  //throw the turnout the other way from where it is
    if (thrown) {
		pwm.setPWM(0, 0, setServoAngle(servoRgt));
    Serial.println("Throw right ... ");
	  } else {
		pwm.setPWM(0, 0, setServoAngle(servoLft));
    Serial.println("Throw left ... ");
	  }
	thrown = !thrown; // flip the turnout direction
 }
 

}

 /*
 * setServoAngle(int ang)
 * gets angle in degrees and returns matching pulse value
 */
int setServoAngle(int ang) {
   int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX); 
	 Serial.print("...Angle: "); Serial.print(ang); 
	 Serial.print(" / Pulse: "); Serial.println(pulse);
  return pulse;
}



/* ******************************************************************************* */
      // 0 = Off, 1 = On, 2 = Blink Off, 3 = Blink On, 4 = Fade Off, 5 = Fade On  ***
/* **********************************************************************************

   <0>   all outputs: Off
   <1>   all outputs: On
   <2>   all outputs: Blink Off
   <3>   all outputs: Blink On
   <4>   all outputs: Fade Off
   <5>   all outputs: Fade On

   <C>   clear everything: Factory Default
   <D>   dumps everything: to Serial.Print

   <F>   controls mobile engine decoder functions F0-F12: <f x y>
   <f>   lists all funtions and settings for all outputs: <F>

   <M>   list the available SRAM on the chip

   <R>   reads a configuration variable: <R x>
   <W>   write a configuration variable: <W x y>

*/
void displayText()
{
   _PL("");    // An extra empty line for better understanding
   _PL(F("Put in one of the following commands: "          ));
   _PL(F("------------------------------------------------"));
   _PL(F("<C>   clear everything: Factory Default"         ));
   _PL(F("<D>   prints CV values: to your monitor"         ));
   _PL("");
   _PL(F("<F>   control decoder functions F0-F12: <Fx>"    ));
   _PL("");
   _PL(F("<R>   reads a configuration variable: <R x>"     ));
   _PL(F("<W>   write a configuration variable: <W x y>"   ));
   _PL(F("----------------------------------------------- "));
   _PL(F("* include '<', '>' and spaces in your command * "));
   _PL("");    // An extra empty line for better understanding
}


/* ******************************************************************************* */
/*
.*    SerialEvent occurs whenever a new data comes in the hardware serial RX. This
.*    routine is run between each time loop() runs, so using delay inside loop can
.*    delay response.  Multiple bytes may be available and be put in commandString
.*/
void serialEvent() {
   while (Serial.available())
   {
      char inChar = (char)Serial.read(); // get the new byte

      if (inChar == bomMarker)       // start of new command
      {
         sprintf(commandString, "%s%c", ""           ,  inChar);
         foundBom = true;
      }
      else if (inChar == eomMarker)   // end of this command
      {
         foundEom = true;
         sprintf(commandString, "%s%c", commandString,  inChar);
      }
      else if (strlen(commandString) <  16) // put it all in
      {
         sprintf(commandString, "%s%c", commandString,  inChar);
         /* if comandString still has space, append character just read from serial line
         otherwise, character is ignored, (but we'll continue to look for '<' or '>') */
      }
   }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

