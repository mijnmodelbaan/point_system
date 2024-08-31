

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


#define baseAddressServos1  0x40   //  Base address of the SERVOS part
#define baseAddressJuicer1  0x20   //  Base address of the JUICER part
#define baseAddressSignal1  0x21   //  Base address of the SIGNAL part
#define baseAddressSwitch1  0x24   //  Base address of the SWITCH part
#define baseAddressSwitch2  0x26   //  Base address of the SWITCH part


#include <Arduino.h>   /*  Needed for C++ conversion of INOfile.  */

#include <avr/wdt.h>   /*  Needed for automatic reset functions.  */

#include <avr/io.h>    /*  AVR device-specific  IO  definitions.  */

#include <Wire.h>      /*  The file for the TWI (I2C) interface.  */


#include <NmraDcc.h>   /*  Needed for computing the DCC signals.  */

NmraDcc     Dcc ;
DCC_MSG  Packet ;

#define This_Decoder_Address     40
#define NMRADCC_SIMPLE_RESET_CV 251
#define NMRADCC_MASTER_RESET_CV 252
#define FunctionPinDcc            2  /* Inputpin DCCsignal */

uint8_t thisDecoderdirection = DCC_DIR_FWD;
uint8_t prevDecoderdirection = DCC_DIR_REV;


#include <Adafruit_PWMServoDriver.h>   /*  Needed for driving the servos */

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(baseAddressServos1);

const int SERVOMIN =  125;   // min pulse value for full left servo throw (zero degrees)
const int SERVOMAX =  625;   // max pulse value for full right servo throw (180 degrees)
const int SERVOOFF = 4096;   // pulse value that turns the servo pin off (wear and tear)

int servoPos = 90;
int servoLft = 90;
int servoRgt = 90;
bool thrown  = false; // turnout is in normal position


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


struct CVPair
{
   uint16_t    CV;
   uint8_t  Value;
};


CVPair FactoryDefaultCVs [] =
{
   /*  this is the primary short decoder address  */
   { CV_MULTIFUNCTION_PRIMARY_ADDRESS,      (( This_Decoder_Address >> 0 ) & 0x7F ) +   0 },

   /*  these two CVs define the Long DCC Address  */
   { CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, (( This_Decoder_Address >> 8 ) & 0x7F ) + 192 },
   { CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, (( This_Decoder_Address >> 0 ) & 0xFF ) +   0 },

   /* ONLY uncomment 1 CV_29_CONFIG line below as approprate -      DEFAULT IS SHORT ADDRESS  */
// { CV_29_CONFIG,                0},                        // Short Address 14     Speed Steps
   { CV_29_CONFIG, CV29_F0_LOCATION},                        // Short Address 28/128 Speed Steps
// { CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION},  // Long  Address 28/128 Speed Steps

   { NMRADCC_MASTER_RESET_CV,     0},

   {  40,   1},   /*  0 = disable ALL, 1 = enable ALL ( >> at start up or reset)  F0  */
   {  41, 255},
   {  42, 255},
   {  43, 155},
   {  44,   1},   /*  dimmingfactor multiplier  */
   {  45,  10},   /*  blinking  ON  multiplier  */
   {  46,  10},   /*  blinking OFF  multiplier  */
   {  47,  10},   /*  fade up time  multiplier  */
   {  48,  10},   /*  fade down     multiplier  */
   {  49,   0},   /*    */

/*   0  disabled
     1  normal on/off
     2  blinking
     4  fading 
     8  forward 
    16 backward
    */

   {  50,   5},  /*  0 = disabled - 1+ = enabled >> NEO   F1  */
   {  51, 255},  /*  color value  red  channel  */
   {  52, 255},  /*  color value green channel  */
   {  53, 155},  /*  color value blue  channel  */
   {  54, 127},  /*  dimming factor of channel  */
   {  55,   0},  /*  blinking  ON time setting  */
   {  56,   0},  /*  blinking OFF time setting  */
   {  57,  40},  /*  fade-up   time    setting  */
   {  58,  60},  /*  fade-down time    setting  */
   {  59,   0},  /*    */


   { 251,   0},  /*  NMRADCC_SIMPLE_RESET_CV  */
   { 252,   0},  /*  NMRADCC_MASTER_RESET_CV  */
};

uint8_t FactoryDefaultCVIndex = sizeof( FactoryDefaultCVs ) / sizeof( CVPair );
void notifyCVResetFactoryDefault()
{
      // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset.
      // Flag to the loop() function a reset to FactoryDefaults has to be done.
        FactoryDefaultCVIndex = sizeof( FactoryDefaultCVs ) / sizeof( CVPair );
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {




#if defined(_DEBUG_) || defined(TESTRUN)

  Serial.begin(115200);

  while (!Serial)
  {
    ; // wait for Serial port to connect. Needed for native USB port.
  }

  Serial.flush();   // Wait for all the rubbish to finish displaying.

  while (Serial.available() > 0)
  {
    Serial.read(); // Clear the input buffer to get 'real' inputdata.
  }

  _PL(F( "-------------------------------------" ));

#endif



  servos.begin();

  servos.setPWMFreq(60);
  servos.setPWM( 0, 0, setServoAngle(90) );   //set servo to center (90 degrees)




#if defined(_DECODER_LOADED_)

  if ( Dcc.getCV( NMRADCC_MASTER_RESET_CV ) == NMRADCC_MASTER_RESET_CV ) 
  {

#endif

    _PL(F( "wait for the copy process to finish.." ));

    FactoryDefaultCVIndex =  sizeof( FactoryDefaultCVs ) / sizeof( CVPair );

    for ( int i = 0; i < FactoryDefaultCVIndex; ++i )
    {
      Dcc.setCV( FactoryDefaultCVs[ i ].CV, FactoryDefaultCVs[ i ].Value );
    }

#if defined(_DECODER_LOADED_)

  }

#endif


  //  Call the DCC 'pin' and 'init' functions to enable the DCC Receivermode
  Dcc.pin( digitalPinToInterrupt( FunctionPinDcc ), FunctionPinDcc, false );
  Dcc.init( MAN_ID_DIY, 201, FLAGS_MY_ADDRESS_ONLY, 0 );      delay( 1000 );
  //c.initAccessoryDecoder( MAN_ID_DIY,  201, FLAGS_MY_ADDRESS_ONLY,    0 );

  Dcc.setCV( NMRADCC_SIMPLE_RESET_CV, 0 );  /*  Reset the just_a_reset CV */


  displayText(); /* Shows the standard text if applicable */


  interrupts();  /* Ready to rumble....*/
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {



  char inChar = (char)Serial.read();
	 
 if (inChar == 'w') {  //center the servo
    servoPos = 90;
    servos.setPWM(0, 0, setServoAngle(servoPos));
    Serial.println("Set servo center ...");
 }
 
  if (inChar == 'q') {  //adjust servo left 5 degrees at a time
    servoPos = servoPos + 5;
	  if (servoPos > 180) {servoPos = 180;} // prevent going beyond 180
    servos.setPWM(0, 0, setServoAngle(servoPos));
    Serial.println("New servo left ...");
	  servoLft = servoPos;
 }

 if (inChar == 'e') { //adjust servo right 5 degrees at a time
    servoPos = servoPos - 5;
    if (servoPos < 0) {servoPos = 0;} // prevent overdriving servo right
    servos.setPWM(0, 0, setServoAngle(servoPos));
    Serial.println("New servo right ...");
	  servoRgt = servoPos;
 }
 
 if (inChar == 't') {  //throw the turnout the other way from where it is
    if (thrown) {
		servos.setPWM(0, 0, setServoAngle(servoRgt));
    Serial.println("Throw right ... ");
	  } else {
		servos.setPWM(0, 0, setServoAngle(servoLft));
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

