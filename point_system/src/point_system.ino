

//  https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/tree/master
//  https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
//  https://github.com/adafruit/Adafruit_PCF8574

//  https://github.com/MicroBahner/MobaTools/tree/master
//  https://arduino.stackexchange.com/questions/91149/how-to-determine-the-minimum-time-for-a-servo-to-reach-its-destination


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ***** Uncomment to use the PinChangeInterrupts in stead of External Interrupts *****
// #define PIN_CHANGE_INT
//
// ******** REMOVE THE "//" IN THE NEXT LINE UNLESS YOU WANT ALL CV'S RESET ON EVERY POWER-UP
#define DECODER_LOADED
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

#pragma GCC diagnostic error "-Wsign-compare"
#pragma GCC diagnostic error "-Wunused-function"
#pragma GCC diagnostic error "-Wunused-variable"
// #pragma GCC diagnostic error "-Wsomething"


#define baseAddressServos1  0x40   /*  Base address of the SERVOS part  */
#define baseAddressJuicer1  0x20   /*  Base address of the JUICER part  */
#define baseAddressSignal1  0x21   /*  Base address of the SIGNAL part  */
#define baseAddressSwitch1  0x24   /*  Base address of the SWITCH part  */
#define baseAddressSwitch2  0x26   /*  Base address of the SWITCH part  */


#include <Arduino.h>         /*  Needed for C++ conversion of INOfile.  */

#include <avr/wdt.h>         /*  Needed for automatic reset functions.  */

#include <avr/io.h>          /*  AVR device-specific  IO  definitions.  */

#include <avr/interrupt.h>   /*  Might be necessary, for ISR handling.  */

#include <Wire.h>            /*  The file for the TWI (I2C) interface.  */


/* -------------------------------------------------------------- */
#include <NmraDcc.h>   /*  needed for computing the DCC signals   */

NmraDcc     Dcc ;
DCC_MSG  Packet ;

#define This_Decoder_Address     40
#define NMRADCC_SIMPLE_RESET_CV 251
#define NMRADCC_MASTER_RESET_CV 252
#define FunctionPinDcc            2  /* Inputpin DCCsignal */

// uint8_t thisDecoderdirection = DCC_DIR_FWD;
// uint8_t prevDecoderdirection = DCC_DIR_REV;


/* --------------------------------------------------------------------- */
#include <Adafruit_PWMServoDriver.h>  /*  needed for driving the servos  */

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver( baseAddressServos1 );

#define  SERVOMIN   150   // min pulse value for full left servo throw (zero degrees)
#define  SERVOMAX   600   // max pulse value for full right servo throw (180 degrees)
#define  SERVOOFF  4096   // pulse value that turns the servo pin off (wear and tear)
#define  USEC_MIN   600   // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define  USEC_MAX  2400   // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define  SERVOFRQ    60   // Digital servos run at ~60Hz updates, analog servos at ~50Hz

/* WATCH IT: pins are numbered from 0 to 15 (so NOT from 1 to 16)  */


/* -------------------------------------------------------------- */
#include <Adafruit_MCP23X17.h> /* used by JUICER and SIGNAL driving  */

Adafruit_MCP23X17 juicer;
Adafruit_MCP23X17 signal;

/* WATCH IT: pins are numbered from 0 to 15 (so NOT from 1 to 16)  */


/* -------------------------------------------------------------- */
#include <Adafruit_PCF8574.h>  /* used by both SWITCH drivers  */

Adafruit_PCF8574  inputa;
Adafruit_PCF8574  inputb;

/* WATCH IT: pins are numbered from 0 to 7 (so NOT from 1 to 8)  */


#pragma GCC diagnostic pop
/* don't print warnings of unused functions till here */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* printing of debug and test options */

#if defined( _DEBUG_ ) || defined(TESTRUN)
  #define _PP( a ) Serial.print(      a );
  #define _PL( a ) Serial.println(    a );
  #define _2P(a,b) Serial.print(   a, b );
  #define _2L(a,b) Serial.println( a, b );
#else
  #define _PP(  a)
  #define _PL(  a)
  #define _2P(a,b)
  #define _2L(a,b)
#endif


// char bomMarker  =   '<' ;    /*  Begin of message marker.  */
// char eomMarker  =   '>' ;    /*  End   of message marker.  */
// char commandString[ 32] ;    /*  Max length for a buffer.  */
// bool foundBom   = false ;    /*  Found begin of messages.  */

char    sprintfBuffer[ 64 ] ;    /*  Max length for a buffer.  */
bool    foundEom    = false ;    /*  Found  end  of messages.  */
String  commandString =  "" ;    /*  String to hold incoming   */
// uint8_t cvCheck = 0;

volatile unsigned long currentMillis; /*  used for the timekeeping  */


/* /////////////////////////////////////////////////////////////////////////////////////////////////// */

/* //////////////////////////////////////////////////////////////////////////////////////////////////////
/*  *****  small macro used to create timed events  *****   */
#define runEvery( n ) for ( static unsigned long lasttime; millis() - lasttime > ( unsigned long )( n ); lasttime = millis() )
/* example:  runEvery( time ) { first command; second command; }  checks if the time (in millis) has passed and if so executes

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */


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

  /*  ONLY uncomment 1 CV_29_CONFIG line below as approprate -      DEFAULT IS SHORT ADDRESS  */
// { CV_29_CONFIG,                0},                        // Short Address 14     Speed Steps
  { CV_29_CONFIG, CV29_F0_LOCATION},                         // Short Address 28/128 Speed Steps
// { CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION},  // Long  Address 28/128 Speed Steps

  { NMRADCC_MASTER_RESET_CV,     0},

  {  40,   2},  /*  mover Lft for all servos   */
  {  41,   2},  /*  mover Rgt for all servos   */
  {  42,  16},  /*  total number of servos     */
  {  49,   0},  /*                             */


                /*  SERVO  1                   */
  {  50,  90},  /*  angle Lft for this servo   */
  {  51,  90},  /*  angle Rgt for this servo   */
  {  52,   0},  /*  0 SERVOS standard Lft      */
  {  53,   0},  /*  0 JUICER standard Off      */
  {  54,   0},  /*  0 SIGNAL standard Off      */
  {  55,  10},  /*  0 SWITCH is Output         */

                /*  SERVO  2                   */
  {  60,  90},  /*  angle Lft for this servo   */
  {  61,  90},  /*  angle Rgt for this servo   */
  {  62,   0},  /*  0 SERVOS standard Lft      */
  {  63,   0},  /*  0 JUICER standard Off      */
  {  64,   0},  /*  0 SIGNAL standard Off      */
  {  65,  10},  /*  0 SWITCH is Output         */

                /*  SERVO  3                   */
  {  70,  90},  /*  angle Lft for this servo   */
  {  71,  90},  /*  angle Rgt for this servo   */
  {  72,   0},  /*  0 SERVOS standard Lft      */
  {  73,   0},  /*  0 JUICER standard Off      */
  {  74,   0},  /*  0 SIGNAL standard Off      */
  {  75,  10},  /*  0 SWITCH is Output         */



                /*  SERVO 13                   */
  { 170,  90},  /*  angle Lft for this servo   */
  { 171,  90},  /*  angle Rgt for this servo   */
  { 172,   0},  /*  0 SERVOS standard Lft      */
  { 173,   0},  /*  0 JUICER standard Off      */
  { 174,   0},  /*  0 SIGNAL standard Off      */
  { 175,  10},  /*  0 SWITCH is Output         */

                /*  SERVO 14                   */
  { 180,  90},  /*  angle Lft for this servo   */
  { 181,  90},  /*  angle Rgt for this servo   */
  { 182,   0},  /*  0 SERVOS standard Lft      */
  { 183,   0},  /*  0 JUICER standard Off      */
  { 184,   0},  /*  0 SIGNAL standard Off      */
  { 185,  10},  /*  0 SWITCH is Output         */

                /*  SERVO 15                   */
  { 190,  90},  /*  angle Lft for this servo   */
  { 191,  90},  /*  angle Rgt for this servo   */
  { 192,   0},  /*  0 SERVOS standard Lft      */
  { 193,   0},  /*  0 JUICER standard Off      */
  { 194,   0},  /*  0 SIGNAL standard Off      */
  { 195,  10},  /*  0 SWITCH is Output         */

                /*  SERVO 16                   */
  { 200,  90},  /*  angle Lft for this servo   */
  { 201,  90},  /*  angle Rgt for this servo   */
  { 202,   0},  /*  0 SERVOS standard Lft      */
  { 203,   0},  /*  0 JUICER standard Off      */
  { 204,   0},  /*  0 SIGNAL standard Off      */
  { 205,  10},  /*  0 SWITCH is Output         */


  { 250,   0},  /*  MASTER SWITCH OVER-RULE    */
  { 251,   0},  /*  NMRADCC_SIMPLE_RESET_CV    */
  { 252,   0},  /*  NMRADCC_MASTER_RESET_CV    */
  { 253,   0},  /*                             */

};

uint8_t FactoryDefaultCVIndex = sizeof( FactoryDefaultCVs ) / sizeof( CVPair );
void    notifyCVResetFactoryDefault()
{
      // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset.
      // Flag to the loop() function a reset to FactoryDefaults has to be done.
        FactoryDefaultCVIndex = sizeof( FactoryDefaultCVs ) / sizeof( CVPair );
};



/* /////////////////////////////////////////////////////////////////////////////////////////////////// */

/* //////////////////////////////////////////////////////////////////////////////////////////////////////
  this  */


/*

what we need:

typedef struct {
angle Lft
angle Rgt
moving to 90 (center)
moving on to Lft/Rgt  ??? ( angle value   or   travel direction )
wait time for JUICER
current state

} TurnoutStruct;

real number of servos (16) is in CV 42

TurnoutStruct TurnoutSettings [ 16 ] =     fill the table during setup
  { Lft, Rgt, t/f, Lft/Rgt/t/f, millis },  /*  Turnout  0  //


calculate the table settings in setup and after receiving a CV
set all the turnouts correctly in setup and after receiving a command


*/


struct QUEUE
{
  uint8_t   angleLft      =     90 ;  /*  angle to go full  Left  */
  uint8_t   angleRgt      =     90 ;  /*  angle to go full Right  */
  bool      moveToCenter  =  false ;  /*  makes a move to Center  */
  uint8_t   moveFromCntr  =     90 ;  /*  moving to Left / Right  */
  uint16_t  juicerTime    =   1255 ;  /*  a juicer needs to wait  */
  bool      currentState  =  false ;  /*  do we need attention ?  */
};
QUEUE volatile *turnout_queue = new QUEUE[ 16 + 1 ];   /*  16 turnouts max [15:0] + 1 dummy  */

volatile byte currentTurnout = 16;  /*  default turnout updating commands  */
volatile byte positionUpdate =  2;  /*  turnout position update (x, c, v)  */

/* /////////////////////////////////////////////////////////////////////////////////////////////////// */

/* //////////////////////////////////////////////////////////////////////////////////////////////////////
  this is the place where everything is set up  */

void setup()
{
  noInterrupts(); /*  cli() can also be used  */


/* =========================================================================================
  the following lines set the A0 to A3 lines as input (with a pull-up) for the interrupts */
  DDRC   = DDRC   & 0b11110000;  /*  Set bits A[3-0] as inputs, leave the rest as-is      */
  PORTC  = PORTC  | 0b00001111;  /*  Switch bits A[3-0] to PULL-UP, leave the rest as-is  */

  PCICR  = PCICR  | 0b00000010;  /*  Set PCIE1, meaning enable the PCINT[14:8] interrupts */
  PCMSK1 = PCMSK1 | 0b00001111;  /*  A0 = PCINT8, A1 = PCINT9, A2 = PCINT10, A3 = PCINT11 */
  PCIFR  = PCIFR  | 0b00000010;  /*  Clear interrupt for all interrupts from port A[7:0]  */

ADCSRA = 0;  /*  disable analog conversions  */


  interrupts();  /* sei() can also be used   */


  Wire.begin();   /*  start the I2C / TWI library - mandatory!  */

/* =============================================
  if you want to really speed Wire-stuff up
  you can go into 'fast 400khz I2C' mode,
  but some i2c devices dont like this so if
  you're sharing the bus, watch out for this! */
  Wire.setClock(400000);

/*  TODO:  make setClock selectable via a CV ???   */





/*  check if starting the Serial Interface is needed and do so if yes  */
#if defined(_DEBUG_) || defined(TESTRUN)

  // Serial.begin( 9600, SERIAL_8N1 ); /*  ( 115200, SERIAL_8N1 );  */
  Serial.begin( 115200, SERIAL_8N1 );

  while (!Serial)
  {
    ; // wait for Serial port to connect. Needed for native USB port.
  }

  Serial.flush();   // Wait for all the rubbish to finish displaying.

  while (Serial.available() > 0)
  {
    Serial.read(); // Clear the input buffer to get 'real' inputdata.
  }

  Serial.setTimeout( 1000 );  /*  maximum wait time after Cr/Lf /n  */
  commandString.reserve( 32 ); /* reserve 32 bytes for inputString  */

  _PL(F( "-------------------------------------" ));

#endif


/* ===========================================================
  here the existence of several parts of the project is tested
  if you don't have a part in your system, delete that code */

  if ( !servos.begin( ) )
  {
    _PL( "Couldn't find SERVOS  - check and reset" );
    // while ( 1 );
  }

  if ( !juicer.begin_I2C( baseAddressJuicer1 ) )
  {
    _PL( "Couldn't find JUICER  - check and reset" );
    // while ( 1 );
  }

  if ( !signal.begin_I2C( baseAddressSignal1 ) )
  {
    _PL( "Couldn't find SIGNAL  - check and reset" );
    // while ( 1 );
  }

  if ( !inputa.begin( baseAddressSwitch1 ) )
  { 
    _PL( "Couldn't find SWITCH1 - check and reset" ); 
    // while ( 1 );
  }

  if ( !inputb.begin( baseAddressSwitch2 ) )
  { 
    _PL( "Couldn't find SWITCH2 - check and reset" ); 
    // while ( 1 ); 
  }

/* ======================================
   previously we checked if parts existed
   now we're going to initiale them    */

  servos.setPWMFreq( SERVOFRQ );

  for ( uint8_t p = 0; p < 16; ++p )   /*  initialise all 16 as OUTPUT  */
  {
    juicer.pinMode( p, OUTPUT );
    signal.pinMode( p, OUTPUT );
  }

  for ( uint8_t p = 0; p <  8; ++p )   /*  initialise all 2*8 as INPUT  */
  {
    inputa.pinMode( p,  INPUT );
    inputb.pinMode( p,  INPUT );
  }



/*  check for setting the CVs to Factory Default during start and do so if yes  */
#if defined( DECODER_LOADED )

  if ( Dcc.getCV( NMRADCC_MASTER_RESET_CV ) == NMRADCC_MASTER_RESET_CV ) {

#endif

    _PL(F( "wait for the copy process to finish.." ));

    FactoryDefaultCVIndex =  sizeof( FactoryDefaultCVs ) / sizeof( CVPair );

    for ( uint8_t i = 0; i < FactoryDefaultCVIndex; ++i )
    {
      Dcc.setCV( FactoryDefaultCVs[ i ].CV,  FactoryDefaultCVs[ i ].Value );
    }

#if defined( DECODER_LOADED )

  }

#endif


  //  Call the DCC 'pin' and 'init' functions to enable the DCC Receivermode
  Dcc.pin( digitalPinToInterrupt( FunctionPinDcc ), FunctionPinDcc, false );
  Dcc.init( MAN_ID_DIY, 201, FLAGS_MY_ADDRESS_ONLY, 0 );      delay( 1000 );
  //c.initAccessoryDecoder( MAN_ID_DIY,  201, FLAGS_MY_ADDRESS_ONLY,    0 );

  Dcc.setCV( NMRADCC_SIMPLE_RESET_CV, 0 );  /*  Reset the just_a_reset CV */


  displayText( true, false ); /* Shows the standard text if allowed */


  interrupts();  /* Ready to rumble....*/
};


/* /////////////////////////////////////////////////////////////////////////////////////////////////// */

/* //////////////////////////////////////////////////////////////////////////////////////////////////////
   */

/*  TODO:  check this out - maybe usable for JUICER setting during half-time SERVOS  */
// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVOFRQ;   // Digital servos run at ~60 Hz updates  1.000.000 / 50 = 20.000
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution   20.000 / 4096 = 4,882
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us   4,882 * 1.000.000 = 4.882.812,5
  pulse /= pulselength;  //   4.882.812 / 20.000 = 244
  Serial.println(pulse);
  servos.setPWM(n, 0, pulse);
};


/* /////////////////////////////////////////////////////////////////////////////////////////////////// */

/* //////////////////////////////////////////////////////////////////////////////////////////////////////
  this is the place where the interrupts arrive and are checked - then the corresponding flags are set */

#if defined( USE_INTERRUPTS )

volatile bool myEvent_A0 = false;  /*  holds -interrupt from A0- signal  */
volatile bool myEvent_A1 = false;  /*  holds -interrupt from A1- signal  */
volatile bool myEvent_A2 = false;  /*  holds -interrupt from A2- signal  */
volatile bool myEvent_A3 = false;  /*  holds -interrupt from A3- signal  */
volatile byte myKeyPcint =     0;  /*  holds  keypress when interrupted  */

ISR ( PCINT1_vect )  /*  PCINT1_vect: interrupt vector for PORTC A[3:0]  */
{

  myKeyPcint = PINC;  /*  the variable myKeyPcint now holds the activated inputs  */

  if (( myKeyPcint & bit ( 0 )) == 0 )  myEvent_A0 = true;   /*  sets myEvent_A0  */
  if (( myKeyPcint & bit ( 1 )) == 0 )  myEvent_A1 = true;   /*  sets myEvent_A1  */
  if (( myKeyPcint & bit ( 2 )) == 0 )  myEvent_A2 = true;   /*  sets myEvent_A2  */
  if (( myKeyPcint & bit ( 3 )) == 0 )  myEvent_A3 = true;   /*  sets myEvent_A3  */

};

volatile bool myState_A0 = false;  /*  holds previous -interrupt from A0- signal  */
volatile bool myState_A1 = false;  /*  holds previous -interrupt from A1- signal  */
volatile bool myState_A2 = false;  /*  holds previous -interrupt from A2- signal  */
volatile bool myState_A3 = false;  /*  holds previous -interrupt from A3- signal  */
volatile byte myKeyPcold =  &HFF;  /*  hold  old value keypress when interrupted  */

/*  TODO: check if we need previous vars  */

#endif


/* /////////////////////////////////////////////////////////////////////////////////////////////////// */

/* /////////////////////////////////////////////////////////////////////////////////////////////////// */


void loop() {

  Dcc.process();  /*  Call this process for a Dcc connection  */


  currentMillis = millis();  /*  lets keep track of the time  */


#if defined( USE_INTERRUPTS )


  // if ( myEvent_A0 == true ) { _PP( "A0 - " ); _2L( myKeyPcint, BIN ); myEvent_A0 = false; }
  // if ( myEvent_A1 == true ) { _PP( "A1 - " ); _2L( myKeyPcint, BIN ); myEvent_A1 = false; }
  // if ( myEvent_A2 == true ) { _PP( "A2 - " ); _2L( myKeyPcint, BIN ); myEvent_A2 = false; }
  // if ( myEvent_A3 == true ) { _PP( "A3 - " ); _2L( myKeyPcint, BIN ); myEvent_A3 = false; }

  /*  TODO: check if we really need this and if we need debouncing and if it is working correctly  */


  if( myEvent_A0 )
  {
    for ( uint8_t p = 0; p < 8; ++p )
    {
      if (! inputa.digitalRead( p ))
      {
        sprintf( sprintfBuffer, "INPUTA button on GPIO  %2u  pressed!", p );
        _PL( sprintfBuffer );
      }
    }
    myEvent_A0 = false;
  }


  if( myEvent_A1 )
  {
    for ( uint8_t p = 0; p < 8; ++p )
    {
      if ( !inputb.digitalRead( p ) )
      {
        sprintf( sprintfBuffer, "INPUTB button on GPIO  %2u  pressed!", p );
        _PL( sprintfBuffer );
      }
    }
    myEvent_A1 = false;
  }


  if( myEvent_A2 )
  {
    for ( uint8_t p = 0; p < 16; ++p )
    {
      sprintf( sprintfBuffer, "SIGNAL output  %2u  set to ON!", p + 1 );
      _PL( sprintfBuffer );
      signal.digitalWrite( p, HIGH );
    }
    myEvent_A2 = false;
  }


  if( myEvent_A3 )
  {
    for ( uint8_t p = 0; p < 16; ++p )
    {
      sprintf( sprintfBuffer, "JUICER output  %2u  set to ON!", p + 1 );
      _PL( sprintfBuffer );
      juicer.digitalWrite( p, HIGH );
    }
    myEvent_A3 = false;
  }

#endif


  if ( Serial.available() > 0 )  /*  Serial Input needs message-handling  */
  {
    commandString = Serial.readString();

    if ( commandString.length() > 0 )
    {
      foundEom = true;
    }

  } 

  if ( foundEom )  /*  Serial Input needs message-handling  */
  {
    parseCom( commandString );
  }


};



 /*
 * setServoAngle(int ang)
 * gets angle in degrees and returns matching pulse value
 */
int setServoAngle( int ang ) {
   int pulse = map( ang, 0, 180, SERVOMIN, SERVOMAX ); 
	 Serial.print("...Angle: "); Serial.print( ang ); 
	 Serial.print(" / Pulse: "); Serial.println( pulse );
  return pulse;
};


/* /////////////////////////////////////////////////////////////////////////////////////////////////// */

/* //////////////////////////////////////////////////////////////////////////////////////////////////////
   function is doing an automatic reset of the processor after the preScaler time has elapsed          */

void softwareReset( uint8_t preScaler )
{
  wdt_enable( preScaler );

  while( 1 ) {}  // Wait for the prescaler time to expire and do an auto-reset
};


/* /////////////////////////////////////////////////////////////////////////////////////////////////// */

/* /////////////////////////////////////////////////////////////////////////////////////////////////// */






/* **********************************************************************************
***                  menu system for setting and dumping values                   ***
*************************************************************************************

first menu:
  FD         factory defaults all settings --> writes to CVs
  DD         lists all data on serial monitor
  DS x#      default switch (1 to 16) -->  display next menu
  DR xx#     read  data from CV address xx# (0 to 255)
  DW xx# x#  write data x# to CV address xx# (0 to 255)

next menu:
  c = center the default servo
  x = adjust servo  left x degrees at a time
  v = adjust servo right x degrees at a time
  p = invert servos setting for this switch
  j = invert juicer setting for this switch
  s = invert signal setting for this switch
  i = invert switch setting for this switch
  d = display all settings for this turnout
  t = throw the switch (servo, juicer, signal)
  w = write the settings in corresponding CVs --> ask for confirmation
  ? = don't save setting, just go back to first menu

   there could be alse x+ and x-, v+ and v-

*************************************************************************************
  We use a flag to make sure we don't enter the wrong function upon reveiving      */
volatile bool next_menu = false;

void displayText( bool first, bool next )
{

  if( first == true )
  {
    _PL(" ");  // An extra empty line for better understanding
    _PL(F("**  put in one of the following commands: "     ));
    _PL(F("----------------------------------------------" ));
    _PL(F("FD  clear everything: Factory Default"          ));
    _PL(F("DD  prints CV values: to your monitor"          ));
    _PL(" "                                                 );
    _PL(F("DS  set default item number ( 1-16 ): DS x#"    ));
    _PL(" "                                                 );
    _PL(F("DR  reads a configuration variable: DR xx#"     ));
    _PL(F("DW  write a configuration variable: DW xx# x#"  ));
    _PL(F("----------------------------------------------" ));
    _PL(F("**        confirm with  [Enter]  key        **" ));
    _PL(" ");  // An extra empty line for better understanding
  }

  if(  next  == true )
  {
    _PL("");   // An extra empty line for better understanding
    _PL(F("**  put in one of the following commands: "     ));
    _PL(F("----------------------------------------------" ));
    _PL(F("x = adjust servo  left x degrees at a time"     ));
    _PL(F("c = center the default servo (set with DS #)"   ));
    _PL(F("v = adjust servo right x degrees at a time"     ));
    _PL(" "                                                 );
    _PL(F("p = invert  SERVOS  setting for this turnout"   ));
    _PL(F("j = invert  JUICER  setting for this turnout"   ));
    _PL(F("s = invert  SIGNAL  setting for this turnout"   ));
    _PL(F("i = invert  SWITCH  setting for this turnout"   ));
    _PL(" "                                                 );
    _PL(F("d = display changed setting for this turnout"   ));
    _PL(F("t = throw the switch (servo, juicer, signal)"   ));
    _PL(F("w = write the settings  in corresponding CVs"   ));
    _PL(F("----------------------------------------------" ));
    _PL(F("**   any other key discards the settings    **" ));
    _PL("");   // An extra empty line for better understanding
  }

};


/* /////////////////////////////////////////////////////////////////////////////////////////////////// */

/* /////////////////////////////////////////////////////////////////////////////////////////////////// */


//   sensVal = constrain(sensVal, 10, 150);  // limits range of sensor values to between 10 and 150
//   map(value, fromLow, fromHigh, toLow, toHigh)

void parseCom( String commandString )
{
  commandString.trim();    /*  remove spaces at both ends  */
  commandString.toLowerCase();  /*  make it all lowercase  */

  char charOne = '!', charTwo = '!';   /*  first two could be letters  */
  uint16_t intOne = -1, intTwo = -1;   /*  after that come the numbers */

  int deTected = sscanf( commandString.c_str(), "%1c%1c%u%u", &charOne, &charTwo, &intOne, &intTwo );

  memset( sprintfBuffer, 0, sizeof sprintfBuffer );
  sprintf( sprintfBuffer, "charOne: %1c - charTwo: %1c - intOne:  %3u - intTwo:  %3u - deTected:  %3u", charOne, charTwo, intOne, intTwo, deTected );
  _PL( sprintfBuffer );

  if (( next_menu == true ) & ( deTected == 1 ))  /*  single character commands of next menu  */
  {
    switch ( charOne )
    {
      case 'x': /*  x = adjust servo  left x degrees at a time  */
      {
        turnout_queue[ 16 ].angleLft += positionUpdate ;
        if ( turnout_queue[ 16 ].angleLft > 180 ) { turnout_queue[ 16 ].angleLft = 180; }
        servos.setPWM( currentTurnout, 0, setServoAngle( turnout_queue[ 16 ].angleLft ) );
        sprintf( sprintfBuffer, "new servo  left:  %3u ", turnout_queue[ 16 ].angleLft );
        _PL( sprintfBuffer );
        turnout_queue[ 16 ].currentState = true;  /*  something has changed  */
        break;
      }
      case 'c': /*  c = center the default servo (set with DS)  */
      {
        uint8_t servoPos = 90;
        servos.setPWM( currentTurnout, 0, setServoAngle( servoPos ) );
        _PL( "Set servo center ..." );
        break;
      }
      case 'v': /*  v = adjust servo right x degrees at a time  */
      {
        turnout_queue[ 16 ].angleRgt -= positionUpdate ;
        if ( turnout_queue[ 16 ].angleRgt < 0 ) { turnout_queue[ 16 ].angleRgt = 0; } /*  prevent overdrive  */
        servos.setPWM( currentTurnout, 0, setServoAngle( turnout_queue[ 16 ].angleRgt ) );
        sprintf( sprintfBuffer, "new servo right:  %3u ", turnout_queue[ 16 ].angleRgt );
        _PL( sprintfBuffer );
        turnout_queue[ 16 ].currentState = true;  /*  something has changed  */
        break;
      }
      case 'p': /*  p = invert SERVOS setting for this turnout  */
      {

        break;
      }
      case 'j': /*  j = invert JUICER setting for this turnout  */
      {

        break;
      }
      case 's': /*  s = invert SIGNAL setting for this turnout  */
      {

        break;
      }
      case 'i': /*  i = invert SWITCH setting for this turnout  */
      {

        break;
      }
      case 'd': /*  d = display changed setting for this turnout  */
      {

        break;
      }
      case 't': /*  t = throw the switch (servo, juicer, signal)  */
      {

//  if (inChar == 't') {  //throw the turnout the other way from where it is
//     // if (thrown) {
// 		// servos.setPWM(0, 0, setServoAngle(servoRgt));
//     // Serial.println("Throw right ... ");
// 	  // } else {
// 		// servos.setPWM(0, 0, setServoAngle(servoLft));
//     // Serial.println("Throw left ... ");
// 	  // }
// 	// thrown = !thrown; // flip the turnout direction
//  }

        break;
      }
      case 'w': /*  w = write the settings, in corresponding CVs  */
      {

        break;
      }
      default:  /*  discard all changes  */
        break;
    }
  }     /*  end of single character commands  */

  if (( next_menu != true ) & ( deTected == 2 ))  /*  double character commands of first menu  */
  {
    switch ( charOne )
    {
      case 'f': /*    */
      {
        switch ( charTwo )
        {
          case 'd': /*  FD  clear everything: Factory Default  */
          {
            uint8_t cvCheck = Dcc.setCV( NMRADCC_MASTER_RESET_CV, NMRADCC_MASTER_RESET_CV );
            _2L( cvCheck, DEC );
            softwareReset(  WDTO_15MS  );
            break;
          }
          default:  /*  discard all changes  */
            break;
        }
        break;
      }

      case 'd':
      {
        switch ( charTwo )
        {
          case 'd': /*  DD  prints CV values: to your monitor  */
          {
            FactoryDefaultCVIndex = sizeof( FactoryDefaultCVs ) / sizeof( CVPair );

            for ( uint8_t i = 0; i < FactoryDefaultCVIndex; ++i )
            {
              uint8_t cvValue = Dcc.getCV( FactoryDefaultCVs[ i ].CV );

              sprintf( sprintfBuffer, "  cv:  %3u - value:  %3u ", FactoryDefaultCVs[ i ].CV, cvValue );
              _PL( sprintfBuffer );

            }
            break;
          }
          default:  /*  discard all changes  */
            break;
        }
        break;
      }
      default:  /*  discard all changes  */
        break;
    }
  }     /*  end of double character commands  */

  if (( next_menu != true ) & ( deTected == 3 ))  /*  triple character commands of first menu  */
  {
    switch ( charOne )
    {
      case 'd':
      {
        switch ( charTwo )
        {
          case 's': /*  DS  set default item number ( 1-16 ): DS x#  */
          {
            // uint16_t cT;

//           if ( sscanf( com + 3, "%d", &cT ) != 1 ) { return; }  /*  data incorrect  */

//           if ( ( cT & 0b00011111 ) > 16 ) { return; }  /*  data incorrect  */

            intOne = constrain( intOne, 1, 16 );  // limits range of sensor values to 1 till 16


            sprintf( sprintfBuffer, "Default turnout changed to: %d", intOne );

//           currentTurnout = cT;

          // sprintf( sprintfBuffer, "DR  cv:  %3u - value:  %3u ", cv, cvCheck );
            _PL( sprintfBuffer );

            break;
          }
          case 'r': /*  DR  reads a configuration variable: DR xx#  */
          {
            intOne = constrain( intOne, 40, 255 );  /*  limits range of CVs from 40 till 255  */
            uint8_t cvCheck = Dcc.getCV( intOne );

            sprintf( sprintfBuffer, "DR  cv:  %3u - value:  %3u ", intOne, cvCheck );
            _PL( sprintfBuffer );
            break;
          }
          default:
            break;
        }
        break;
      }
      default:
        break;
    }
  }     /*  end of triple character commands  */

  if (( next_menu != true ) & ( deTected == 4 ))  /*  quad character commands of first menu  */
  {
    switch ( charOne )
    {
      case 'd':
      {
        switch ( charTwo )
        {
          case 'w': /*  DW  write a configuration variable: DW xx# x#  */
          {
            /*  TODO: do not use constrain on intOne, but check !!!! else values might be written to the wrong CV  */
            intOne = constrain( intOne, 40, 255 );  /*  limits range of CVs from 40 till 255  */

            intTwo = constrain( intTwo,  0, 255 );  /*  limits CVvalue range from 0 till 255  */

            uint8_t cvCheck = Dcc.setCV( intOne, intTwo );  /*  write action  */

            /*  TODO: calculate all the turnout settings  */

            sprintf( sprintfBuffer, "DW  cv:  %3u - value:  %3u ", intOne, cvCheck );
            _PL( sprintfBuffer );
            break;
          }
          default:
            break;
        }
        break;
      }
      default:  /*  discard all changes  */
        break;
    }
  }     /*  end of quad character commands  */


  //       // default:    /****  DEFAULT FOR THE SWITCH FUNCTION = NO ACTION  ****/
  //       // {
  //       //   displayText( true, false ); // Shows the standard explanation text
  //       //   break;
  //       // }
  //       // break;
  //     }
  //     break;
  //   }


  // if ( ( currentTurnout & 0b00001111 ) != 0)  // if ( !( ( currentTurnout < 0 ) || ( currentTurnout > 15 ) ) )
  // {
  //   turnout_queue[ 16 ].angleLft = turnout_queue[ currentTurnout ].angleLft;
  //   turnout_queue[ 16 ].angleRgt = turnout_queue[ currentTurnout ].angleRgt;
  //   turnout_queue[ 16 ].currentState = false;

  //   _2P( currentTurnout, DEC);
  //   _PP( "  -   " );
  //   _PL( com );

  //   switch ( com[1] )  /*  com[0] = '<' or ' '  */
  //   {

  //     default:    /****  DEFAULT FOR THE SWITCH FUNCTION  ****/
  //     {
  //       displayText( true, false ); // Shows the standard explanation text
  //       break;
  //     }

  //     break;
  //   }
  // }
  // else
  // {
  //   displayText( true, false ); // Shows the standard explanation text
  // }


  /*  let us now reset the whole thing to start anew  */
  foundEom = false;
  commandString.remove( 0 );

};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/************************************************************************************
                        Call-back functions from DCC
************************************************************************************/

/*  TODO:   some of the following functions can be removed after testing with DCC  */


void    notifyDccAccTurnoutBoard (uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower)
{
            _PL("notifyDccAccTurnoutBoard");
}

void    notifyDccAccTurnoutOutput (uint16_t Addr, uint8_t Direction, uint8_t OutputPower)
{
            _PL("notifyDccAccTurnoutOutput");
}


void    notifyDccAccBoardAddrSet (uint16_t BoardAddr)
{
            _PL("notifyDccAccBoardAddrSet");
}

void    notifyDccAccOutputAddrSet (uint16_t Addr)
{
            _PL("notifyDccAccOutputAddrSet");
}


void    notifyDccSigOutputState (uint16_t Addr, uint8_t State)
{
            _PL("notifyDccSigOutputState");
}

void    notifyDccMsg (DCC_MSG * Msg)
{
            _PL("notifyDccMsg");
}


/* deprecated, only for backward compatibility with version 1.4.2.
   don't use in new designs. These functions may be dropped in future versions */
void    notifyDccAccState (uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State)
{
            _PL("notifyDccAccState");
}

void     notifyDccSigState (uint16_t Addr, uint8_t OutputIndex, uint8_t State)
{
            _PL("notifyDccSigState");
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* **********************************************************************************
    notifyCVChange()  Called when a CV value is changed.
                      This is called whenever a CV's value is changed.
    notifyDccCVChange()  Called only when a CV value is changed by a Dcc packet or a lib function.
                      it is NOT called if the CV is changed by means of the setCV() method.
                      Note: It is not called if notifyCVWrite() is defined
                      or if the value in the EEPROM is the same as the value
                      in the write command.

    Inputs:
      CV        - CV number.
      Value     - Value of the CV.

    Returns:
      None
*/
void    notifyCVChange( uint16_t CV, uint8_t Value )
{

  sprintf( sprintfBuffer, "notifyCVChange:  cv:  %3u - value:  %3u ", CV, Value );
  _PL( sprintfBuffer );

  if ( ( ( CV == 251 ) && ( Value == 251 ) ) || ( ( CV == 252 ) && ( Value == 252 ) ) )
  {
    wdt_enable( WDTO_15MS );  //  Resets after 15 milliSecs

    while ( 1 ) {} // Wait for the prescaler time to expire
  }

}       //   end notifyCVChange()


/* **********************************************************************************
      notifyDccFunc() Callback for a multifunction decoder function command.

    Inputs:
      Addr        - Active decoder address.
      AddrType    - DCC_ADDR_SHORT or DCC_ADDR_LONG.
      FuncGrp     - Function group. FN_0      - 14 speed headlight  Mask FN_BIT_00

                                    FN_0_4    - Functions  0 to  4. Mask FN_BIT_00 - FN_BIT_04
                                    FN_5_8    - Functions  5 to  8. Mask FN_BIT_05 - FN_BIT_08
                                    FN_9_12   - Functions  9 to 12. Mask FN_BIT_09 - FN_BIT_12
                                    FN_13_20  - Functions 13 to 20. Mask FN_BIT_13 - FN_BIT_20
                                    FN_21_28  - Functions 21 to 28. Mask FN_BIT_21 - FN_BIT_28
      FuncState   - Function state. Bitmask where active functions have a 1 at that bit.
                                    You must &FuncState with the appropriate
                                    FN_BIT_nn value to isolate a given bit.

    Returns:
      None
*/
void    notifyDccFunc( uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState )
{
  _PL( "notifyDccFunc" );
  _PP( "Address = "    );
  _2L( Addr,        DEC);
  _PP( "AddrType = "   );
  _2L( AddrType,    DEC);
  _PP( "FuncGrp = "    );
  _2L( FuncGrp,     DEC);
  _PP( "FuncState = "  );
  _2L( FuncState,   DEC);

  switch ( FuncGrp )
  {
    case FN_0_4:    //  Function Group 1    F0 F4 F3 F2 F1
    {
      //   exec_function(  0, DATA_PIN_0, ( FuncState & FN_BIT_00 ) >> 4 );
      //   exec_function(  1, FunctionPinDum, ( FuncState & FN_BIT_01 ) >> 0 );
      //   exec_function(  2, FunctionPinDum, ( FuncState & FN_BIT_02 ) >> 1 );
      //   exec_function(  3, FunctionPinDum, ( FuncState & FN_BIT_03 ) >> 2 );
      //   exec_function(  4, FunctionPinDum, ( FuncState & FN_BIT_04 ) >> 3 );
        break ;
    }

    case FN_5_8:    //  Function Group 1    F8  F7  F6  F5
    {
      //   exec_function(  5, FunctionPinDum, ( FuncState & FN_BIT_05 ) >> 0 );
      //   exec_function(  6, FunctionPinDum, ( FuncState & FN_BIT_06 ) >> 1 );
      //   exec_function(  7, FunctionPinDum, ( FuncState & FN_BIT_07 ) >> 2 );
      //   exec_function(  8, FunctionPinDum, ( FuncState & FN_BIT_08 ) >> 3 );
        break ;
    }
    case FN_9_12:   //  Function Group 1    F12 F11 F10 F9
    {
      //   exec_function(  9, FunctionPinDum, ( FuncState & FN_BIT_09 ) >> 0 );
      //   exec_function( 10, FunctionPinDum, ( FuncState & FN_BIT_10 ) >> 1 );
      //   exec_function( 11, FunctionPinDum, ( FuncState & FN_BIT_11 ) >> 2 );
      //   exec_function( 12, FunctionPinDum, ( FuncState & FN_BIT_12 ) >> 3 );
        break ;
    }
    case FN_13_20:  //  Function Group 2  ==  F20 - F13
    case FN_21_28:  //  Function Group 2  ==  F28 - F21
    default:
      {
        break ;
      }
  }
}                     //  End notifyDccFunc()


/* **********************************************************************************
 *  notifyDccSpeed() Callback for a multifunction decoder speed command.
 *                   The received speed and direction are unpacked to separate values.
 *
 *  Inputs:
 *    Addr        - Active decoder address.
 *    AddrType    - DCC_ADDR_SHORT or DCC_ADDR_LONG.
 *    Speed       - Decoder speed. 0               = Emergency stop
 *                                 1               = Regular stop
 *                                 2 to SpeedSteps = Speed step 1 to max.
 *    Dir         - DCC_DIR_REV or DCC_DIR_FWD
 *    SpeedSteps  - Highest speed, SPEED_STEP_14   =  15
 *                                 SPEED_STEP_28   =  29
 *                                 SPEED_STEP_128  = 127
 *
 *  Returns:
 *    None
 */
void    notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Direction, DCC_SPEED_STEPS SpeedSteps )
{
  _PL("notifyDccSpeed  and  DCC_DIRECTION");
  
  _PP( "Address = "    );
  _2L( Addr,        DEC);
  _PP( "AddrType = "   );
  _2L( AddrType,    DEC);
  _PP( "Speed = "      );
  _2L( Speed,       DEC);
  _PP( "Direction = "  );
  _2L( Direction,   DEC);
  _PP( "SpeedSteps = " );
  _2L( SpeedSteps, DEC );
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void exec_function ( int function, int pin, int FuncState )
{
  _PP( "exec_function = " );
  _2L( function,      DEC );
  _PP( "pin (n/u) = "     );
  _2L( pin,           DEC );
  _PP( "FuncState = "     );
  _2L( FuncState,     DEC );

  // switch ( Dcc.getCV( 40 + function ) )
  // {
  //   case  0:  // Function  F0
  //   case  1:
  //   case  2:
  //   case  3:
  //   case  4:
  //   case  5:
  //   case  6:
  //   case  7:
  //   case  8:
  //   case  9:
  //   case 10:  // Function F10
  //     {
  //     //   function_value[ function ] = byte( FuncState );
  //       break ;
  //     }

  //   default:
  //     {
  //       break ;
  //     }
  // }
}                //  End exec_function()


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// // You can use this function if you'd like to set the pulse length in seconds
// // e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
// void setServoPulse(uint8_t n, double pulse) {
//   double pulselength;
  
//   pulselength = 1000000;   // 1,000,000 us per second
//   pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
//   Serial.print(pulselength); Serial.println(" us per period"); 
//   pulselength /= 4096;  // 12 bits of resolution
//   Serial.print(pulselength); Serial.println(" us per bit"); 
//   pulse *= 1000000;  // convert input seconds to us
//   pulse /= pulselength;
//   Serial.println(pulse);
//   pwm.setPWM(n, 0, pulse);
// }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/*  */
