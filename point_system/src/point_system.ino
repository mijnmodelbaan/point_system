

//  https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/tree/master
//  https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
//  https://github.com/adafruit/Adafruit_PCF8574
//  https://github.com/felias-fogg/FlexWire/tree/main

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
// ******** UNCOMMENT THE FOLLOWING LINE TO -not- CHECK AND INITIALISE THE TWO WIRE INTERFACE
// #define NO_TWI_CHECK
//


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* don't print warnings of diagnostic functions from here */
# pragma GCC diagnostic push

# pragma GCC diagnostic ignored "-Wunknown-pragmas"
# pragma GCC diagnostic ignored "-Wsign-compare"
# pragma GCC diagnostic ignored "-Wunused-function"
# pragma GCC diagnostic ignored "-Wunused-variable"


#define baseAddressServos1  0x40   /*  Base address of the SERVOS part  */
#define baseAddressJuicer1  0x20   /*  Base address of the JUICER part  */
#define baseAddressSignal1  0x21   /*  Base address of the SIGNAL part  */
#define baseAddressSwitch1  0x24   /*  Base address of the SWITCH part  */
#define baseAddressSwitch2  0x26   /*  Base address of the SWITCH part  */


/* -------------------------------------------------------------------- */
#include <Arduino.h>         /*  Needed for C++ conversion of INOfile.  */


/* -------------------------------------------------------------------- */
#include <avr/wdt.h>         /*  Needed for automatic reset functions.  */


/* -------------------------------------------------------------------- */
#include <avr/io.h>          /*  AVR device-specific  IO  definitions.  */


/* -------------------------------------------------------------------- */
#include <avr/interrupt.h>   /*  Might be necessary, for ISR handling.  */


/* -------------------------------------------------------------------- */
#include <Wire.h>            /*  The file for the TWI (I2C) interface.  */


/* -------------------------------------------------------------------- */
#include <NmraDcc.h>         /*  needed for computing the DCC signals   */

NmraDcc     Dcc ;
DCC_MSG  Packet ;

#define This_Decoder_Address     40
#define NMRADCC_SIMPLE_RESET_CV 251
#define NMRADCC_MASTER_RESET_CV 252
#define FunctionPinDcc            2  /* Inputpin DCCsignal (PD2) */


/* --------------------------------------------------------------------- */
#include <Adafruit_PWMServoDriver.h>  /*  needed for driving the servos  */

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver( baseAddressServos1 );

#define  SERVOMIN   150   /*  min pulse value for full left servo throw (zero degrees)  */
#define  SERVOMAX   600   /*  max pulse value for full right servo throw (180 degrees)  */
#define  SERVOOFF  4096   /*  pulse value that turns the servo pin off (wear and tear)  */
#define  USEC_MIN   600   /*  rounded 'minimum' microsec based on minimum pulse of 150  */
#define  USEC_MAX  2400   /*  rounded 'maximum' microsec based on maximum pulse of 600  */
#define  SERVOFRQ    50   /*  digital servos run at ~60Hz updates, analog servos ~50Hz  */

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


/* -----------------------------------------------------------------
   here we declare our own 'header' file ( optional variables )   */

void displayText( bool first, bool next, bool clear = true, byte shift = 3 );


# pragma GCC diagnostic pop
/* don't print warnings of diagnostic functions till here */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*  printing of debug and test options  */

# if defined( _DEBUG_ ) || defined( TESTRUN )
  # define _PP( a   ) Serial.print(      a );
  # define _PL( a   ) Serial.println(    a );
  # define _2P( a,b ) Serial.print(   a, b );
  # define _2L( a,b ) Serial.println( a, b );
# else
  # define _PP( a   )
  # define _PL( a   )
  # define _2P( a,b )
  # define _2L( a,b )
# endif

char    sprintfBuffer[ 96 ] ;    /*  Max length for a buffer.  */
bool    foundEom    = false ;    /*  Found  end  of messages.  */
String  commandString =  "" ;    /*  String to hold incoming   */


volatile unsigned long currentMillis; /*  used for the timekeeping  */
volatile unsigned long elapsedMillis; /*  used for the timekeeping  */


/* /////////////////////////////////////////////////////////////////////////////////////////////////// */
/*  *****  small macro, is used to create timed events  *****  see the example below the macro  *****  */

#define runEvery( n ) for ( static unsigned long lasttime; millis() - lasttime > ( unsigned long )( n ); lasttime = millis() )

/* example: runEvery( time ) { command; }  check if the time (in millis) has passed and if so executes */
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
  uint16_t  angleLft      = SERVOMIN ;  /*  angle to go full  Left  */
  uint16_t  angleRgt      = SERVOMAX ;  /*  angle to go full Right  */
  uint16_t  moveEndState  = SERVOOFF ;  /*  this is end point move  */
  uint16_t  actualServoP  = SERVOOFF ;  /*  keep track of movement  */
  uint16_t  juicerTime    =     1255 ;  /*  a juicer needs to wait  */
  bool      moveToCenter  =    false ;  /*  makes a move to Center  */
  bool      moveFromCntr  =    false ;  /*  moving to Left / Right  */
  bool      currentState  =    false ;  /*  do we need attention ?  */
};
QUEUE volatile *turnout_queue = new QUEUE[ 16 + 1 ];   /*  16 turnouts max [15:0] + 1 dummy  */

/* /////////////////////////////////////////////////////////////////////////////////////////////////// */


volatile byte currentTurnout = 16;  /*  default turnout updating commands  */
volatile byte positionUpdate =  2;  /*  turnout position update (x, c, v)  */
volatile long ninety_degrees = ( ( SERVOMAX + SERVOMIN ) / 2 ) * ( ( 1000000 / SERVOFRQ ) / 4096 );


/* //////////////////////////////////////////////////////////////////////////////////////////////////////
  this is the place where everything is set up  */

void setup()
{
  // noInterrupts(); /*  cli() can also be used  */


/* =========================================================================================
  the following lines set the A0 to A3 lines as input (with a pull-up) for the interrupts */
  // DDRC   = DDRC   & 0b11110000;  /*  Set bits A[3-0] as inputs, leave the rest as-is      */
  // PORTC  = PORTC  | 0b00001111;  /*  Switch bits A[3-0] to PULL-UP, leave the rest as-is  */

  // PCICR  = PCICR  | 0b00000010;  /*  Set PCIE1, meaning enable the PCINT[14:8] interrupts */
  // PCMSK1 = PCMSK1 | 0b00001111;  /*  A0 = PCINT8, A1 = PCINT9, A2 = PCINT10, A3 = PCINT11 */
  // PCIFR  = PCIFR  | 0b00000010;  /*  Clear interrupt for all interrupts from port A[7:0]  */


  /*  TODO: check these regs  carefully with the sequences  */
  // ADCSRA = ADCSRA & 0b01010000;  /*  disable possible analog conversions and interrupts   */
  // ADCSRB = ADCSRB & 0b00000000;  /*  disable possible analog conversions and interrupts   */

  // ACSR   = ACSR   & 0b11110111;  /*  Bit 3 – ACIE: Analog Comparator Interrupt Enable     */
  // ACSR   = ACSR   | 0b10000000;  /*  Bit 7 – ACD: Analog Comparator Disable               */


  interrupts(); /*  Arduino UNO seems to require that we turn on interrupts for I2C to work  */

  Wire.begin( /* SDA_PIN, SCL_PIN */ );        /*  start the I2C / TWI library - mandatory!  */
  Wire.setClock( 400000 );       /*  go into 'fast 400khz I2C' mode, but watch out for this  */

/*  TODO:  make setClock selectable via a CV ???   */


/*  check if starting the Serial Interface is needed and do so if yes  */
#if defined( _DEBUG_ ) || defined( TESTRUN )

  // Serial.begin( 9600, SERIAL_8N1 ); /*  ( 115200, SERIAL_8N1 );  */
  Serial.begin( 115200, SERIAL_8N1 );

  while (! Serial)
  {
    ; // wait for Serial port to connect. Needed for native USB port.
  }

  Serial.flush();   // Wait for all the rubbish to finish displaying.

  while (Serial.available() > 0)
  {
    Serial.read(); // Clear the input buffer to get 'real' inputdata.
  }

  Serial.setTimeout( 1000 );  /*  maximum wait time after an input  */
  commandString.reserve( 64 ); /* reserve 64 bytes for inputString  */

  displayText( false, false, false,  3 );  /*  do clear the screen  */

#endif


#if !defined( NO_TWI_CHECK )
/* ===========================================================
  here the existence of several parts of the project is tested
  if you don't have a part in your system, delete that code */

  if (! servos.begin( ) )
  {
    _PL( "Couldn't find SERVOS  - check and reset" );
    // softwareReset( WDTO_4S );  /*  resets every 4s */
  } else {
    _PL( "Found SERVOS  - going to the next steps" ); 
    servos.setPWMFreq( SERVOFRQ );
  }

  if (! juicer.begin_I2C( baseAddressJuicer1, &Wire ) )
  {
    _PL( "Couldn't find JUICER  - check and reset" );
    // softwareReset( WDTO_4S );  /*  resets every 4s */
  } else {
    _PL( "Found JUICER  - going to the next steps" ); 
    for ( uint8_t p = 0; p < 16; ++p )   /*  initialise all 16 as OUTPUT  */
    {
      juicer.pinMode( p, OUTPUT );
    }
  }

  if (! signal.begin_I2C( baseAddressSignal1, &Wire ) )
  {
    _PL( "Couldn't find SIGNAL  - check and reset" );
    // softwareReset( WDTO_4S );  /*  resets every 4s */
  } else {
    _PL( "Found SIGNAL  - going to the next steps" ); 
    for ( uint8_t p = 0; p < 16; ++p )   /*  initialise all 16 as OUTPUT  */
    {
      signal.pinMode( p, OUTPUT );
    }
  }

  if (! inputa.begin( baseAddressSwitch1, &Wire ) )
  { 
    _PL( "Couldn't find SWITCH1 - check and reset" ); 
    softwareReset( WDTO_4S );  /*  resets every 4s */
  } else {
    _PL( "Found SWITCH1 - going to the next steps" ); 
    for ( uint8_t p = 0; p <  8; ++p )   /*  initialise all  8 as  INPUT  */
    {
      inputa.pinMode( p,  INPUT_PULLUP ); // INPUT );
    }
  }

  if (! inputb.begin( baseAddressSwitch2, &Wire ) )
  { 
    _PL( "Couldn't find SWITCH2 - check and reset" ); 
    softwareReset( WDTO_4S );  /*  resets every 4s */
  } else {
    _PL( "Found SWITCH2 - going to the next steps" ); 
    for ( uint8_t p = 0; p <  8; ++p )   /*  initialise all  8 as  INPUT  */
    {
      inputb.pinMode( p,  INPUT_PULLUP ); // INPUT );
    }
  }

/* ======================================================== */
#endif  /*  NO_TWI_CHECK  */



turnout_queue[ 0].angleLft  = 175;
turnout_queue[ 0].angleRgt  = 475;

turnout_queue[ 1].angleLft  = 275;
turnout_queue[ 1].angleRgt  = 375;

turnout_queue[ 2].angleLft  = 175;
turnout_queue[ 2].angleRgt  = 475;

turnout_queue[ 3].angleLft  = 175;
turnout_queue[ 3].angleRgt  = 475;

turnout_queue[ 4].angleLft  = 175;
turnout_queue[ 4].angleRgt  = 475;

turnout_queue[ 5].angleLft  = 175;
turnout_queue[ 5].angleRgt  = 475;

turnout_queue[10].angleLft  = 175;
turnout_queue[10].angleRgt  = 475;

turnout_queue[11].angleLft  = 175;
turnout_queue[11].angleRgt  = 475;

turnout_queue[12].angleLft  = 175;
turnout_queue[12].angleRgt  = 475;

turnout_queue[13].angleLft  = 175;
turnout_queue[13].angleRgt  = 475;

turnout_queue[14].angleLft  = 275;
turnout_queue[14].angleRgt  = 375;

turnout_queue[15].angleLft  = 175;
turnout_queue[15].angleRgt  = 475;


/*  set all SERVOS, JUICERs and SINALs to the SWITCHes states  */
readButtonsInputa( );
readButtonsInputb( );


  //  Call the DCC 'pin' and 'init' functions to enable the DCC Receivermode
  Dcc.pin( digitalPinToInterrupt( FunctionPinDcc ), FunctionPinDcc, false );
  Dcc.init( MAN_ID_DIY, 201, FLAGS_MY_ADDRESS_ONLY, 0 );      delay( 1000 );
  //c.initAccessoryDecoder( MAN_ID_DIY,  201, FLAGS_MY_ADDRESS_ONLY,    0 );

  Dcc.setCV( NMRADCC_SIMPLE_RESET_CV, 0 );  /*  Reset the just_a_reset CV */


  displayText(  true, false,  true,  2 );   /*  finally clear the screen  */

};
/* /////////////////////////////////////////////////////////////////////////////////////////////////// */


void loop() {

  currentMillis = millis();  /*  lets keep track of the time  */


  runEvery( 25 ) { readButtonsInputa( ); }

  runEvery( 25 ) { readButtonsInputb( ); }

  runEvery(  5 ) { setsServosCorrect( ); }


  if ( Serial.available() > 0 )  /*  Serial Input needs message-handling  */
  {
    commandString = Serial.readString();

    foundEom = ( ( commandString.length() > 0 ) ? true : false );
  } 

  if ( foundEom )  /*  Serial Input needs message-handling  */
  {
    parseCom( commandString );
  }

};
/* /////////////////////////////////////////////////////////////////////////////////////////////////// */


void setsServosCorrect()
{
  if ( scan ( baseAddressServos1 ) == 0 )  /*  test SERVOS  existance  */
  {
    for ( uint8_t p = 0; p < 16; ++p )
    {
      if ( turnout_queue[ p ].currentState )
      {
        _PP( currentMillis );
        _PP( " - " );
        _PP( p + 1 );
        _PP( " - " );
        _PL( turnout_queue[ p ].actualServoP )

        if ( turnout_queue[ p ].actualServoP  > turnout_queue[ p ].moveEndState )
        {
          turnout_queue[ p ].actualServoP -= 1;           /*  move Lft  */
        }

        if ( turnout_queue[ p ].actualServoP  < turnout_queue[ p ].moveEndState )
        {
          turnout_queue[ p ].actualServoP += 1;           /*  move Rgt  */
        }

        servos.setPWM( p, 0, turnout_queue[ p ].actualServoP );  /*  move the servo  */

        if ( turnout_queue[ p ].actualServoP == turnout_queue[ p ].moveEndState )
        {
          turnout_queue[ p ].actualServoP = SERVOOFF;  /*  stop moving  */
          turnout_queue[ p ].currentState = false;  /*  clear the flag  */

          servos.setPWM( p, 0, SERVOOFF );          /*  stop the servo  */
        }

      }
    }
  }
};
/* /////////////////////////////////////////////////////////////////////////////////////////////////// */


/*  inspired by  https://forum.arduino.cc/t/debouncing-buttons-read-with-a-pcf-8575/1038096/14  */


volatile uint8_t  inputaPreviousInput;                          // previous reading of buttons
volatile uint8_t  inputaPreviousState;                          // Button previous state 
volatile uint8_t  inputaActualInState;                          // Button state (pressed, no pressed)

volatile uint8_t  inputaActualToggled;                          // actual Toggle
volatile uint8_t  inputaActualPressed;                          // actual Pressed
volatile uint8_t  inputaActualRelease;                          // Actual Released

volatile uint8_t  inputaStablePressed;                          // Stable Pressed
volatile uint8_t  inputaStableRelease;                          // Stable Released

volatile uint8_t  inputaButtonPinMask = 0b11111111;  /*  tells us which pins are marked as inputs (1)  */

void readButtonsInputa()  /*  inputa = SWITCH1 = 0x24 = inputs  1 t/m  8  */
{
  interrupts(); // Arduino UNO seems to require that we turn on interrupts for I2C to work!

  uint8_t inputaHoldsNewInput = ~inputa.digitalReadByte() & inputaButtonPinMask;

  /*  this section reacts immediately to a new, different reading (changes)  */

  inputaActualPressed  =  inputaHoldsNewInput & ~inputaPreviousInput;
  inputaActualRelease  = ~inputaHoldsNewInput &  inputaPreviousInput;
  inputaActualToggled  =  inputaActualToggled ^  inputaActualPressed;

  /*  this sketch will use the upper part (the rest is for a later session)  */

  if ( ( inputaActualPressed ) || ( inputaActualRelease ) )
  {
    setTheSwitchesInputa( );
  }

  /*  next section develops the idea of a stable press (same for 2 readings) */

  // inputaStablePressed  = ~( inputaHoldsNewInput ^ inputaPreviousInput );
  // inputaActualInState &= ~inputaStablePressed;
  // inputaActualInState |=  inputaHoldsNewInput & inputaStablePressed;

  /*  next section might even get a stabler reading of fuzzy button presses  */

  // inputaStablePressed  =  inputaActualInState & ~inputaPreviousState;
  // inputaStableRelease  = ~inputaActualInState &  inputaPreviousState;

  /*  always insert the next lines to keep the history in sync with reading  */
  inputaPreviousInput = inputaHoldsNewInput;
  inputaPreviousState = inputaActualInState;
};
/* /////////////////////////////////////////////////////////////////////////////////////////////////// */


volatile uint8_t  inputbPreviousInput;                          // previous reading of buttons
volatile uint8_t  inputbPreviousState;                          // Button previous state 
volatile uint8_t  inputbActualInState;                          // Button state (pressed, no pressed)

volatile uint8_t  inputbActualToggled;                          // actual Toggle
volatile uint8_t  inputbActualPressed;                          // actual Pressed
volatile uint8_t  inputbActualRelease;                          // Actual Released

volatile uint8_t  inputbStablePressed;                          // Stable Pressed
volatile uint8_t  inputbStableRelease;                          // Stable Released

volatile uint8_t  inputbButtonPinMask = 0b11111111;  /*  tells us which pins are marked as inputs (1)  */

void readButtonsInputb()  /*  inputb = SWITCH2 = 0x26 = inputs  9 t/m 16  */
{
  interrupts(); // Arduino UNO seems to require that we turn on interrupts for I2C to work!

  uint8_t inputbHoldsNewInput = ~inputb.digitalReadByte() & inputbButtonPinMask;

  /*  this section reacts immediately to a new, different reading  (changes) */

  inputbActualPressed  =  inputbHoldsNewInput & ~inputbPreviousInput;
  inputbActualRelease  = ~inputbHoldsNewInput &  inputbPreviousInput;
  inputbActualToggled  =  inputbActualToggled ^  inputbActualPressed;

  /*  this sketch will use the upper part (the rest is for a later session)  */

  if ( ( inputbActualPressed ) || ( inputbActualRelease ) )
  {
    setTheSwitchesInputb( );
  }

  /*  next section develops the idea of a stable press (same for 2 readings) */

  // inputbStablePressed  = ~( inputbHoldsNewInput ^ inputbPreviousInput );
  // inputbActualInState &= ~inputbStablePressed;
  // inputbActualInState |=  inputbHoldsNewInput & inputbStablePressed;

  /*  next section might even get a stabler reading of fuzzy button presses  */

  // inputbStablePressed  =  inputbActualInState & ~inputbPreviousState;
  // inputbStableRelease  = ~inputbActualInState &  inputbPreviousState;

  /*  always insert the next lines to keep the history in sync with reading  */
  inputbPreviousInput = inputbHoldsNewInput;
  inputbPreviousState = inputbActualInState;
};
/* /////////////////////////////////////////////////////////////////////////////////////////////////// */


void setTheSwitchesInputa( )  /*  here the switches  1 till  8 are handled  */
{
  for ( uint8_t p = 0; p <  8; ++p )
  {

    if ( bitRead ( inputaActualPressed, p ) )  /*  key pressed   */
    {
      turnout_queue[ p + 0 ].moveEndState = turnout_queue[ p + 0 ].angleRgt;
      turnout_queue[ p + 0 ].currentState = true;
      turnout_queue[ p + 0 ].actualServoP = turnout_queue[ p + 0 ].angleLft;
    }

    if ( bitRead ( inputaActualRelease, p ) )  /*  key released  */
    {
      turnout_queue[ p + 0 ].moveEndState = turnout_queue[ p + 0 ].angleLft;
      turnout_queue[ p + 0 ].currentState = true;
      turnout_queue[ p + 0 ].actualServoP = turnout_queue[ p + 0 ].angleRgt;
    }

  }

  if ( scan ( baseAddressJuicer1 ) == 0 )  /*  test JUICER  existance  */
  {
    uint8_t juicerInputaRead = juicer.readGPIOA ( );

    for ( uint8_t p = 0; p <  8; ++p )
    {

      if ( bitRead ( inputaActualPressed, p ) )  /*  key pressed   */
      {
        bitSet ( juicerInputaRead, p );
      }

      if ( bitRead ( inputaActualRelease, p ) )  /*  key released  */
      {
        bitClear ( juicerInputaRead, p );
      }

    }
    juicer.writeGPIOA ( juicerInputaRead );
  }

  if ( scan ( baseAddressSignal1 ) == 0 )  /*  test SIGNAL  existance  */
  {
    uint8_t signalInputaRead = signal.readGPIOA ( );

    for ( uint8_t p = 0; p <  8; ++p )
    {

      if ( bitRead ( inputaActualPressed, p ) )  /*  key pressed   */
      {
        bitSet ( signalInputaRead, p );
      }

      if ( bitRead ( inputaActualRelease, p ) )  /*  key released  */
      {
        bitClear ( signalInputaRead, p );
      }

      if ( bitRead ( inputaActualToggled, p ) )  /*  key toggled   */
      {
        bitClear ( inputaActualToggled, p );  /*  clear toggled bit  */
      }

    }
    signal.writeGPIOA ( signalInputaRead );
  }
};
/* /////////////////////////////////////////////////////////////////////////////////////////////////// */


void setTheSwitchesInputb( )  /*  here the switches  9 till 16 are handled  */
{
  for ( uint8_t p = 0; p <  8; ++p )
  {

    if ( bitRead ( inputbActualPressed, p ) )  /*  key pressed   */
    {
      turnout_queue[ p + 8 ].moveEndState = turnout_queue[ p + 8 ].angleRgt;
      turnout_queue[ p + 8 ].currentState = true;
      turnout_queue[ p + 8 ].actualServoP = turnout_queue[ p + 8 ].angleLft;
    }

    if ( bitRead ( inputbActualRelease, p ) )  /*  key released  */
    {
      turnout_queue[ p + 8 ].moveEndState = turnout_queue[ p + 8 ].angleLft;
      turnout_queue[ p + 8 ].currentState = true;
      turnout_queue[ p + 8 ].actualServoP = turnout_queue[ p + 8 ].angleRgt;
    }

  }

  if ( scan ( baseAddressJuicer1 ) == 0 )  /*  test JUICER existance  */
  {
    uint8_t juicerInputbRead = juicer.readGPIOB ( );

    for ( uint8_t p = 0; p <  8; ++p )
    {

      if ( bitRead ( inputbActualPressed, p ) )  /*  key pressed   */
      {
        bitSet ( juicerInputbRead, p + 8 );
      }

      if ( bitRead ( inputbActualRelease, p ) )  /*  key released  */
      {
        bitClear ( juicerInputbRead, p + 8 );
      }

    }
    juicer.writeGPIOB ( juicerInputbRead );
  }

  if ( scan ( baseAddressSignal1 ) == 0 )  /*  test SIGNAL  existance  */
  {
    uint8_t signalInputbRead = signal.readGPIOB ( );

    for ( uint8_t p = 0; p <  8; ++p )
    {

      if ( bitRead ( inputbActualPressed, p ) )  /*  key pressed   */
      {
        bitSet ( signalInputbRead, p + 8 );
      }

      if ( bitRead ( inputbActualRelease, p ) )  /*  key released  */
      {
        bitClear ( signalInputbRead, p + 8 );
      }

      if ( bitRead ( inputbActualToggled, p ) )  /*  key toggled   */
      {
        bitClear ( inputbActualToggled, p );  /*  clear toggled bit  */
      }

    }
    signal.writeGPIOB ( signalInputbRead );
  }
};
/* /////////////////////////////////////////////////////////////////////////////////////////////////// */




 /*
 * setServoAngle(int ang)
 * gets angle in degrees and returns matching pulse value
 */
int setServoAngle( int ang ) {
  int pulse = map( ang, 0, 180, SERVOMIN, SERVOMAX ); 
  _PP( "...Angle: " ); _PP( ang ); 
  _PP( " / Pulse: " ); _PP( pulse );

  double pulselength = 1000000 / SERVOFRQ;  /*  1,000,000 us per second / 50 = 20.000  */
  _PP ( pulselength ); _PP ( " us per period = " ); 
  pulselength /= 4096;  // 12 bits of resolution   20.000 / 4096 = 4,883
  _PP ( pulselength ); _PP ( " us per bit - " ); 
  _PP ( "default center: "); _PL ( ninety_degrees );
  return pulse;
};
/* /////////////////////////////////////////////////////////////////////////////////////////////////// */



/* /////////////////////////////////////////////////////////////////////////////////////////////////// */
/* next function is doing an automatic reset of the processor after the preScaler time has elapsed     */

void softwareReset( uint8_t preScaler )
{
  wdt_enable( preScaler );

  while( 1 ) {}  // Wait for the prescaler time to expire and do an auto-reset
};
/* /////////////////////////////////////////////////////////////////////////////////////////////////// */



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
volatile bool next_menu = false, do_escape = true ;

void displayText( bool first, bool next, bool clear, byte shift )
{
  if ( clear ==  true)   /*  clear the screen and display a header text  */
  {
    if ( do_escape )
    {
    _PP(F( "\e[2J\e[1;1H" )); /*  send escape sequences out  */
    }

    _PL(F( " ***  AtMega328 controling your turnouts  *** " ));
  }

  if ( shift > 0 )
  {
    byte lines = constrain( shift, 1, 26 );  /*  max 26 empty lines  */

    for ( byte p = 0; p < lines; ++p )
    {
      _PL( " ");
    }
  }

  if( first == true )
  {
    _PL( " " ); // An extra empty line for better understanding
    _PL(F( "**   put in one of the following commands   **" ));
    _PL(F( "----------------------------------------------" ));
    _PL(F( "FD  clear everything: Factory Default"          ));
    _PL(F( "DD  prints CV values: to your monitor"          ));
    _PL(   " "                                               );
    _PL(F( "DS  set default item number ( 1-16 ): DS x#"    ));
    _PL(   " "                                               );
    _PL(F( "DR  reads a configuration variable: DR xx#"     ));
    _PL(F( "DW  write a configuration variable: DW xx# x#"  ));
    _PL(F( "----------------------------------------------" ));
    _PL(F( "**    use the  [Enter]  key if necessary    **" ));
    _PL(F( "**    do NOT forget the mandatory spaces    **" ));
    _PL( " " ); // An extra empty line for better understanding
  }

  if(  next  == true )
  {
    _PL( " " ); // An extra empty line for better understanding
    _PL(F( "**   put in one of the following commands   **" ));
    _PL(F( "----------------------------------------------" ));
    _PL(F( "x = adjust servo  left x degrees at a time"     ));
    _PL(F( "c = center the default servo (set with DS #)"   ));
    _PL(F( "v = adjust servo right x degrees at a time"     ));
    _PL(   " "                                               );
    _PL(F( "p = invert  SERVOS  setting for this turnout"   ));
    _PL(F( "j = invert  JUICER  setting for this turnout"   ));
    _PL(F( "s = invert  SIGNAL  setting for this turnout"   ));
    _PL(F( "i = invert  SWITCH  setting for this turnout"   ));
    _PL(   " "                                               );
    _PL(F( "d = display changed setting for this turnout"   ));
    _PL(F( "t = throw the switch (servo, juicer, signal)"   ));
    _PL(F( "w = write the settings, in corresponding CVs"   ));
    _PL(F( "----------------------------------------------" ));
    _PL(F( "**   any other key discards the settings    **" ));
    _PL( "" );  // An extra empty line for better understanding
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

#if defined( _DEBUG_ )
  memset( sprintfBuffer, 0, sizeof sprintfBuffer );
  sprintf( sprintfBuffer, "charOne: %1c - charTwo: %1c - intOne: %3u - intTwo: %3u - deTected: %3u", charOne, charTwo, intOne, intTwo, deTected );
  _PL( sprintfBuffer );
#endif /*  _DEBUG_  */


  if ( ( next_menu ) & ( deTected == 1 ) )  /*  single character commands of next menu  */
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


  if ( ( next_menu ) & ( deTected == 2 ) )  /*  double character commands of next menu  */
  {
    switch ( charOne )
    {
      case 'x': /*  x = adjust servo  left x degrees at a time  */
      {
        switch ( charTwo )
        {
          case '+': /*  x+  */
          {
            turnout_queue[ 16 ].angleLft += positionUpdate ;

            turnout_queue[ 16 ].angleLft = constrain( turnout_queue[ 16 ].angleLft, turnout_queue[ 16 ].angleRgt, 180 );

            // if ( turnout_queue[ 16 ].angleLft > 180 ) { turnout_queue[ 16 ].angleLft = 180; }
            servos.setPWM( currentTurnout, 0, setServoAngle( turnout_queue[ 16 ].angleLft ) );
            sprintf( sprintfBuffer, "new servo  left:  %3u ", turnout_queue[ 16 ].angleLft );
            _PL( sprintfBuffer );
            turnout_queue[ 16 ].currentState = true;  /*  something has changed  */

            break;
          }
          case '-': /*  x-  */
          {
            turnout_queue[ 16 ].angleLft -= positionUpdate ;

            turnout_queue[ 16 ].angleLft = constrain( turnout_queue[ 16 ].angleLft, turnout_queue[ 16 ].angleRgt, 180 );

            // if ( turnout_queue[ 16 ].angleLft > 180 ) { turnout_queue[ 16 ].angleLft = 180; }
            servos.setPWM( currentTurnout, 0, setServoAngle( turnout_queue[ 16 ].angleLft ) );
            sprintf( sprintfBuffer, "new servo  left:  %3u ", turnout_queue[ 16 ].angleLft );
            _PL( sprintfBuffer );
            turnout_queue[ 16 ].currentState = true;  /*  something has changed  */

            break;
          }
          default:  /*  discard all changes  */
            break;
        }
        break;
      }
      case 'v': /*  v = adjust servo right x degrees at a time  */
      {
        switch ( charTwo )
        {
          case '+': /*  v+  */
          {
            turnout_queue[ 16 ].angleRgt += positionUpdate ;

            turnout_queue[ 16 ].angleRgt = constrain( turnout_queue[ 16 ].angleRgt, 0, turnout_queue[ 16 ].angleLft );

            // if ( turnout_queue[ 16 ].angleRgt < 0 ) { turnout_queue[ 16 ].angleRgt = 0; } /*  prevent overdrive  */

            servos.setPWM( currentTurnout, 0, setServoAngle( turnout_queue[ 16 ].angleRgt ) );
            sprintf( sprintfBuffer, "new servo right:  %3u ", turnout_queue[ 16 ].angleRgt );
            _PL( sprintfBuffer );
            turnout_queue[ 16 ].currentState = true;  /*  something has changed  */

            break;
          }
          case '-': /*  v-  */
          {
            turnout_queue[ 16 ].angleRgt -= positionUpdate ;

            turnout_queue[ 16 ].angleRgt = constrain( turnout_queue[ 16 ].angleRgt, 0, turnout_queue[ 16 ].angleLft );

            // if ( turnout_queue[ 16 ].angleRgt < 0 ) { turnout_queue[ 16 ].angleRgt = 0; } /*  prevent overdrive  */

            servos.setPWM( currentTurnout, 0, setServoAngle( turnout_queue[ 16 ].angleRgt ) );
            sprintf( sprintfBuffer, "new servo right:  %3u ", turnout_queue[ 16 ].angleRgt );
            _PL( sprintfBuffer );
            turnout_queue[ 16 ].currentState = true;  /*  something has changed  */

            break;
          }
          default:  /*  discard all changes  */
            break;
        }
        break;
      }
      default:  /*  discard all changes  */
        // next_menu = false;
        break;
    }
  }     /*  end of double character commands  */


  if ( (! next_menu ) & ( deTected == 2 ) )  /*  double character commands of first menu  */
  {
    switch ( charOne )
    {
      case 'f': /*    */
      {
        switch ( charTwo )
        {
          case 'd': /*  FD  clear everything: Factory Default  */
          {
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
            intOne = constrain( intOne, 1, 16 );  // limits range of sensor values to 1 till 16

            sprintf( sprintfBuffer, "Default turnout changed to: %d", intOne );
            _PL( sprintfBuffer );

            break;
          }
          case 'r': /*  DR  reads a configuration variable: DR xx#  */
          {
            intOne = constrain( intOne,  0, 255 );  /*  limits range of CVs from  0 till 255  */

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


  if ( (! next_menu ) & ( deTected == 4 ) )  /*  quad character commands of first menu  */
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
            intOne = constrain( intOne,  0, 255 );  /*  limits range of CVs from  0 till 255  */

            intTwo = constrain( intTwo,  0, 255 );  /*  limits CVvalue range from 0 till 255  */

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
/* /////////////////////////////////////////////////////////////////////////////////////////////////// */

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

//   switch ( FuncGrp )
//   {
//     case FN_0_4:    //  Function Group 1    F0 F4 F3 F2 F1
//     {
//       //   exec_function(  0, DATA_PIN_0, ( FuncState & FN_BIT_00 ) >> 4 );
//       //   exec_function(  1, FunctionPinDum, ( FuncState & FN_BIT_01 ) >> 0 );
//       //   exec_function(  2, FunctionPinDum, ( FuncState & FN_BIT_02 ) >> 1 );
//       //   exec_function(  3, FunctionPinDum, ( FuncState & FN_BIT_03 ) >> 2 );
//       //   exec_function(  4, FunctionPinDum, ( FuncState & FN_BIT_04 ) >> 3 );
//         break ;
//     }

//     case FN_5_8:    //  Function Group 1    F8  F7  F6  F5
//     {
//       //   exec_function(  5, FunctionPinDum, ( FuncState & FN_BIT_05 ) >> 0 );
//       //   exec_function(  6, FunctionPinDum, ( FuncState & FN_BIT_06 ) >> 1 );
//       //   exec_function(  7, FunctionPinDum, ( FuncState & FN_BIT_07 ) >> 2 );
//       //   exec_function(  8, FunctionPinDum, ( FuncState & FN_BIT_08 ) >> 3 );
//         break ;
//     }
//     case FN_9_12:   //  Function Group 1    F12 F11 F10 F9
//     {
//       //   exec_function(  9, FunctionPinDum, ( FuncState & FN_BIT_09 ) >> 0 );
//       //   exec_function( 10, FunctionPinDum, ( FuncState & FN_BIT_10 ) >> 1 );
//       //   exec_function( 11, FunctionPinDum, ( FuncState & FN_BIT_11 ) >> 2 );
//       //   exec_function( 12, FunctionPinDum, ( FuncState & FN_BIT_12 ) >> 3 );
//         break ;
//     }
//     case FN_13_20:  //  Function Group 2  ==  F20 - F13
//     case FN_21_28:  //  Function Group 2  ==  F28 - F21
//     default:
//       {
//         break ;
//       }
//   }
};                    //  End notifyDccFunc()


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
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// void exec_function ( int function, int pin, int FuncState )
// {
//   _PP( "exec_function = " );
//   _2L( function,      DEC );
//   _PP( "pin (n/u) = "     );
//   _2L( pin,           DEC );
//   _PP( "FuncState = "     );
//   _2L( FuncState,     DEC );

//   // switch ( Dcc.getCV( 40 + function ) )
//   // {
//   //   case  0:  // Function  F0
//   //   case  1:
//   //   case  2:
//   //   case  3:
//   //   case  4:
//   //   case  5:
//   //   case  6:
//   //   case  7:
//   //   case  8:
//   //   case  9:
//   //   case 10:  // Function F10
//   //     {
//   //     //   function_value[ function ] = byte( FuncState );
//   //       break ;
//   //     }

//   //   default:
//   //     {
//   //       break ;
//   //     }
//   // }
// }                //  End exec_function()


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*  TODO:  check this out - maybe usable for JUICER setting during half-time SERVOS  */

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


/* /////////////////////////////////////////////////////////////////////////////////////////////////// */

/*  scanning the TWI (I2C) interface for existing addresses  */
/*  maybe a replacement of 
  if (! servos.begin( ) )
*/

/*  example from   https://github.com/luisllamasbinaburo/Arduino-I2CScanner/tree/master  */

byte scan( byte address )
{
  Wire.beginTransmission( address );
  byte error = Wire.endTransmission( );
  delay ( 100 );  /*  wait a little wile - debounce  */
  return error;
};


void printBin(byte aByte) {
  for ( int8_t aBit = 7; aBit >= 0; --aBit )
    Serial.write(bitRead(aByte, aBit) ? '1' : '0');
};

/* /////////////////////////////////////////////////////////////////////////////////////////////////// */


/* /////////////////////////////////////////////////////////////////////////////////////////////////// */


/*  */
