

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


/* -------------------------------------------------------------- */
#include <NmraDcc.h>   /*  needed for computing the DCC signals   */

NmraDcc     Dcc ;
DCC_MSG  Packet ;

#define This_Decoder_Address     40
#define NMRADCC_SIMPLE_RESET_CV 251
#define NMRADCC_MASTER_RESET_CV 252
#define FunctionPinDcc            2  /* Inputpin DCCsignal */

uint8_t thisDecoderdirection = DCC_DIR_FWD;
uint8_t prevDecoderdirection = DCC_DIR_REV;


/* --------------------------------------------------------------------- */
#include <Adafruit_PWMServoDriver.h>  /*  needed for driving the servos  */

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver( baseAddressServos1 );

const int SERVOMIN =  125;   // min pulse value for full left servo throw (zero degrees)
const int SERVOMAX =  625;   // max pulse value for full right servo throw (180 degrees)
const int SERVOOFF = 4096;   // pulse value that turns the servo pin off (wear and tear)

int servoPos = 90;
int servoLft = 90;
int servoRgt = 90;
bool thrown  = false; // turnout is in normal position


/* -------------------------------------------------------------- */
#include <MCP23017.h>     /* needed by JUICER and SIGNAL driving  */

MCP23017 juicer = MCP23017( baseAddressJuicer1 );
MCP23017 signal = MCP23017( baseAddressSignal1 );


/* -------------------------------------------------------------- */
#include <pcf8574.h>  /* used by both SWITCH drivers  */

PCF8574 ex1( baseAddressSwitch1 );
PCF8574 ex2( baseAddressSwitch2 );


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

  {  40,   0},   /*  0 = disable ALL, 1 = enable ALL ( >> at start up or reset)  F0  */
  {  41, 255},
  {  42, 255},
  {  43, 155},
  {  44,   1},   /*  dimmingfactor multiplier  */
  {  45,  10},   /*  blinking  ON  multiplier  */
  {  46,  10},   /*  blinking OFF  multiplier  */
  {  47,  10},   /*  fade up time  multiplier  */
  {  48,  10},   /*  fade down     multiplier  */
  {  49,   0},   /*    */


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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {



/*  check if starting the Serial Interface is needed and do so if yes  */
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


/*  TODO: ==========================================  */
  servos.begin();

  servos.setPWMFreq(60);
  servos.setPWM( 0, 0, setServoAngle(90) );   //set servo to center (90 degrees)


/**  example
 * On every loop, the state of the port B is copied to port A.
 *
 * Use active low inputs on port B. Internal pullups are enabled by default by the library so there is no need for external resistors.
 * Place LEDS on port A for instance. 
 * When pressing a button, the corresponding led is shut down.
 * 
 * You can also uncomment one line to invert the input (when pressing a button the corresponding led is lit)
 */
  juicer.init();
  juicer.portMode(MCP23017Port::A, 0);          //Port A as output
  juicer.portMode(MCP23017Port::B, 0b11111111); //Port B as input

  juicer.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A 
  juicer.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B

  // GPIO_B reflects the same logic as the input pins state
  juicer.writeRegister(MCP23017Register::IPOL_B, 0x00);
  // Uncomment this line to invert inputs (press a button to lit a led)
  //juicer.writeRegister(MCP23017Register::IPOL_B, 0xFF);


  signal.init();
  signal.portMode(MCP23017Port::A, 0);          //Port A as output
  signal.portMode(MCP23017Port::B, 0b11111111); //Port B as input

  signal.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A 
  signal.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B

  // GPIO_B reflects the same logic as the input pins state
  signal.writeRegister(MCP23017Register::IPOL_B, 0x00);
  // Uncomment this line to invert inputs (press a button to lit a led)
  //signal.writeRegister(MCP23017Register::IPOL_B, 0xFF);



// Example blink diode at P1
// LED diode between GND and P1

	pinMode(ex1, 1, OUTPUT);
	pinMode(ex2, 1, OUTPUT);




/*  check for setting the CVs to Factory Default during start and do so if yes  */
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


  displayText( true, false ); /* Shows the standard text if allowed */


  interrupts();  /* Ready to rumble....*/
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {


  uint8_t currentB;

  currentB = juicer.readPort(MCP23017Port::B);
  juicer.writePort(MCP23017Port::A, currentB);

  currentB = signal.readPort(MCP23017Port::B);
  signal.writePort(MCP23017Port::A, currentB);



	digitalToggle(ex1, 1);
	digitalToggle(ex2, 1);



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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*  function is doing an automatic reset of the processor after preScaler (time) elapsed  */
void softwareReset( uint8_t preScaler )
{
  wdt_enable( preScaler );

  while( 1 ) {}  // Wait for the prescaler time to expire and do an auto-reset
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





/* **********************************************************************************
***                  menu system for setting and dumping values                   ***
*************************************************************************************

first menu:
  FD         factory defaults all settings --> writes to CVs
  DD         lists all data on serial monitor
  DS x#      default switch (1 to 16) -->  display next menu
  RD xx#     read  data from CV address xx# (0 to 255)
  WD xx# x#  write data x# to CV address xx# (0 to 255)

next menu:
  c = center the default servo
  x = adjust servo  left x degrees at a time
  v = adjust servo right x degrees at a time
  j = invert juicer setting for this switch
  s = invert signal setting for this switch
  t = throw the switch (servo, juicer, signal)
  w = write the settings in corresponding CVs --> go back to first menu
  y = don't save setting, just go back to first menu

********************************************************************************** */
void displayText( bool first = true, bool next = false )
{

/*  check if we need to display this text  */
#if defined(_DEBUG_) || defined(TESTRUN)

  if( first = true )
  {
    _PL("");   // An extra empty line for better understanding
    _PL(F("**  put in one of the following commands: "     ));
    _PL(F("----------------------------------------------" ));
    _PL(F("FD  clear everything: Factory Default"          ));
    _PL(F("DD  prints CV values: to your monitor"          ));
    _PL("");
    _PL(F("DS  set default switch number (1-16): DS x#"    ));
    _PL("");
    _PL(F("RD  reads a configuration variable: RD xx#"     ));
    _PL(F("WD  write a configuration variable: WD xx# x#"  ));
    _PL(F("----------------------------------------------" ));
    _PL(F("** use [Enter] key when needed **"              ));
    _PL("");   // An extra empty line for better understanding
  }

  if( next  = true )
  {
    _PL("");   // An extra empty line for better understanding
    _PL(F("**  put in one of the following commands: "     ));
    _PL(F("----------------------------------------------" ));
    _PL(F("x = adjust servo  left x degrees at a time"     ));
    _PL(F("c = center the default servo"                   ));
    _PL(F("v = adjust servo right x degrees at a time"     ));
    _PL("");
    _PL(F("j = invert juicer setting for this switch"      ));
    _PL(F("s = invert signal setting for this switch"      ));
    _PL("");
    _PL(F("t = throw the switch (servo, juicer, signal)"   ));
    _PL(F("w = write the settings  in corresponding CVs"   ));
    _PL(F("----------------------------------------------" ));
    _PL(F("**  any other key discards settings  **"        ));
    _PL("");   // An extra empty line for better understanding
  }



#endif

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


void parseCom( char *com )
{
  switch ( com[1] )  // com[0] = '<' or ' '
  {


/****  CLEAR SETTINGS TO FACTORY DEFAULTS  ****/

    case 'C':     // <C>
/*
 *    clears settings to Factory Defaults
 *
 *    returns: <FD done> cvCheck
 */
      {
        uint8_t cvCheck = Dcc.setCV( NMRADCC_MASTER_RESET_CV, NMRADCC_MASTER_RESET_CV );

        _PP( "<FD done>"  );
        _2L( cvCheck, DEC );

        softwareReset(  WDTO_15MS  );
        break;
      }


/***** DUMPS CONFIGURATION VARIABLES FROM DECODER ****/

      case 'D':     // <D>
/*
 *    dumps all Configuration Variables from the decoder
 *
 *    returns a list of: <CV VALUE)
 *    where VALUE is a number from 0-255 as read from the CV, or -1 if read could not be verified
 */
      {
        FactoryDefaultCVIndex = sizeof( FactoryDefaultCVs ) / sizeof( CVPair );

        for (int i = 0; i < FactoryDefaultCVIndex; i++)
        {
          uint8_t cvValue = Dcc.getCV( FactoryDefaultCVs[ i ].CV );

          _PP( " cv: "                      );
          _2P( FactoryDefaultCVs[i].CV, DEC );
          _PP( "\t"              " value: " );
          _2L( cvValue,                 DEC );

        }
        break;
      }


/****  PRINT CARRIAGE RETURN IN SERIAL MONITOR WINDOW  ****/

      case ' ':     // < >
/*
 *    simply prints a carriage return - useful when interacting with Ardiuno through serial monitor window
 *
 *    returns: a carriage return and the menu text
 */
      {
        _PL("");

        displayText(); // Shows the standard explanation text

        break;
      }


      default:    /****  DEFAULT FOR THE SWITCH FUNCTION = NO ACTION  ****/
        break;
   }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/************************************************************************************
                        Call-back functions from DCC
************************************************************************************/

/*  TODO:  some of the following functions can be removed after testing with DCC  */


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


// deprecated, only for backward compatibility with version 1.4.2.
// don't use in new designs. These functions may be dropped in future versions.
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
  _PP( "notifyCVChange: CV: " );
  _2L( CV,                DEC );
  _PP( "Value: "              );
  _2L( Value,             DEC );

  if ( ( ( CV == 251 ) && ( Value == 251 ) ) || ( ( CV == 252 ) && ( Value == 252 ) ) )
  {
    wdt_enable( WDTO_15MS );  //  Resets after 15 milliSecs

    while ( 1 ) {} // Wait for the prescaler time to expire
  }

  //  /*  calculate the time setting for almost every output  */
  //  for(int i = 0; i < (int)(sizeof( run_switch_set ) ); ++i )
  //  {
  //     if ( i < 7 ) { calculateNeoQueue( i ); } else { calculateLedQueue( i ); }
  //  }

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



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/*  */

