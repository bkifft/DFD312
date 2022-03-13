#include <LiquidCrystal.h>  // https://github.com/arduino-libraries/LiquidCrystal
#include <SoftTimers.h>     // https://github.com/end2endzone/SoftTimers
#include <LTC1661.h>        // https://github.com/walle86/

#include "Button.h"
#include "usart.c"

#define VERSION   "0.95"

#define  PA7  31  // Line In R LowPass filtered
#define  PA6  30  //  mixed  Line In L / Mic/Remote LowPass filtered
#define  PA5  29  // Channel level B
#define  PA4  28  // Channel level A
#define  PA3  27  // 12V measurment 
#define  PA2  26  // V+ measurement
#define  PA1  25  // Multi-Adj VR3G$1, LINE1 IN
#define  PA0  24  // Measure output current

#define  PB7   7  // SCK DAC, pin2 /  ISP SCK JP1-3
#define  PB6   6  //  /  ISP MISO JP1-1
#define  PB5   5  // DIN DAC, pin3 /  ISP MOSI JP1-4
#define  PB4   4  // not connected
#define  PB3   3  // Output A Gate-
#define  PB2   2  // Output A Gate+
#define  PB1   1  // Output B Gate-
#define  PB0   0  // Output B Gate+

#define  PC7  23  // LCD_DB7 or Menu       J10-16
#define  PC6  22  // LCD_DB6 or RightUp    J10-14
#define  PC5  21  // LCD_DB5 or OK         J10-12
#define  PC4  20  // LCD_DB4 or LeftDown   J10-10
#define  PC3  19  // LCD_RS                J10-8
#define  PC2  18  // LCD_E                 J10-6
#define  PC1  17  // LCD_RW                J10-4
#define  PC0  16  // Activate Buttons

#define  PD7  15  // LCD Kathode
#define  PD6  14  // Output LED1
#define  PD5  13  // Output LED2
#define  PD4  12  // CS/LD DAC
#define  PD3  11  // Multi-Adj VRG3$1 Line In R 
#define  PD2  10  // Multi-Adj VRG3$2  mixed  Line In L / Mic/Remote 
#define  PD1  9  // Link wired T1In  Max232 
#define  PD0  8  // Link wired R1Out Max232

// Mode constances
// ---------------------------------------------------------------------
#define ClimbPulsTime   130   // 130 uSecond puls time in 'climb' mode
#define StrokePulsTime  240   // 240uSecond fixe puls time in 'stroke' mode
#define MinPulsDistance   6   // 6mSeconds repeat time for pulses, quicker does not work so far
#define IntensePulsTime 130   // 130uSeconds fix puls time for 'intense' mode
#define RythmPulsTime1   70
#define RythmPulsTime2  180
#define AudioPulsTime   130
#define TogglePulsTime  130

#define NormalPwrLevel         590   // power level settings, smaller values means more output power
#define LowPwrLevel            650   // power level settings, defines value when pot is on max
#define HighPwrLevel           400   // power level settings
#define NormalModulationFactor 330   // power level are modulated in certrain patterns, this is the modulation value in normal power mode
#define LowModulationFactor    270   // power level are modulated in certrain patterns, this is the modulation value in low power mode
#define HighModulationFactor   520   // power level are modulated in certrain patterns, this is the modulation value in high power mode

// Menu Definitions
// ---------------------------------------------------------------------
byte CurrentModeIX; // identifier for the currently active mode, 0.. NrModes-1
const char m1[] PROGMEM  = "Waves   ";
const char m2[] PROGMEM  = "Stroke  ";
const char m3[] PROGMEM  = "Climb   ";
const char m4[] PROGMEM  = "Combo   ";
const char m5[] PROGMEM  = "Intense ";
const char m6[] PROGMEM  = "Rhythm  ";
const char m7[] PROGMEM  = "Audio1  ";
const char m8[] PROGMEM  = "Audio2  ";
const char m9[] PROGMEM  = "Audio3  ";
const char m10[] PROGMEM = "Split   ";
const char m11[] PROGMEM = "Random1 ";
const char m12[] PROGMEM = "Random2 ";
const char m13[] PROGMEM = "Toggle  ";
const char m14[] PROGMEM = "Orgasm  ";
const char m15[] PROGMEM = "Torment ";
const char m16[] PROGMEM = "Phase1  ";
const char m17[] PROGMEM = "Phase2  ";
const char m18[] PROGMEM = "Phase3  ";

#define SplitModeIX 9
#define RandomMode1 10
#define RandomMode2 11


const char* const ModeStrings[18] PROGMEM = {m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13, m14, m15, m16, m17, m18};
#define NrModes 18  // nr. of Modes, used for cycling thru all Modes
#define NrSplitsModes 6  // Mode m1 .. m6 can be combined in 'split mode'

byte CurrentOptionIX;
const char o1[] PROGMEM = "Start Ramp Up?  ";
const char o2[] PROGMEM = "Set Split Mode? ";
const char o3[] PROGMEM = "Set As Favorite?";
const char o4[] PROGMEM = "Set Pwr Level?  ";
const char o5[] PROGMEM = "More Options?   ";
const char o6[] PROGMEM = "Save Settings?  ";
const char o7[] PROGMEM = "Reset Settings? ";
const char o8[] PROGMEM = "Adjust Advanced?";
const char* const OptionsStrings[8] PROGMEM = {o1, o2, o3, o4, o5, o6, o7, o8};
#define NrOptions 8  // nr. of Options for Options Menu

byte CurrentPwrLevelIX;
const char p1[] PROGMEM = "Pwr Lev: Normal ";
const char p2[] PROGMEM = "Pwr Lev: Low    ";
const char p3[] PROGMEM = "Pwr Lev: High   ";
const char* const PwrLevelStrings[3] PROGMEM = {p1, p2, p3};
#define NrPwrLevels 3  // nr. of Options for Options Menu

byte MoreOptionsIX;
const char r1[] PROGMEM = "Random3 Mode ?  ";
const char r2[] PROGMEM = "Random4 Mode ?  ";
const char r3[] PROGMEM = "Debug Mode ?    ";
const char r4[] PROGMEM = "Link Slave Unit?";
const char r5[] PROGMEM = "Config Bluetooth";
const char* const MoreOptionsStrings[5] PROGMEM = {r1, r2, r3, r4, r5};
#define NrMoreOptions 5 // nr. of Options for Options Menu
byte SplitIX;
const char s1[] PROGMEM = "Split:A         ";
const char s2[] PROGMEM = "Split:B         ";
const char* const SplitStrings[2] PROGMEM = {s1, s2};
#define NrSplits 2 // nr. of Options for Options Menu

byte AdvancedIX;
const char a1[] PROGMEM = "RampLevl Adjust?";
const char a2[] PROGMEM = "RampTime Adjust?";
const char a3[] PROGMEM = "   Depth Adjust?";
const char a4[] PROGMEM = "   Tempo Adjust?";
const char a5[] PROGMEM = "   Freq. Adjust?";
const char a6[] PROGMEM = "  Effect Adjust?";
const char a7[] PROGMEM = "   Width Adjust?";
const char a8[] PROGMEM = "     PaceAdjust?";
const char* const AdvancedStrings[8] PROGMEM = {a1, a2, a3, a4, a5, a6, a7, a8};
#define NrAdvanced 8 // nr. of Options for Options Menu

const char h1[] PROGMEM = "AT\r\n\0";
const char h2[] PROGMEM = "AT+VERSION?\r\n\0";
const char h3[] PROGMEM = "AT+NAME=DFD312V095\r\n\0";
const char h4[] PROGMEM = "AT+PSWD=1234\r\n\0";
const char h5[] PROGMEM = "AT+POLAR=0,0\r\n\0";
const char h6[] PROGMEM = "AT+IPSCAN=1024,1,1024,1\r\n\0";
const char h7[] PROGMEM = "AT+RESET\r\n\0";
const char h8[] PROGMEM = "                         \0";

const char* const ATStrings[8] PROGMEM = {h1, h2, h3, h4, h5, h6, h7, h8};
#define NrAT 7 // nr. of Options for Options Menu

char  LineBuffer[30]; // ram buffer for copying progmem to LCD
char LineBuffer1[30];

// some menues operate the same way. We keep pointers to these menues in an array for easy reference
// function prototypes, leave these be :) 
byte CurrentMenuIX;
void      ModeMenu       (byte f);
void      OptionsMenu    (byte f);
void      SplitModeMenu  (byte f);
void      PowerLevelMenu (byte f);
void      MoreOptionsMenu(byte f);
void      AdvancedMenu   (byte f);

void (*MenuFunctions[6])(byte f) = {
          ModeMenu,
          OptionsMenu,
          SplitModeMenu,
          PowerLevelMenu,
          MoreOptionsMenu,
          AdvancedMenu
        };
// to be called as (*MenuFunctions[ix](funParam);
#define   NrMenus 6  // nr. of  Menus

byte pd5Output;
byte pd6Output;

boolean SetSplitA; // used for setting the split modes

#define defaultRampUpTime 50   // default for 1/100 of ramp up time in ms
uint8_t currentRampUpTime;
boolean RampUpOn;              // is true during ramp-up phase
uint8_t RampUpCounter;

byte BurstCounter[2];
byte BurstQuadrant[2];
volatile byte GatePulsCount[2];   // these are modified in the ISR, therefore 'volatile' declaration

byte WaveCounter[2];
byte WaveQuadrant[2];

byte CurrentMode[2];    // holds the mode index per channel A/B

uint16_t ChannelAPwrBase; // value for maximum output power, smaller=more power, 
uint16_t ChannelBPwrBase; // start with 600, 500=max power, 800=min power
uint16_t ChannelAModulationBase;    // 300 .. 600, corresponds to 0.3..0.6 as it is diveded by 1024
uint16_t ChannelBModulationBase;    // 300 .. 600, corresponds to 0.3..0.6 as it is diveded by 1024
uint16_t ChannelAModulationFactor;  // 0.. 50, small changes in function of time 
uint16_t ChannelBModulationFactor;  // 0 .. 50, corresponds to 0.3..0.6 as it is diveded by 1024
uint16_t RampUpPowerIncA;
uint16_t RampUpPowerIncB;

uint16_t  MAInputLevel; // input from MA Adjust potentiometer, 0..1.8Volt, scaled to 0..100
uint16_t  MAInput;      // raw analog input value on MA potentiometer port
uint16_t  EnveloppeSpeed;  // calculated out of the MA Adjust value, used for modulation in wave etc.
uint16_t  EnveloppeSpeedm1; // the EnveloppeSpeed from the previous measuring cycle


// End Menu Definitions
// ---------------------------------------------------------------------

// Timer Variables
// -------------------------------------------------------------------------------------------------------

SoftTimer Enveloppe[2];  // millisecond timer for enveloppe of channel A and B
SoftTimer RandomModeTimer;  // used to time the slow changes in the random modes
SoftTimer RampUpTimer;      // used to time the ramp up during mode changes
SoftTimer PwrModulation[2]; // overall output power modulation

// -------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------
// data tables for predefined signals
// some functions with 256 values per quadrant, e.g. 1024 for a full cycle
//  linear ramp up 0..52 -> 0 .. 255


const byte sineTable[] PROGMEM = {
0 ,0 ,1 ,1 ,2 ,2 ,3 ,4 ,4 ,5 ,5 ,6 ,7 ,7 ,8 ,8 ,9 ,10 ,10 ,11 ,11 ,12 ,12 ,13 ,14 ,14 ,15 ,15 ,16 ,17 ,17 ,18 ,
18 ,19 ,19 ,20 ,21 ,21 ,22 ,22 ,23 ,23 ,24 ,25 ,25 ,26 ,26 ,27 ,27 ,28 ,29 ,29 ,30 ,30 ,31 ,31 ,32 ,32 ,33 ,34 ,34 ,35 ,35 ,36 ,
36 ,37 ,37 ,38 ,38 ,39 ,40 ,40 ,41 ,41 ,42 ,42 ,43 ,43 ,44 ,44 ,45 ,45 ,46 ,46 ,47 ,47 ,48 ,48 ,49 ,49 ,50 ,50 ,51 ,51 ,52 ,52 ,
53 ,53 ,54 ,54 ,55 ,55 ,56 ,56 ,57 ,57 ,58 ,58 ,59 ,59 ,60 ,60 ,61 ,61 ,61 ,62 ,62 ,63 ,63 ,64 ,64 ,65 ,65 ,65 ,66 ,66 ,67 ,67 ,
67 ,68 ,68 ,69 ,69 ,70 ,70 ,70 ,71 ,71 ,72 ,72 ,72 ,73 ,73 ,73 ,74 ,74 ,75 ,75 ,75 ,76 ,76 ,76 ,77 ,77 ,77 ,78 ,78 ,78 ,79 ,79 ,
79 ,80 ,80 ,80 ,81 ,81 ,81 ,82 ,82 ,82 ,83 ,83 ,83 ,83 ,84 ,84 ,84 ,85 ,85 ,85 ,85 ,86 ,86 ,86 ,86 ,87 ,87 ,87 ,87 ,88 ,88 ,88 ,
88 ,88 ,89 ,89 ,89 ,89 ,90 ,90 ,90 ,90 ,90 ,91 ,91 ,91 ,91 ,91 ,91 ,92 ,92 ,92 ,92 ,92 ,92 ,93 ,93 ,93 ,93 ,93 ,93 ,93 ,93 ,94 ,
94 ,94 ,94 ,94 ,94 ,94 ,94 ,94 ,94 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95 ,95
};
inline byte sine(byte x) {
  return pgm_read_byte(&sineTable[x]);
}

const byte linearTable[] PROGMEM = {
0 ,0 ,0 ,1 ,1 ,1 ,2 ,2 ,3 ,3 ,3 ,4 ,4 ,4 ,5 ,5 ,6 ,6 ,6 ,7 ,7 ,7 ,8 ,8 ,9 ,9 ,9 ,10 ,10 ,10 ,11 ,11 ,
12 ,12 ,12 ,13 ,13 ,13 ,14 ,14 ,15 ,15 ,15 ,16 ,16 ,16 ,17 ,17 ,18 ,18 ,18 ,19 ,19 ,19 ,20 ,20 ,21 ,21 ,21 ,22 ,22 ,22 ,23 ,23 ,
24 ,24 ,24 ,25 ,25 ,25 ,26 ,26 ,27 ,27 ,27 ,28 ,28 ,28 ,29 ,29 ,30 ,30 ,30 ,31 ,31 ,31 ,32 ,32 ,33 ,33 ,33 ,34 ,34 ,34 ,35 ,35 ,
36 ,36 ,36 ,37 ,37 ,37 ,38 ,38 ,39 ,39 ,39 ,40 ,40 ,40 ,41 ,41 ,42 ,42 ,42 ,43 ,43 ,43 ,44 ,44 ,45 ,45 ,45 ,46 ,46 ,46 ,47 ,47 ,
48 ,48 ,48 ,49 ,49 ,49 ,50 ,50 ,51 ,51 ,51 ,52 ,52 ,52 ,53 ,53 ,54 ,54 ,54 ,55 ,55 ,55 ,56 ,56 ,57 ,57 ,57 ,58 ,58 ,58 ,59 ,59 ,
60 ,60 ,60 ,61 ,61 ,61 ,62 ,62 ,63 ,63 ,63 ,64 ,64 ,64 ,65 ,65 ,66 ,66 ,66 ,67 ,67 ,67 ,68 ,68 ,69 ,69 ,69 ,70 ,70 ,70 ,71 ,71 ,
72 ,72 ,72 ,73 ,73 ,73 ,74 ,74 ,75 ,75 ,75 ,76 ,76 ,76 ,77 ,77 ,78 ,78 ,78 ,79 ,79 ,79 ,80 ,80 ,81 ,81 ,81 ,82 ,82 ,82 ,83 ,83 ,
84 ,84 ,84 ,85 ,85 ,85 ,86 ,86 ,87 ,87 ,87 ,88 ,88 ,88 ,89 ,89 ,90 ,90 ,90 ,91 ,91 ,91 ,92 ,92 ,93 ,93 ,93 ,94 ,94 ,94 ,95 ,95
}; 
inline byte linear(byte x) {
  return pgm_read_byte(&linearTable[x]);
}



// wave form functions for burst (High Frequency)
// ----------------------------------------------

byte burstFunction01(byte ix, byte quadrant) {
  // linear ramp function, 255 -> 511 -> 255 -> 0 -> 255
  switch(quadrant) {
    case 0: {
    return 96 + linear(ix);
    break;
    }
    case 1: {
    return 96 + linear(255 - ix);
    break;
    }
    case 2: {
    return 96 - linear(ix);
    break;
    }
    case 3: {
    return 96 - linear(255 - ix);
    break;
    }
  break;
  }
}


byte burstFunction21(byte ix, byte quadrant, byte AMax, byte tMax) {
    // linear zig-zag function, calculated, returns 0..255 for ix in 0..255
  switch(quadrant) {
    case 0: {
      return AMax/2 +     (AMax*ix/2)/tMax;  // AMax/2 .. AMax
      break;
    }
    case 1: {
      return   AMax -    (AMax*ix/2)/tMax; // AMax .. AMax/2
      break;
    }
    case 2: {
      return AMax/2 - (AMax*ix/2)/tMax;  // AMax/2 .. 0
      break;
    }
    case 3: {
    return    0  + (AMax*ix/2)/tMax;  // 0 .. AMax/2
    break;
    }
  break;
  }
}


byte burstFunction32(byte ix, byte quadrant, byte AMax, byte tMax) {
    // linear zig-0 (linear ramp), slow to fast,  function, calculated, returns 0..AMax  for ix in 0..tMax
      return AMax - (AMax*ix)/tMax; // AMax .. 0
}

byte burstFunction02(byte ix, byte quadrant) {
    // sine function
  switch(quadrant) {
    case 0: {
    return 96 + sine(ix);  //128 .. 255
    break;
    }
    case 1: {
    return 96 + sine(255 - ix); // 255 .. 128
    break;
    }
    case 2: {
    return 96 - sine(ix);  // 128 ..1
    break;
    }
    case 3: {
    return 96 - sine(255 - ix); // 1 .. 128
    break;
    }
  break;
  }
}

   
// -------------------------------------------------------------------------------------------------------


// LCD Displace Definitions
// ---------------------------------------------------------------------

// make some custom characters:
byte heart[8] = {
  0b00000000,
  0b00001010,
  0b00011111,
  0b00011111,
  0b00011111,
  0b00001110,
  0b00000100,
  0b00000000
};

byte arrowRight[8] = {
  0b00001000,
  0b00000100,
  0b00000010,
  0b00011111,
  0b00011111,
  0b00000010,
  0b00000100,
  0b00001000
};

byte arrowLeft[8] = {
  0b00000010,
  0b00000100,
  0b00001000,
  0b00011111,
  0b00011111,
  0b00001000,
  0b00000100,
  0b00000010
};


// instantinate the D/A converter
//LTC1661 dac(PD4, PB5, PB7);    //creats an instance with DLTC1661(CS, DIN, SDK)
LTC1661 dac(PD4);    //creats an instance with DLTC1661(CS) with Arduino SPI

// instantinate the LCD display
// -----------------------------
// LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
LiquidCrystal lcd(PC3, PC2, PC4, PC5, PC6, PC7);

// END LCD Displace Definitions
// ---------------------------------------------------------------------


// Generic Functions
// ---------------------------------------------------------------------

int ReadAndUpdateChannel(byte c) {
  // reads the value of the pot meter (10bit) and returns a byte 0..99
  int v;  // the 10 bit value read from the A/D port
  byte bv; 
  
  if (c == 1) {
    v = analogRead(PA5);
    bv = min(99, max(0,(byte)trunc(v/8))); 
 //   SetChannel(c, ChannelAPwrBase +((1023 - v)/3));
    SetChannel(c, ChannelAPwrBase + ChannelAModulationFactor + (uint32_t (ChannelAModulationBase) * uint32_t(1023 - v))/1024);

  } else {
    v = analogRead(PA4);
    bv = min(99, max(0,(byte)trunc(v/8))); 
 //   SetChannel(c, ChannelBPwrBase +((1023 - v)/3));
     SetChannel(c, ChannelBPwrBase + ChannelBModulationFactor + (uint32_t(ChannelBModulationBase) * uint32_t(1023 - v))/1024);

  }
  return bv;
}

void SetChannel(byte c, uint16_t val) {
  // sets the DC output level of channel 'c' to value 'val' (0..1023)
   if (c == 1) {
    dac.loUpCH(0, (int)val);
  } else {
    dac.loUpCH(1, (int)val);
  }
}

boolean odd(uint32_t i) {
 if ((i % 2) == 0) { return true;} else { return false;}
}

byte CycleIX(byte IX, byte maxIX, boolean ForBack) {
  // rotates the index IX in the boundaries 0..maxIX, forward (true) or backward (false)
  byte i;
  
  if (ForBack) { // modulo increment
   i = IX + 1;
   if (i == maxIX) { i = 0;}
  } else { // modulo decrement
   if (IX == 0) { i = maxIX - 1;} else {i = IX - 1;}
  }
  return i;
}

// End Generic Functions
// ---------------------------------------------------------------------



// Timer Functions
// ---------------------------------------------------------------------
// timer0 and timer2 are 8bit, timer1 is 16 with A/B compare interrupts

 void SetupTimer1() {
  //set timer1 (16bit timer) A and B for interrupts in the range 245Hz(OCR2A=255 ... 62.5KHz(OCR2A=1)
  cli();//stop interrupts

  TCNT1  = 0;//initialize counter value to 0
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);                             // turn on CTC mode on Timer1B
  TCCR1B |= (1 << CS11); // set 8 prescalar on Timer1B
  // set compare match registers on Timer2/A and Timer2/B  for 8khz increments
  OCR1A = 0;      // 255 = 30us ISR call frequency = (8*10^6) / (f*1024) - 1 (must be < 65536)
  // enable timer compare interrupt for Timer2/A and Timer2/B
  TIMSK |= (1 << OCIE1A);  
  sei();//allow interrupts
}

void SetupTimer2() {
  // Timer 2 for high frequency
  cli();//stop interrupts
  TCNT2  = 0;//initialize counter value to 0
  TCCR2 |= (1 << WGM21); // turn on CTC mode on Timer2
  TCCR2 = 0b00001010;  // prescalar 8
  OCR2 = 0;      // 255=255us,100=100us,50=50us, 10=20us, 1=20us < 256, 0.5MHz, set compare value for A, defines ISR call frequency = (8*10^6) / (f*1024) - 1 (must be < 65536)
  TIMSK |= (1 << OCIE2);  // enable Timer1A compare interrupt
  sei();  
}

// Timer a A/B ISR: low  frequency signal generation
// ---------------------------------------------
ISR(TIMER1_COMPA_vect){  // generates usecond puls for channel A
//PB2 = R37/38, yello
// parameter are given by global variables GatePulsCount[] and the timer registers
// length of a puls is controlled by the value in OCR1A
// depending on GatePulsCount different pulses are generated:
// GatePulsCount = 2 => a positive puls followed by an equally length negative puls
// GatePulsCount = 1 => just a negative puls of length controlled by OCR1A 
// GatePulsCount = 3 => just  positive puls of a length controlleb by OCR1A 
   switch (GatePulsCount[0]) {
     case 3: {
               PORTB = PORTB | B00000100;  // set PB2 HIGH, fast version
               PORTB = PORTB & B11110111;  // set PB3 LOW, fast version
               GatePulsCount[0] = 0;
               break;
              }
     case 2: {
               PORTB = PORTB | B00000100;  // set PB2 HIGH, fast version
               PORTB = PORTB & B11110111;  // set PB3 LOW, fast version
               GatePulsCount[0] = 1;
               break;
              }
     case 1:  {
               PORTB = PORTB | B00001000;  // set PB3 HIGH, fast version
               PORTB = PORTB & B11111011;  // set PB2 LOW, fast version 
               GatePulsCount[0] = 0;
               break;
              }
     default: {
               TIMSK &= ~(1 << OCIE1A);
               PORTB = PORTB & B11111011;  // set PB2 LOW, fast version
               PORTB = PORTB & B11110111;  // set PB3 LOW, fast version
               GatePulsCount[0] = 0;
               break;
     }
  break;
  }
}

ISR(TIMER2_COMP_vect){ // generates uSecond puls for channel B
//PB2 = R37/38, yello
// parameter are given by global variables GatePulsCount[] and the timer registers
// length of a puls is controlled by the value in OCR2
// depending on GatePulsCount different pulses are generated:
// GatePulsCount = 2 => a positive puls followed by an equally length negative puls
// GatePulsCount = 1 => just a negative puls of length controlled by OCR2 
// GatePulsCount = 3 => just  positive puls of a length controlleb by OCR2
   switch (GatePulsCount[1]) {
     case 3: {
              PORTB = PORTB & B11111101;  // set PB2 LOW, fast version
              PORTB = PORTB | B00000001;  // set PB1 HIGH, fast version
              GatePulsCount[1] = 0;
              break;
              }
     case 2: {
              PORTB = PORTB & B11111101;  // set PB2 LOW, fast version
              PORTB = PORTB | B00000001;  // set PB1 HIGH, fast version
              GatePulsCount[1] = 1;
              break;
              }
     case 1:  {
              PORTB = PORTB & B11111110;  // set PB1 LOW, fast version
              PORTB = PORTB | B00000010;  // set PB2 HIGH, fast version
              GatePulsCount[1] = 0;
              break;
              }
     default: {
               TIMSK &= ~(1 << OCIE2);
               PORTB = PORTB & B11111110;  // set PB1 LOW, fast version
               PORTB = PORTB & B11111101;  // set PB2 LOW, fast version
               GatePulsCount[1] = 0;
               break;
     }
  break;
  }
}


// End Timer Functions
// ---------------------------------------------------------------------


// Functions to interface the 4 push-down buttons
// ---------------------------------------------------------------------
// buttons share the same lines as the 4 data lines of the LCD, e.g. are multiplexed
// to update the LCD, the buttons need to be disabled

// Button objects instantiation
const bool Mypullup = true;
Button  left(PC4, Mypullup);
Button right(PC6, Mypullup);
Button    ok(PC5, Mypullup);
Button  menuB(PC7,Mypullup);

void EnableButtons() {
  pinMode(PC4,INPUT_PULLUP);
  pinMode(PC5,INPUT_PULLUP);
  pinMode(PC6,INPUT_PULLUP);
  pinMode(PC7,INPUT_PULLUP);
  digitalWrite(PC0, HIGH);  // HIGH signal on PC0 closes the 4066 switches and connects the buttons to the ATMega16 pins
}
void DisableButtons() { 
  pinMode(PC4,OUTPUT);
  pinMode(PC5,OUTPUT);
  pinMode(PC6,OUTPUT);
  pinMode(PC7,OUTPUT);
  digitalWrite(PC0, LOW);  // LOW signal on PC0 opens the 4066 switches and isolates  the buttons of the ATMega16 pins
}

// Ende Functions to interface the 4 push-down buttons
// ---------------------------------------------------------------------

// Menu Interraction
// ---------------------------------------------------------------------

void RunningLine1() {
  byte va, vb;
  va = ReadAndUpdateChannel(0); vb = ReadAndUpdateChannel(1);
  // PA1 is 0..340
  MAInput = analogRead(PA1); // read the MA potentiometer, 0..1.8V is on PA1 => analogRead returns 0..370
  MAInputLevel = (400 - MAInput)/2; // read the MA potentiometer, 0..1.8V is on PA1 => analogRead returns 0..370, scale it to ca. 0..100 
  EnveloppeSpeed = CalcEnveloppeSpeed(MAInputLevel);
  DisableButtons();
  lcd.setCursor(0, 0);  lcd.print("A");
  lcd.setCursor(1, 0); if (va < 10) { lcd.print(" ");} lcd.print(va); lcd.print(" ");
  lcd.setCursor(4, 0);  lcd.print("B");
  lcd.setCursor(5, 0); if (vb < 10) { lcd.print(" ");} lcd.print(vb); lcd.print(" ");
  EnableButtons();
}

void SetMode(byte m) {
  byte mm;
  
  mm = min(max(0,m),(NrModes - 1));  // make sure that Mode buffer not overflows
  RunningLine1();  // shows line 1 which is the same for start-up and operation
  strcpy_P(LineBuffer, (char*)pgm_read_word(&(ModeStrings[mm]))); // Necessary casts and dereferencing, just copy. 
  DisableButtons();
  lcd.setCursor(8, 0); lcd.print(LineBuffer);
  if (RampUpOn) { // second LCD line looks different during ramp-up phase
    lcd.setCursor(0,1); lcd.write("Ramp:           "); // clear line
    lcd.setCursor(9,1); lcd.write(byte(1)); lcd.write(byte(2)); lcd.write("=Mode");
  } else { // normal second line 
    if ((mm == SplitModeIX) ||
        (mm == RandomMode1) ||
        (mm == RandomMode2)) {  // split/random modes: add extra info on line 1
      lcd.setCursor(14, 0); lcd.print(CurrentMode[0]); lcd.print(CurrentMode[1]);
    }
    lcd.setCursor(0,1);  lcd.write(" "); lcd.setCursor(0,1); lcd.write(byte(1));  
    lcd.setCursor(1,1);  lcd.write(" ");
    lcd.setCursor(2,1);  lcd.write(" "); lcd.setCursor(2,1); lcd.write(byte(2)); 
    lcd.setCursor(3,1);  lcd.print(" Select Mode ");
  }
  EnableButtons();
}

void PrepareMode(byte m) {
   byte mm,i;
   uint16_t tmp;
   
  RampUpOn = true;
  RampUpCounter = 0;
  // during the Ramp Up phase, the power starts with  1/4 less than the current setting and slowly moves to the current setting
  // lowest to be expected level of Powernumber is 355 (= maximum high power). 1/4 of that is ca. 90
  tmp = (ChannelAPwrBase + ChannelAModulationFactor + ((ChannelAModulationBase) * uint32_t(1023 - analogRead(PA5)))/1024)/4;
  RampUpPowerIncA = max(1,tmp/100);  // there are 100 inc steps in the Ramp-Up phase
  ChannelAModulationFactor = tmp;
  tmp = (ChannelBPwrBase + ChannelBModulationFactor + ((ChannelBModulationBase) * uint32_t(1023 - analogRead(PA4)))/1024)/4;
  RampUpPowerIncB = max(1,tmp/100);
  ChannelBModulationFactor = tmp;

  RampUpTimer.setTimeOutTime(currentRampUpTime);
  RampUpTimer.reset();

}

void StartMenu() {
  DisableButtons();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DFD312 by Martin");
  EnableButtons();
  SelftestMenu();
  // just blink the LEDs for degugging reasons
  digitalWrite(PD5, LOW);
  digitalWrite(PD6, LOW);
  BatteryMenu();
  DisableButtons();
  lcd.clear();
  EnableButtons();
  SetMode(CurrentModeIX);
}

// to be coded
//-------------
void SaveFavorites() {
  // not yet implemented
  // saves the current favorites in EEPROM
  DisableButtons();
  lcd.clear(); lcd.print("Call Save Fav");
  lcd.setCursor(0,1); lcd.write("Press     or OK "); // static text mask
  lcd.setCursor(6,1); lcd.write(byte(1)); lcd.setCursor(8,1); lcd.write(byte(2));
  EnableButtons();
  delay(1000);
}

void SaveSettings() {
  // not yet implemented
  // saves the current state in EEPROM
    DisableButtons();
  lcd.clear(); lcd.print("Call Save Set");
  lcd.setCursor(0,1); lcd.write("Press     or OK "); // static text mask
  lcd.setCursor(6,1); lcd.write(byte(1)); lcd.setCursor(8,1); lcd.write(byte(2));
  EnableButtons();
  delay(1000);
}

void ResetSettings() {
  // not yet implemented
  // resets the current settings
    DisableButtons();
  lcd.clear(); lcd.print("Call Rst Set");
  lcd.setCursor(0,1); lcd.write("Press     or OK "); // static text mask
  lcd.setCursor(6,1); lcd.write(byte(1)); lcd.setCursor(8,1); lcd.write(byte(2));
  EnableButtons();
  delay(1000);
}

void SetAdvanced(byte ButtonCode) {
  // not yet implemented
  DisableButtons();
  lcd.clear(); lcd.print("Call Set Ad"); lcd.print(ButtonCode);
  lcd.setCursor(0,1); lcd.write("Press     or OK "); // static text mask
  lcd.setCursor(6,1); lcd.write(byte(1)); lcd.setCursor(8,1); lcd.write(byte(2));
  EnableButtons();
  delay(1000);
}

void SetPowerLevel(byte ButtonCode) {
  DisableButtons();
  lcd.clear(); lcd.print("Call Set PwrLv"); lcd.print(ButtonCode);
  lcd.setCursor(0,1); lcd.write("Press     or OK "); // static text mask
  lcd.setCursor(6,1); lcd.write(byte(1)); lcd.setCursor(8,1); lcd.write(byte(2));
  switch (ButtonCode) {
    case 0: { // set level to 'normal'
               ChannelAModulationBase = NormalModulationFactor;
               ChannelBModulationBase = NormalModulationFactor;
               ChannelAPwrBase =        NormalPwrLevel; 
               ChannelBPwrBase =        NormalPwrLevel;
           break;
         }
    case 1: { // set level to 
               ChannelAModulationBase = LowModulationFactor;
               ChannelBModulationBase = LowModulationFactor; 
               ChannelAPwrBase =        LowPwrLevel; 
               ChannelBPwrBase =        LowPwrLevel;

           break;
         }
    case 2: { 
               ChannelAModulationBase = HighModulationFactor;
               ChannelBModulationBase = HighModulationFactor;
               ChannelAPwrBase =        HighPwrLevel; 
               ChannelBPwrBase =        HighPwrLevel;

           break;
         }
      break;
  }
  EnableButtons();
  delay(500);
}

void SetMoreOptions(byte ButtonCode) {
  // not yet implemented
  DisableButtons();
  lcd.clear(); lcd.print("Call Set MrOp"); lcd.print(ButtonCode);
  lcd.setCursor(0,1); lcd.write("Press     or OK "); // static text mask
  lcd.setCursor(6,1); lcd.write(byte(1)); lcd.setCursor(8,1); lcd.write(byte(2));
  EnableButtons();
  delay(1000);
}



void BTInit(byte ButtonCode) {
  // see https://wiki.iteadstudio.com/Serial_Port_Bluetooth_Module_(Master/Slave)_:_HC-05#Download for the description
  // of the AT commands that the HC-05 understands
  byte i;
   DisableButtons();
    
   lcd.clear(); lcd.print("BT Setup");
   UART_init(38400);

  for (i=0; i<NrAT; i++) {
//     strcpy_P(LineBuffer, (char*)pgm_read_word(&(ATStrings[8]))); // Necessary casts and dereferencing, just copy. 
     strcpy_P(LineBuffer, (char*)pgm_read_word(&(ATStrings[i]))); // Necessary casts and dereferencing, just copy. 
     lcd.clear(); lcd.print(LineBuffer);
     UART_SendString(LineBuffer);
//     strcpy_P(LineBuffer, (char*)pgm_read_word(&(ATStrings[8]))); // Necessary casts and dereferencing, just copy. 
     UART_ReceiveString(LineBuffer);  // data = Serial.read();
     lcd.setCursor(0,1); lcd.write(LineBuffer);
     delay(8000);
  }
  lcd.clear();        lcd.print("Bluetooth ..");
  lcd.setCursor(0,1); lcd.print(".. configured");
  delay(2000);
   
   EnableButtons();
}

void ShowDebug(byte ButtonCode) {
  // used to display certain variables
  byte i;
  DisableButtons();
  lcd.clear(); lcd.print("MA: ");lcd.print(MAInputLevel);
  lcd.setCursor(0,1); lcd.print("EnvSpd: "); lcd.print(EnveloppeSpeed);
  delay(1000);
  DisableButtons();
  /*
  lcd.clear(); lcd.print("DAC Test");
  for (i=0; i<64; i++) {
    lcd.setCursor(0,1); lcd.write("   "); lcd.setCursor(0,1); 
    lcd.print(ChannelAPwrBase + ChannelAModulationFactor + ((ChannelAModulationBase ) * uint32_t(1023 - analogRead(PA5)))/1024);
    lcd.setCursor(8,1); lcd.write("   "); lcd.setCursor(8,1); 
    lcd.print(ChannelBPwrBase + ChannelBModulationFactor + ((ChannelBModulationBase ) * uint32_t(1023 - analogRead(PA4)))/1024);
    // 540 .. 896 power value displayed
  //  TestDAC();
    delay(1000);
  }
  */
  EnableButtons();
  delay(1000);
  
  // do the bluetooth initialization
 // BTInit();
}


// END to be coded
//-------------

void OptionsMenu(byte ButtonCode) {
    // 1 : forward
    // 2 : backward
    // 4 : ok
    // 8 : menu
    RampUpOn = false;  // stop ramp-up in case its still running
    switch (ButtonCode) {
      case 1: { // foreard
            CurrentOptionIX = CycleIX(CurrentOptionIX, NrOptions, true);
            strcpy_P(LineBuffer, (char*)pgm_read_word(&(OptionsStrings[CurrentOptionIX]))); // Necessary casts and dereferencing, just copy. 
            DisplayMenu(LineBuffer);
            break;
        }
      case 2: { // backward
            CurrentOptionIX = CycleIX(CurrentOptionIX, NrOptions, false);
            strcpy_P(LineBuffer, (char*)pgm_read_word(&(OptionsStrings[CurrentOptionIX]))); // Necessary casts and dereferencing, just copy. 
            DisplayMenu(LineBuffer);
            break;
        }
       case 4: { // ok
             switch (CurrentOptionIX) {
                 case 0: { // go back to normal operations, Start Ramp Up
                           CurrentMenuIX = 0;
                           PrepareMode(CurrentModeIX);
                           SetMode(CurrentModeIX);
                       break;
                  }
                 case 1: { // Set Split Mode
                       CurrentMenuIX = 2; SplitIX = 0;
                       SplitModeMenu(0);
                       break;
                  }             
                 case 2: { // Set As Favorite
                           SaveFavorites();
                           CurrentMenuIX = 0;
                           PrepareMode(CurrentModeIX);
                           SetMode(CurrentModeIX);
                       break;
                  }
                 case 3: { // Set Power Level
                      CurrentMenuIX = 3; CurrentPwrLevelIX = 0;
                      PowerLevelMenu(0);
                       break;
                  }
                 case 4: { // More Options
                       CurrentMenuIX = 4; MoreOptionsIX = 0;
                       MoreOptionsMenu(0);
                       break;
                  }
                 case 5: { // Save Settings
                           SaveSettings();
                           CurrentMenuIX = 0;
                           PrepareMode(CurrentModeIX);
                           SetMode(CurrentModeIX);
                       break;
                  }             
                 case 6: { // Rest Settings
                           ResetSettings();
                           CurrentMenuIX = 0;
                           PrepareMode(CurrentModeIX);
                           SetMode(CurrentModeIX);
                       break;
                  }
                 case 7: { // Adjust Advanced
                       CurrentMenuIX = 5;
                       AdvancedMenu(0);
                       break;
                  }                                                    
                break; 
             }// switch (CurrentOptionIX)           
       break;} // end ok
       case 8 : { // menu button pressed, just go back to the main operations            
            CurrentMenuIX = 0;
            PrepareMode(CurrentModeIX);
            SetMode(CurrentModeIX);              
            break;
         }
        case 0: { // called this function just for updating LCD, e.g. do nothing
            strcpy_P(LineBuffer, (char*)pgm_read_word(&(OptionsStrings[CurrentOptionIX]))); // Necessary casts and dereferencing, just copy. 
            DisplayMenu(LineBuffer);
            break;
        }
       default : {
              CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
              OptionsMenu(0); 
              break;
       }
     break;         
  }
}

void SplitModeMenu(byte ButtonCode) {
   // 0: just update display
   // 1 : forward
   // 2 : backward
   // 4 : ok
   // 8 : menu

    switch (ButtonCode) {
      case 1: { // foreard
            SplitIX = CycleIX(SplitIX, NrSplitsModes, true);
            if (SetSplitA) {
                 strcpy_P(LineBuffer, (char*)pgm_read_word(&(SplitStrings[0]))); // Necessary casts and dereferencing, just copy. 
            } else {
                 strcpy_P(LineBuffer, (char*)pgm_read_word(&(SplitStrings[1]))); // Necessary casts and dereferencing, just copy. 
            }
            strcpy_P(LineBuffer1, (char*)pgm_read_word(&(ModeStrings[SplitIX]))); // Necessary casts and dereferencing, just copy. 
            LineBuffer[8] = '\0';
            strcat(LineBuffer, LineBuffer1);
            DisplayMenu(LineBuffer);
            break;
        }
      case 2: { // backward
            SplitIX = CycleIX(SplitIX, NrSplitsModes, false);
            if (SetSplitA) {
                 strcpy_P(LineBuffer, (char*)pgm_read_word(&(SplitStrings[0]))); // Necessary casts and dereferencing, just copy. 
            } else {
                 strcpy_P(LineBuffer, (char*)pgm_read_word(&(SplitStrings[1]))); // Necessary casts and dereferencing, just copy. 
            }
            strcpy_P(LineBuffer1, (char*)pgm_read_word(&(ModeStrings[SplitIX]))); // Necessary casts and dereferencing, just copy. 
            LineBuffer[8] = '\0';
            strcat(LineBuffer, LineBuffer1);
            DisplayMenu(LineBuffer);;
            break;
        }
      case 4: { // ok
                  if (SetSplitA) { 
                       CurrentMode[0] = SplitIX;
                       SplitIX = 0;
                       SetSplitA = false; // go now and set mode for channel B
                       strcpy_P(LineBuffer, (char*)pgm_read_word(&(SplitStrings[1]))); // Necessary casts and dereferencing, just copy. 
                       strcpy_P(LineBuffer1, (char*)pgm_read_word(&(ModeStrings[SplitIX]))); // Necessary casts and dereferencing, just copy. 
                       LineBuffer[8] = '\0';
                       strcat(LineBuffer, LineBuffer1);
                       DisplayMenu(LineBuffer);
                  } else {
                       CurrentMode[1] = SplitIX;
                       SetSplitA = true;  // initialize for the next time
                       CurrentModeIX = SplitModeIX;
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // finished set 'split', go back to Options Menu
                       OptionsMenu(0); 
                  } 
            break;
       } // end ok case
       case 8 : { // menu button pressed, go back to the Options Menu
            CurrentMenuIX = 1; CurrentOptionIX = 0; 
            OptionsMenu(0);               
            break;
        }
       case 0: {
            SetSplitA = true; // first call, start with setting mode for channel A
            SplitIX = 0;
            strcpy_P(LineBuffer, (char*)pgm_read_word(&(SplitStrings[0]))); // Necessary casts and dereferencing, just copy. 
            strcpy_P(LineBuffer1, (char*)pgm_read_word(&(ModeStrings[SplitIX]))); // Necessary casts and dereferencing, just copy. 
            LineBuffer[8] = '\0';
            strcat(LineBuffer, LineBuffer1);
            DisplayMenu(LineBuffer);
            break;
       }
       default : {
              CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
              OptionsMenu(0); 
              break;
       }
     break;   // end ButtonCode case      
  }
}

void MoreOptionsMenu(byte ButtonCode) {
   // 0: just update display
   // 1 : forward
   // 2 : backward
   // 4 : ok
   // 8 : menu

    switch (ButtonCode) {
      case 1: { // foreard
            MoreOptionsIX = CycleIX(MoreOptionsIX, NrMoreOptions, true);
            strcpy_P(LineBuffer, (char*)pgm_read_word(&(MoreOptionsStrings[MoreOptionsIX]))); // Necessary casts and dereferencing, just copy. 
            DisplayMenu(LineBuffer);  
            break;
        }
      case 2: { // backward
            MoreOptionsIX = CycleIX(MoreOptionsIX, NrMoreOptions, false);
            strcpy_P(LineBuffer, (char*)pgm_read_word(&(MoreOptionsStrings[MoreOptionsIX]))); // Necessary casts and dereferencing, just copy. 
            DisplayMenu(LineBuffer);
            break;
        }
      case 4: { // ok
             switch (MoreOptionsIX) {
                 case 0: { // Start Ramp Up
                       SetMoreOptions(0);
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
                       OptionsMenu(0); 
                       break;
                  }
                 case 1: { // Set Split Mode
                       SetMoreOptions(1);
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
                       OptionsMenu(0); 
                       break;
                  }             
                 case 2: { // Debug
                       ShowDebug(2);
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
                       OptionsMenu(0); 
                       break;
                  }
                 case 3: { // Set As Favorite
                       SetMoreOptions(3);
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
                       OptionsMenu(0); 
                       break;
                  }
                  case 4: { // Bluetooth configuration
                       BTInit(2);
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
                       OptionsMenu(0); 
                       break;
                  }
                  default : {  // should not happen
                       SetMoreOptions(9);
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
                       OptionsMenu(0); 
                       break;
                  }
                  break;
              }// end switch CurrentPwrLevel
              break;        
       } // end ok case
       case 8 : { // menu button pressed, go back to the Options Menu
            CurrentMenuIX = 1; CurrentOptionIX = 0; 
            OptionsMenu(0);               
            break;
        }
       case 0: {
              strcpy_P(LineBuffer, (char*)pgm_read_word(&(MoreOptionsStrings[MoreOptionsIX]))); // Necessary casts and dereferencing, just copy. 
              DisplayMenu(LineBuffer);
              break;
       }
       default : {
              CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
              OptionsMenu(0); 
              break;
       } 
     break;   // end ButtonCode case      
  }
}

void ModeMenu(byte  ButtonCode) {
   byte m;
  switch (ButtonCode) {
    case 1: { 
              CurrentModeIX = (byte(CurrentModeIX + 1) % NrModes); 
              CurrentMode[0] = CurrentModeIX; CurrentMode[1] = CurrentModeIX;
              break; 
            } // forward
    case 2: { 
              CurrentModeIX = (byte(CurrentModeIX - 1) % NrModes); 
              CurrentMode[0] = CurrentModeIX; CurrentMode[1] = CurrentModeIX;
              break;
            } // backward
    default: {break;}
    break;
  }
  m = min(max(0,CurrentModeIX),(NrModes - 1));  // make sure that Mode buffer not overflows
  PrepareMode(m);
  SetMode(m);
}

void DisplayMenu(char *buffer) {
     DisableButtons();
     lcd.clear(); lcd.print(buffer);
     // show second line (static)
     lcd.setCursor(0,1); lcd.write("Press     or OK "); // static text mask
     lcd.setCursor(6,1); lcd.write(byte(1)); lcd.setCursor(8,1); lcd.write(byte(2));
     EnableButtons();
}

void AdvancedMenu(byte ButtonCode) {
   // 0: just update display
   // 1 : forward
   // 2 : backward
   // 4 : ok
   // 8 : menu

    switch (ButtonCode) {
      case 1: { // foreard
            AdvancedIX = CycleIX(AdvancedIX, NrAdvanced, true);
            strcpy_P(LineBuffer, (char*)pgm_read_word(&(AdvancedStrings[AdvancedIX]))); // Necessary casts and dereferencing, just copy. 
            DisplayMenu(LineBuffer);
            break;
        }
      case 2: { // backward
            AdvancedIX = CycleIX(AdvancedIX, NrAdvanced, false);
            strcpy_P(LineBuffer, (char*)pgm_read_word(&(AdvancedStrings[AdvancedIX]))); // Necessary casts and dereferencing, just copy. 
            DisplayMenu(LineBuffer);
            break;
        }
      case 4: { // ok
             switch (AdvancedIX) {
                 case 0: { // Start Ramp Up
                       SetAdvanced(0);
                       break;
                  }
                 case 1: { // Set Split Mode
                       SetAdvanced(1);
                       break;
                  }             
                 case 2: { // Set As Favorite
                       SetAdvanced(2);
                       break;
                  }
                 case 3: { // Set Power Level
                       SetAdvanced(3);
                       break;
                  }
                 case 4: { // More Options
                       SetAdvanced(4);
                       break;
                  }
                 case 5: { // Save Settings
                       SetAdvanced(5);
                       break;
                  }             
                 case 6: { // Rest Settings
                       SetAdvanced(6);
                       break;
                  }
                 case 7: { // Adjust Advanced
                       SetAdvanced(7);
                       break;
                  }
                  default : {  // should not happen
                       SetAdvanced(9);
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
                       OptionsMenu(0); 
                       break;
                  }                                                    
                break; 
              }// switch (CurrentOptionIX)           
       } // end ok case
       case 8 : { // menu button pressed, go back to the Options Menu
            CurrentMenuIX = 1; CurrentOptionIX = 0; 
            OptionsMenu(0);               
            break;
        }
       case 0: {
              strcpy_P(LineBuffer, (char*)pgm_read_word(&(AdvancedStrings[AdvancedIX]))); // Necessary casts and dereferencing, just copy. 
              DisplayMenu(LineBuffer);
              break;
       }
       default : {
              CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
              OptionsMenu(0); 
              break;
       }
     break;   // end ButtonCode case      
  }
}

void PowerLevelMenu(byte ButtonCode) {
   // 0: just update display
   // 1 : forward
   // 2 : backward
   // 4 : ok
   // 8 : menu

    switch (ButtonCode) {
      case 1: { // foreward
            CurrentPwrLevelIX = CycleIX(CurrentPwrLevelIX, NrPwrLevels, true);
            strcpy_P(LineBuffer, (char*)pgm_read_word(&(PwrLevelStrings[CurrentPwrLevelIX]))); // Necessary casts and dereferencing, just copy. 
            DisplayMenu(LineBuffer);
            break;
        }
      case 2: { // backward
            CurrentPwrLevelIX = CycleIX(CurrentPwrLevelIX, NrPwrLevels, false);
            strcpy_P(LineBuffer, (char*)pgm_read_word(&(PwrLevelStrings[CurrentPwrLevelIX]))); // Necessary casts and dereferencing, just copy. 
            DisplayMenu(LineBuffer);
            break;
        }
      case 4: { // ok button pressed
             switch (CurrentPwrLevelIX) {
                 case 0: { // 
                       SetPowerLevel(0);
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
                       OptionsMenu(0); 
                       break;
                  }
                 case 1: { // 
                       SetPowerLevel(1);
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
                       OptionsMenu(0); 
                       break;
                  }             
                 case 2: { // 
                       SetPowerLevel(2);
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
                       OptionsMenu(0); 
                       break;
                  }
                  default : {
                       CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
                       OptionsMenu(0); 
                       break;
                  }
                  break;
              }// end switch CurrentPwrLevel
              break;        
       } // end ok case
       case 8 : { // menu button pressed, go back to the Options Menu
            CurrentMenuIX = 1;
            OptionsMenu(0);               
            break;
        }
       case 0: {
              strcpy_P(LineBuffer, (char*)pgm_read_word(&(PwrLevelStrings[CurrentPwrLevelIX]))); // Necessary casts and dereferencing, just copy. 
              DisplayMenu(LineBuffer);
              break;
       }
       default : {
              CurrentMenuIX = 1; CurrentOptionIX = 0;    // go back to Options Menu
              OptionsMenu(0); 
              break;
       }
     break;   // end ButtonCode case      
  }
}
// End Menu Interraction
// ---------------------------------------------------------------------


// Buttons Handling
// ---------------------------------------------------------------------

void buttonsCheck() {
  // pressed buttons are communicated to the menu functions via 4 bit
  // 1 : forward
  // 2 : backward
  // 4 : ok
  // 8 : menu
  if (right.check() == LOW) {
      (*MenuFunctions[CurrentMenuIX])(1); // call the mode menu 'forward'
  }
  if (left.check() == LOW) {
      (*MenuFunctions[CurrentMenuIX])(2); // call the mode menu 'backbard'
  }
  if (ok.check() == LOW) {
    (*MenuFunctions[CurrentMenuIX])(4); // call the mode menu 'ok'
  }
  if (menuB.check() == LOW) {
     CurrentMenuIX = 1; CurrentOptionIX = 0;    // start with Options Menu
     (*MenuFunctions[CurrentMenuIX])(0); // call the mode menu 'forward'
  }
}

// End Buttons Handling
// ---------------------------------------------------------------------

// Initialization
// ---------------------------------------------------------------------

void SelftestMenu() {
  // not yet implemented, just blink and show progress bar on LDC
  byte i;
  DisableButtons();
  for (i=0; i<16; i++) {
    lcd.setCursor(i,1); lcd.write(byte(0));
    delay(100);
    if ((odd(i))) {
        digitalWrite(PD5, LOW);
        digitalWrite(PD6, HIGH);
    } else {
        digitalWrite(PD5, HIGH);
        digitalWrite(PD6, LOW);
    }
  }
  delay(500);
 // TestDAC();
  lcd.setCursor(0,1); 
  lcd.print("SelfTest OK "); lcd.print(VERSION);
  EnableButtons();
  digitalWrite(PD5, LOW);
  digitalWrite(PD6, LOW);
  delay(500);
}

void BatteryMenu() {
  // 5% = 2.87V ; 10% = 2.9V; 21% = 2.92; 54% = 3.04V; 92% = 3.2V
  // ca. 0% = 2.85V = 2.85*1024/5  = 584
  //     100%% = 3.3V = 3.3*1024/5 = 676
   int b;
   
   b = analogRead(PA3) - 584;
   b = min(max(0,b), 100);
   DisableButtons();
   lcd.setCursor(0, 1);
   lcd.print("Battery: ");
   lcd.print((int)trunc(b));   // 10 bits scale approximately to 100%
   lcd.setCursor(11,1);
   lcd.print("%    ");
   EnableButtons();
   delay(1000);
}

void SwitchFET(boolean channel, boolean phase) {
  // Channel A = TRUE, B = false
  // phase positive = TRUE, negative = FALSE
  if (channel)  // channel A
        { if (phase) { digitalWrite(PB2, HIGH); digitalWrite(PB3, LOW);} 
                else { digitalWrite(PB2, LOW); digitalWrite(PB3, HIGH);}
       } else { // channel B
          if (phase) { digitalWrite(PB0, HIGH); digitalWrite(PB1, LOW);} 
                else { digitalWrite(PB0, LOW); digitalWrite(PB1, HIGH);}
               }
}

void SwitchFEToff() {
  digitalWrite(PB0, HIGH); digitalWrite(PB1, HIGH); digitalWrite(PB2, HIGH); digitalWrite(PB3, HIGH);
}

void TestDAC () {
byte qA, qB;

  qA = 0; qB = 0;
  for (uint32_t i=0; i< (256*100 - 1); i++) {
    if ((i % 256) == 0) {
       qA = byte((qA + 1) % 4);   qB = byte((qB + 1) % 4);
    }
    dac.loUp(LowPwrLevel + burstFunction02((byte)(i%256), qA),
                           burstFunction21((byte)(i%256), qB, 
                              CalcEnveloppeSpeed(analogRead(PA1)),
                              255));
    
 //   delayMicroseconds(10); burstFunction21(byte ix, byte quadrant, byte AMax, byte tMax) 
  }
  SwitchFEToff(); 
  delay(1000);
}

uint16_t CalcEnveloppeSpeed(uint16_t ma) {
  // MAInputLevel is 0..370, scale it to 6..70
///  return uint16_t (6 + ((uint16_t (64*ma))/370));
    return uint16_t (6 + ((uint16_t (ma))/6));

}

byte GetMATime(uint8_t min, uint8_t max) {
   // returns a value between 'min' and 'max' depending on the percentage measured on the MultiAdjust 'ma'
   // typically used for setting a time in milli seconds between 6..255
   // PA1  is 0..340, scale it to min..max
   return (min + (MAInput*(max - min))/340);

}

void setup() {

  pinMode(PA0,INPUT);  // measure output current
  pinMode(PA1,INPUT);  // analog output of channel B
  pinMode(PA2,INPUT);   // use this as analog input for potentiometer
  pinMode(PA3,INPUT);   // use this as analog input for potentiometer
  pinMode(PA4,INPUT);  // Potentiometer Chanel A
  pinMode(PA5,INPUT);  // Potentiometer Chanel B
  pinMode(PA6,INPUT);   // use this as analog input for potentiometer
  pinMode(PA7,INPUT);   // use this as analog input for potentiometer
  
  pinMode(PB0,OUTPUT); // Output A FET +switch on/off
  pinMode(PB1,OUTPUT); // Output A FET -switch on/off
  pinMode(PB2,OUTPUT); // Output B FET +switch on/off
  pinMode(PB3,OUTPUT); // Output B FET -switch on/off
  pinMode(PB4,OUTPUT); // Output A FET +switch on/off
  pinMode(PB5,OUTPUT); // Output A FET -switch on/off
  pinMode(PB6,OUTPUT); // Output B FET +switch on/off
  pinMode(PB7,OUTPUT); // Output B FET -switch on/

  SwitchFEToff(); // turn off all FETs for initialization

  pinMode(PC0,OUTPUT);
  pinMode(PC1,OUTPUT);
  pinMode(PC2,OUTPUT);
  pinMode(PC3,OUTPUT);
  pinMode(PC4,OUTPUT);
  pinMode(PC5,OUTPUT);
  pinMode(PC6,OUTPUT);
  pinMode(PC7,OUTPUT);

 // dac.loUp(0);  // set both DAC LDT1661 outputs to 0 (GND)
  
  pinMode(PD0,INPUT);
  pinMode(PD1,OUTPUT);
  pinMode(PD2,INPUT); // multi level adjust potio, middle pin
  pinMode(PD3,INPUT);// multi level adjust potio, middle pin
  pinMode(PD4,OUTPUT);
  pinMode(PD5,OUTPUT);  // LED B
  pinMode(PD6,OUTPUT);  // LED A
  pinMode(PD7,OUTPUT);

  // Menu initializations
  // ---------------------
  CurrentModeIX     = 0;  // start with mode 'wave' after reboot
  CurrentMenuIX     = 0;  // first normal operation (no menu) after restart is options menu
  CurrentOptionIX   = 0;
  AdvancedIX        = 0;
  SplitIX           = 0;
  CurrentPwrLevelIX = 0;
  MoreOptionsIX     = 0;
  SetSplitA         = true;  // start with setting channel A in mode 'split'

  
  DisableButtons();
  lcd.begin(16, 2);
  // create a new character
  lcd.createChar(0, heart);
  // create a new character 
  lcd.createChar(1, arrowLeft);
    // create a new character
  lcd.createChar(2, arrowRight);
  EnableButtons();

//  Serial.begin(38400);
//  Serial.flush();
//  Serial.println("Initialization done");
//  Serial.flush();

  StartMenu();

  // initialize the D/A converter
  // -----------------------------

  ChannelAModulationFactor = 0;
  ChannelBModulationFactor = 0;
  ChannelAModulationBase   = NormalModulationFactor;
  ChannelBModulationBase   = NormalModulationFactor;
  ChannelAPwrBase =          NormalPwrLevel; 
  ChannelBPwrBase =          NormalPwrLevel;

  dac.loUp(1023);  // set both DAC LDT1661 outputs to max voltage, meaning minimum output power
  dac.loUp(1023); //set both outputs of the DAC LTC1661 to max voltage, meaning minimum output power
  digitalWrite(PB0,LOW); digitalWrite(PB1, LOW);
  digitalWrite(PB2,LOW); digitalWrite(PB3, LOW);

  pd5Output = HIGH;
  pd6Output = HIGH;
  SetupTimer1();
  SetupTimer2();
  
  // eStim Signal generation initializations
  // ---------------------------------------
  currentRampUpTime = defaultRampUpTime;
  RampUpCounter = 0;
  RampUpOn = true;
  RampUpTimer.setTimeOutTime(currentRampUpTime);
  RampUpTimer.reset();
  
  RandomModeTimer.setTimeOutTime(1000);  // initialize the timer for the random modes
  RandomModeTimer.reset();
  
  Enveloppe[0].setTimeOutTime(6);  // must not be smaller than 6ms
  Enveloppe[1].setTimeOutTime(6);  // must not be smaller than 6ms
  Enveloppe[0].reset();
  Enveloppe[1].reset();

  BurstCounter[0]  = 0;
  BurstQuadrant[0] = 0;
 
  WaveCounter[0]  = 0;
  WaveQuadrant[0] = 0;

  CurrentMode[0] = CurrentModeIX;
  CurrentMode[1] = CurrentModeIX;

}

void StartPuls(uint8_t channel, byte PulsMode, byte PulsDuration) {
  GatePulsCount[channel] = PulsMode;
  switch (channel) {
    case 0: { // channel A, use timer1
              OCR1A  = PulsDuration;
              TCNT1  = 0;  //initialize counter value to 0
              TIMSK |= (1 << OCIE1A); // start Timer2 interrupt, e.g. start outputting one burst. ISR stops the interrupt
              break;
            }
    case 1: { // channel B, use timer2
              OCR2   = PulsDuration;
              TCNT2  = 0;//initialize counter value to 0
              TIMSK |= (1 << OCIE2); // start Timer2 interrupt, e.g. start outputting one burst. ISR stops the interrupt
              break;
            }
    break;
  }
}

// End Initialization
// ---------------------------------------------------------------------

void loop() 
{  
  buttonsCheck(); 
  if ((CurrentMenuIX == 0)) { RunningLine1();}
  if (RampUpOn) { // ramp up phase during mode changes
    if (RampUpTimer.hasTimedOut()) {
        RampUpTimer.setTimeOutTime(currentRampUpTime);
        DisableButtons();
        lcd.setCursor(0,1); lcd.write("Ramp:           "); // clear line
        lcd.setCursor(9,1); lcd.write(byte(1)); lcd.write(byte(2)); lcd.write("=Mode");
        lcd.setCursor(6,1);
        if (RampUpCounter < 10) { lcd.print(" ");}
 //       lcd.print(RampUpCounter);
        lcd.print(ChannelAModulationFactor);

        EnableButtons();
        RampUpCounter++;
        ChannelAModulationFactor = max(0,ChannelAModulationFactor - RampUpPowerIncA); // make sure the mod factor stays >0
        ChannelBModulationFactor = max(0,ChannelBModulationFactor - RampUpPowerIncB);

        if (RampUpCounter > 99) {
               RampUpOn = false; 
               RampUpCounter = 0;
               SetMode(CurrentModeIX);
               ChannelAModulationFactor = 0;
               ChannelBModulationFactor = 0;}
        RampUpTimer.reset();
    }
  }
  switch (CurrentModeIX) {
    case 0: { // do nothing, this is the regular case, e.g. NO random mode
      break;
    }
    case 10: {  // random mode 1
            if (RandomModeTimer.hasTimedOut()) { 
                RandomModeTimer.setTimeOutTime(random(5000,8000));  // change randomly everey 10..40 seconds
                CurrentMode[0] = random(0,5);  // change randomly to mode 0..5
                CurrentMode[1] = CurrentMode[0];  // set both channels to the same, random selected mode
                SetMode(CurrentModeIX);  // update LCD
                RandomModeTimer.reset();
               }
      break;
    }
    case 11: {  // random mode 2
      break;
    }
    break;
  }

  // loop over both channels and set the the modes
  for (uint8_t c=0; c < 2; c++) {  // loop over both channels, e.g. do it for A and B
    switch (CurrentMode[c]) {  // following cases code each individual mode, 1..18 cases
    case 0: { // wave
          if (Enveloppe[c].hasTimedOut()) { 
              Enveloppe[c].setTimeOutTime(burstFunction21(WaveCounter[c], WaveQuadrant[c],
                                        CalcEnveloppeSpeed(MAInput),
                                        196));
             StartPuls(c, 2, burstFunction01(BurstCounter[c], BurstQuadrant[c])); // 255=255us,100=100us,50=50us, 10=20us, 1=20us < 256, 20us minimum
             BurstCounter[c] = (BurstCounter[c] + 1) % 256; // 
             if (BurstCounter[c] == 0) { BurstQuadrant[c] = (BurstQuadrant[c] + 1) % 4;}
             WaveCounter[c] = (WaveCounter[c] + 1) % 196; // 
             if (WaveCounter[c] == 0) { WaveQuadrant[c] = (WaveQuadrant[c] + 1) % 4;}
             Enveloppe[c].reset();
           }
           break;
          } // end code for mode wave
    case 1: { // Stroke
              if (Enveloppe[c].hasTimedOut()) { 
                Enveloppe[c].setTimeOutTime(MinPulsDistance);   // MAInutLevel, 0..100, scale it to 100ms .. 40000ms
                if (BurstQuadrant[c] == 0) { 
                    StartPuls(c,3,StrokePulsTime);// generate a positive puls
                   } else {
                    StartPuls(c,1,StrokePulsTime); // generate a negative puls
                   } 
                BurstCounter[c] = (BurstCounter[c] + 1) % (1 + MAInputLevel); // 
                if (BurstCounter[c] == 0) { BurstQuadrant[c] = (BurstQuadrant[c] + 1) % 2;}
                Enveloppe[c].reset();
              }
              break;
          }   // end code for mode 'Stroke'
    case 2: { //Climb, about 300 bytes code generated for this case
              if (Enveloppe[c].hasTimedOut()) { 
                  Enveloppe[c].setTimeOutTime(burstFunction32(WaveCounter[c], WaveQuadrant[c],
                                              CalcEnveloppeSpeed(analogRead(PA1)),
                                              196));
                  StartPuls(c, 2, ClimbPulsTime);    // fix 130uSec puls in this mode
                  WaveCounter[c] = (WaveCounter[c] + 1) % 196; // 
                  if (WaveCounter[c] == 0) { WaveQuadrant[c] = (WaveQuadrant[c] + 1) % 4;}
                  Enveloppe[c].reset();
              }
              break;
          }   // end code for mode 'Climb'
    case 3: { // Combo
              if (Enveloppe[c].hasTimedOut()) { 
                  Enveloppe[c].setTimeOutTime(GetMATime(5,30)); // set a time between 5ms and 30ms, depending on the MA position
                                                        
                  if (odd(BurstQuadrant[c])) {
                       StartPuls(c,2,burstFunction01(BurstCounter[c], BurstQuadrant[c]));// generate a symmetric positive/negative puls
                   } else {
                       StartPuls(c,0,0); // generate no puls for the same amount of time
                   } 
                   BurstCounter[c] = (BurstCounter[c] + 1) % GetMATime(5,100); // 
                   if (BurstCounter[c] == 0) { BurstQuadrant[c] = (BurstQuadrant[c] + 1) % 4;}
                   Enveloppe[c].reset();
              }
              break;
          } // end code for mode 'Combo'
    case 4: { // Intense
              if (Enveloppe[c].hasTimedOut()) { 
                  Enveloppe[c].setTimeOutTime(GetMATime(5,70)); // set a time between 5ms and 30ms, depending on the MA position
                  StartPuls(c, 2, IntensePulsTime);    // fix 130uSec puls in this mode
                  Enveloppe[c].reset();
              }
              break;
          } // end code for mode 'Intense'
    case 5: { // Rhythm
              if (Enveloppe[c].hasTimedOut()) { 
                  Enveloppe[c].setTimeOutTime(GetMATime(5,10)); // set a time between 5ms and 30ms, depending on the MA position
                  if (odd(BurstQuadrant[c])) {
                       if ((odd(WaveQuadrant[c]))) { 
                          StartPuls(c,2,RythmPulsTime1);
                       } else {
                          StartPuls(c,2,RythmPulsTime2);
                       }
                  } else {
                          StartPuls(c,0,0); // generate no puls for the same amount of time
                  }
                  BurstCounter[c] = (BurstCounter[c] + 1) % GetMATime(2,10); // 
                  if (BurstCounter[c] == 0) { BurstQuadrant[c] = (BurstQuadrant[c] + 1) % 4;}
                  WaveCounter[c] = (WaveCounter[c] + 1) % ((byte)(1000/GetMATime(5,10))); // 
                  if (WaveCounter[c] == 0) { WaveQuadrant[c] = (WaveQuadrant[c] + 1) % 2;}
                  
                  Enveloppe[c].reset();
              }
              break;
          } // end code for mode 'Rhythm'
    case 6: { // Audio1
              if (Enveloppe[c].hasTimedOut()) { 
                  Enveloppe[c].setTimeOutTime(7); // set a time between 5ms and 30ms, depending on the MA position
                  StartPuls(c, 2, AudioPulsTime);    // fix 130uSec puls in this mode
                  Enveloppe[c].reset();
              }
           break;
          }   // end code for mode 'Audio1'
    case 7: { //Audio2
              if (Enveloppe[c].hasTimedOut()) { 
                  Enveloppe[c].setTimeOutTime(7); // set a time between 5ms and 30ms, depending on the MA position
                  StartPuls(c, 2, AudioPulsTime);    // fix 130uSec puls in this mode
                  Enveloppe[c].reset();
              } 
           break;
          }   // end code for mode 'Audio2'
    case 8: { // Audio3
           break;
          } // end code for mode 'Audio3'
    case 9: { // Split, nothing needs to be done here. In fact, should never be reached anyway
           break;
          } // end code for mode 'Split'
    case 10: { // Random1, do nothing, handled outside this 'switch' statement
           break;
          } // end code for mode 'Random1'
    case 11: { // Random2
           break;
          }   // end code for mode 'Random2'
    case 12: { //Toggle
             if (Enveloppe[c].hasTimedOut()) { 
                  Enveloppe[c].setTimeOutTime(7); // set a time between 5ms and 30ms, depending on the MA position
                   if (odd(BurstQuadrant[c])) {
                       StartPuls(c,2,TogglePulsTime);// generate a symmetric positive/negative puls
                   } else {
                       StartPuls(c,0,0); // generate no puls for the same amount of time
                   } 
                   BurstCounter[c] = (BurstCounter[c] + 1) % GetMATime(5,100); // 
                   if (BurstCounter[c] == 0) { BurstQuadrant[c] = (BurstQuadrant[c] + 1) % 2;}
                   Enveloppe[c].reset();
              }
           break;
          }   // end code for mode 'Toggle'
    case 13: { // Orgasm
               if (Enveloppe[c].hasTimedOut()) { 
                  Enveloppe[c].setTimeOutTime(GetMATime(7,70)); // set a time between 5ms and 30ms, depending on the MA position
                  StartPuls(c, 2, burstFunction01(BurstCounter[c], BurstQuadrant[c])); // 255=255us,100=100us,50=50us, 10=20us, 1=20us < 256, 20us minimum
                  BurstCounter[c] = (BurstCounter[c] + 1) % 256; // 
                  if (BurstCounter[c] == 0) { BurstQuadrant[c] = (BurstQuadrant[c] + 1) % 4;}
                  WaveCounter[c] = (WaveCounter[c] + 1) % 196; // 
                  Enveloppe[c].reset();
              }
              break;
          } // end code for mode 'Orgasm'
    case 14: { // Torment
           break;
          } // end code for mode 'Torment'
    case 15: { //Phase1
           break;
          }   // end code for mode 'Phase1'
    case 16: { // Phase2
           break;
          } // end code for mode 'Phase2'
    case 17: { // Phase3
           break;
          } // end code for mode 'Phase3'                
    default: {break;}
    break;
  } // end switch case over all modes
 }  // end for loop over both channels, A,B
}
