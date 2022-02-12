/* PROJECT: SMART MiRROR: INFINITY MIRROR-CLOCK-SCANNER
By: Rakshith Badarinath

Description: In this project, we create an infinity mirror clock with LIDAR scanner system. It will have 3 modes of operations: Time mode(default), Randomization mode(auto trigerred based on color interval value)
and LIDAR scan mode(activated by external interrupt).

*/

//Declare libraries
#include <Adafruit_DotStar.h>  //This if for the Minute LED strip - 4 pins: +5,GND,DI,CI
#include <Adafruit_NeoPixel.h> //This is for the Second and Hour Hand - 3 pins: +5,GND,DI
#include <SPI.h> // Serial communication library required for RTC module
#include <Wire.h> //I2C for LIDAR
#include "RTClib.h"


//define global pins
#define DATAPIN 8 //Min Hand
#define CLOCKPIN 9 //Min Hand
#define HOURPIN 10 //Hour Hand
#define SECPIN 13 //Second Hand

//Define # of LEDs in each strip
#define MINPIXELS 60 //Minutes
#define HOURPIXELS 12 //Hours
#define SECPIXELS 31 //Seconds 
#define MAXPIXELS 60

#define LASER 11 //Line laser controlled using digital pip 11 

//Define and set LED libraries
Adafruit_NeoPixel hourstrip = Adafruit_NeoPixel(HOURPIXELS, HOURPIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel secstrip = Adafruit_NeoPixel(SECPIXELS, SECPIN, NEO_GRB + NEO_KHZ800);

Adafruit_DotStar minstrip = Adafruit_DotStar(MINPIXELS, DATAPIN, CLOCKPIN);

// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
RTC_Millis rtc;

// Define time variables
unsigned long seconds = 0;
unsigned long minutes = 0;
unsigned long hours = 0;

unsigned long startTime = 0;
unsigned long currentTime = 0;
unsigned long colourInterval = 150000;

// Define Color Variables for each time elements
uint32_t SecondColor = 0;
uint32_t MinuteColor = 0;
uint32_t HourColor = 0;
uint32_t secNoColor = 0;
uint32_t minNoColor = 0;
uint32_t hourNoColor = 0;
uint32_t BGColor = 0;
uint32_t ClearColor = 0;
uint32_t scanColor = 0;
uint32_t distSwipeColor = 0;
uint32_t distColor = 0;

//randomisation control variables
unsigned int x = 1;

////////////////////////////////////////////////////////////////////
//Stepper motor and LIDAR control variables
// define pin names
// wire colors for Oriental Vexta
#define coilA 4  // yellow
#define coilB 5  // brown
#define coilC 6  // red
#define coilD 7  // orange

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

// declare global variables
int row;    // 16b signed
byte abcd;  // 8b unsigned
byte table[8] = { 8, 4, 2, 1, 8, 4, 2, 1 };  // wave drive
//byte table[8] = { 9, 12, 6, 3, 9, 12, 6, 3 };  // two phase
//byte table[8] = { 8, 12, 4, 6, 2,  3, 1, 9 };  // half step

int nStep, delayStep;  // 16b signed

// length of delay [sec]
// Vexta, ULN2803A, 5V breadboard supply, no load
//  35 msec single step, 25 msec two phase, 35 msec half step

int pos = 0;         // Position of the stepper (degress, [0, 360])
int distance = 0;    // Distance measured
int ledPos = 0;
int minDist  = 0;

//LCD Functions
#include <SoftwareSerial.h>
// Create a software serial port!
SoftwareSerial lcd = SoftwareSerial(0,2); 

///////////////////////////////////////////////////////////////////////

void setup () {

  Serial.begin(115200); //higher baud rate required
  //rtc.begin(); //begin RTC module

  pinMode(3, INPUT_PULLUP); //For interrupt 
  pinMode(2, OUTPUT);  //For LCD

  // Stepper control
  // configure digital outputs
  pinMode( coilA, OUTPUT );
  pinMode( coilB, OUTPUT );
  pinMode( coilC, OUTPUT );
  pinMode( coilD, OUTPUT );

  // send intial position
  row = 0;
  abcd = table[ row ];
  digitalWrite( coilA, bitRead( abcd, 3 ) );
  digitalWrite( coilB, bitRead( abcd, 2 ) );
  digitalWrite( coilC, bitRead( abcd, 1 ) );
  digitalWrite( coilD, bitRead( abcd, 0 ) );

  Wire.begin(); //Begin I2C serial communication

  hourstrip.begin();
  minstrip.begin();
  secstrip.begin();

  hourstrip.show();
  minstrip.show();
  secstrip.show();

  // following line sets the RTC to the date & time this sketch was compiled
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  //

  setClearColor(0, 0, 0);
  setBGColor(0, 255, 0);
  Background();

  scanColor = minstrip.Color(255, 0, 0);
  distColor = hourstrip.Color(0, 174, 220);

  attachInterrupt(1,  scanLIDAR , LOW ); //activate interrupt when pi 3 goes low

  digitalWrite(LASER, LOW);
  
  Serial.print("HH:");
  Serial.print("\t\t");
  Serial.print("MM:");
  Serial.print("\t\t");
  Serial.println("SS:");
  
  /////////////////////////////////////////////////
  //LCD SETUP
  lcd.begin(9600);  
  
  // set the size of the display if it isn't 16x2 (you only have to do this once)
  lcd.write(0xFE);
  lcd.write(0xD1);
  lcd.write(16);  // 16 columns
  lcd.write(2);   // 2 rows
  delay(10); // we suggest putting delays after each command to make sure the data 
            // is sent and the LCD is updated.

  // set the contrast, 200 is a good place to start, adjust as desired
  lcd.write(0xFE);
  lcd.write(0x50);
  lcd.write(200);
  delay(10);       
  
  // set the brightness - we'll max it (255 is max brightness)
  lcd.write(0xFE);
  lcd.write(0x99);
  lcd.write(255);
  delay(10);       
  
  // turn off cursors
  lcd.write(0xFE);
  lcd.write(0x4B);
  lcd.write(0xFE);
  lcd.write(0x54);

  // clear screen
  lcd.write(0xFE);
  lcd.write(0x58);
  delay(10);   // we suggest putting delays after each command 
  
  // go 'home'
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(10);   // we suggest putting delays after each command 
  
  //Print Greeting message
  lcd.println("Welcome To ");       
  lcd.print("INFINITY MIRROR");
    
  delay(1000); 
  
} //End of void setup

///////////////////////////////////////////////////
// lidarGetDistance() Function
// Get a measurement from the LIDAR Lite
int lidarGetRange(void)
{
  int val = -1;

  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit

  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit

  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if (2 <= Wire.available()) // if two bytes were received
  {
    val = Wire.read(); // receive high byte (overwrites previous reading)
    val = val << 8; // shift high byte to be high 8 bits
    val |= Wire.read(); // receive low byte as lower 8 bits
  }

  return val;
}

////////////////////////////////////////////////////////////
// Loop

void loop ()
{
  
  currentTime = millis();

  DateTime now = rtc.now(); //get time from RTC
  
  Serial.print(hours);
  Serial.print("\t\t");
  Serial.print(minutes);
  Serial.print("\t\t");
  Serial.println(seconds);  
  
  printMode(1);  //Print mode on LCD
 
  // Set Time
  if (now.hour() > 12)
    hours = now.hour() - 12;
  else
    hours = now.hour();
  minutes = now.minute();
  seconds = now.second()/2; //scale seconds to be represented on 30 LED strips

  setSecColor(0, 0, 255);
  setMinColor(0, 255, 0);
  setHourColor(0, 0, 255);
  setsecNoColor(0, 100, 0);
  setminNoColor(0, 100, 0);
  sethourNoColor(0, 100, 0);

  showTime(); //Call time function --> display time

  //constraints for randomisation: Happen every colorInterval
  if (currentTime > x * colourInterval)
  {
    x++;

    //reached required interval, start randomisation
    Random();
    delay(1000);
    clearLED();
    delay(50);
    Background();
  }

  else
  {
    setSecColor(0, 0, 255);
    setMinColor(0, 255, 0);
    setHourColor(0, 0, 255);
    setsecNoColor(0, 100, 0);
    setminNoColor(0, 100, 0);
    sethourNoColor(0, 100, 0);
  }
} //end of void loop

//////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS:
// Time Mode Functions:
void ShowSeconds(int s, int c, int nc)
{
  secstrip.setPixelColor(s - 1, nc);
  secstrip.setPixelColor(s, c);
  secstrip.show();

}

void ShowMinutes(int s, int c, int nc)
{
  minstrip.setPixelColor(s - 1, nc);
  minstrip.setPixelColor(s, c);
  minstrip.show();
}

void ShowHour(int s, int c, int nc)
{
  hourstrip.setPixelColor(s - 1, nc);
  hourstrip.setPixelColor(s, c);
  hourstrip.show();
}
/////////////////////////////////////////////////////////////////////////////////////////
//Randomization Color Functions:

void ColorWipe(uint32_t c, uint8_t wait)
{
  for (uint32_t i = 0; i < MINPIXELS; i++)
  {
    minstrip.setPixelColor(i, c);
    minstrip.show();
    delay(wait);
    hourstrip.setPixelColor(i, c);
    hourstrip.show();
    delay(wait);
    secstrip.setPixelColor(i, c);
    secstrip.show();
    delay(wait);
  }

}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait)
{
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++)
  { // 5 cycles of all colors on wheel
    for (i = 0; i < MAXPIXELS; i++)
    {
      secstrip.setPixelColor(i, secWheel(((i * 256 / secstrip.numPixels()) + j) & 255));
    }
    secstrip.show();
    delay(wait);

    for (i = 0; i < minstrip.numPixels(); i++)
    {
      minstrip.setPixelColor(i, minWheel(((i * 256 / minstrip.numPixels()) + j) & 255));
    }
    minstrip.show();
    delay(wait);

    for (i = 0; i < hourstrip.numPixels(); i++)
    {
      hourstrip.setPixelColor(i, hourWheel(((i * 256 / hourstrip.numPixels()) + j) & 255));
    }
    hourstrip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t secWheel(byte WheelPos) {
  if (WheelPos < 85) {
    return secstrip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return secstrip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return secstrip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

uint32_t minWheel(byte WheelPos) {
  if (WheelPos < 85) {
    return minstrip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return minstrip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return minstrip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

uint32_t hourWheel(byte WheelPos) {
  if (WheelPos < 85) {
    return hourstrip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return hourstrip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return hourstrip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void colorWheel()
{      
    uint32_t c1,c2,c3,c4 = 0;
    int w,y = 0;
    int count = 1;
        
    c1 = minstrip.Color(255, 0, 0);
    c2 = minstrip.Color(10, 75, 127);
    c3 = minstrip.Color(1, 255,125 );
    c4 = minstrip.Color(125, 2, 125);
    
    for(int m=0;m<10;m++)
    {
                  
      for(y = 0; y < (MINPIXELS/4); y++)
      {                  
         minstrip.setPixelColor(y, c1);
         minstrip.show();
         secstrip.setPixelColor(y/2, c1);
         secstrip.show();
         hourstrip.setPixelColor(y/5, c1);
         hourstrip.show();
         delay(2);
      }
     delay(50);
     
     for(y = 0; y < (MINPIXELS/4); y++)
      {          
         minstrip.setPixelColor(y, c4);
         minstrip.show();
         secstrip.setPixelColor(y/2, c4);
         secstrip.show();
         hourstrip.setPixelColor(y/5, c4);
         hourstrip.show();
         delay(2);
      }
     
     delay(50);
     
      
      for(y = (MINPIXELS/4); y < (MINPIXELS/2); y++)
      {
         minstrip.setPixelColor(y, c2);
         minstrip.show();
         secstrip.setPixelColor(y/2, c2);
         secstrip.show();
         hourstrip.setPixelColor(y/5, c2);
         hourstrip.show();
         delay(2);
      }
      
      delay(50);
      
      for(y = (MINPIXELS/4); y < (MINPIXELS/2); y++)
      {
         minstrip.setPixelColor(y, c1);
         minstrip.show();
         secstrip.setPixelColor(y/2, c1);
         secstrip.show();
         hourstrip.setPixelColor(y/5, c1);
         hourstrip.show();
         delay(2);
      }
     
     delay(5);
     
            for(y = (MINPIXELS/2); y < (MINPIXELS*0.75); y++)
      {
         minstrip.setPixelColor(y, c3);
         minstrip.show();
         secstrip.setPixelColor(y/2, c3);
         secstrip.show();
         hourstrip.setPixelColor(y/5, c3);
         hourstrip.show();
         delay(2);
      }
      
      delay(50);
      
      for(y = (MINPIXELS/2); y < (MINPIXELS*0.75); y++)
      {
         minstrip.setPixelColor(y, c2);
         minstrip.show();
         secstrip.setPixelColor(y/2, c2);
         secstrip.show();
         hourstrip.setPixelColor(y/5, c2);
         hourstrip.show();
         delay(2);
      }
     
     delay(50);
      
      
      for(y=(MINPIXELS*0.75); y < MINPIXELS; y++)
      {         
         minstrip.setPixelColor(y, c4);
         minstrip.show();
         secstrip.setPixelColor(y/2, c4);
         secstrip.show();
         hourstrip.setPixelColor(y/5, c4);
         hourstrip.show();
         delay(2);
      }
      delay(50);
      
      for(y=(MINPIXELS*0.75); y < MINPIXELS; y++)
      {
         minstrip.setPixelColor(y, c3);
         minstrip.show();
         secstrip.setPixelColor(y/2, c3);
         secstrip.show();
         hourstrip.setPixelColor(y/5, c3);
         hourstrip.show();
         delay(2);
      }    
    }
}

///////////////////////////////////////////////////////////////////////////////////
// Randomization Main function defnition

void Random()
{
  //COLORWIPE RANDOMISATION
  
  printMode(2);
  ColorWipe(hourstrip.Color(0, 0, 255), 40);
  delay(1000);
  //clearLED();

  ColorWipe(hourstrip.Color(0, 255, 0), 40);
  delay(1000);
  //clearLED();
  
  printMode(3);
  //RAINBOW RANDOMISATION
  rainbowCycle(2);
  
  printMode(4);
  //colorWheel - my own function
  colorWheel();
  
}

void clearLED()
{
  {
    for (uint16_t i = 0; i < SECPIXELS; i++)
    {
      secstrip.setPixelColor(i, ClearColor);
      secstrip.show();
    }
    for (uint16_t i = 0; i < MINPIXELS; i++)
    {
      minstrip.setPixelColor(i, ClearColor);
      minstrip.show();
    }
    for (uint16_t i = 0; i < HOURPIXELS; i++)
    {
      hourstrip.setPixelColor(i, ClearColor);
      hourstrip.show();
    }
  }
}
/////////////////////////////////////////////////////////////////
// Show time function defnition
// Maps current time into corrsponding LED positions

void showTime()
{
  ShowSeconds(seconds + 1, SecondColor, secNoColor);
  ShowMinutes(minutes - 1, MinuteColor, minNoColor);
  ShowHour(hours - 1, HourColor, hourNoColor);
}

///////////////////////////////////////////////////////////////
//Set color functions for hour, minute, second hands and background

void setMinColor(uint32_t r, uint32_t g, uint32_t b)
{
  MinuteColor = minstrip.Color(r, g, b);
}

void setSecColor(uint32_t r, uint32_t g, uint32_t b)
{
  SecondColor = secstrip.Color(r, g, b);
}

void setHourColor(uint32_t r, uint32_t g, uint32_t b)
{
  HourColor = hourstrip.Color(r, g, b);
}

void setsecNoColor(uint32_t r, uint32_t g, uint32_t b)
{
  secNoColor = secstrip.Color(r, g, b);
}

void setminNoColor(uint32_t r, uint32_t g, uint32_t b)
{
  minNoColor = secstrip.Color(r, g, b);
}

void sethourNoColor(uint32_t r, uint32_t g, uint32_t b)
{
  hourNoColor = secstrip.Color(r, g, b);
}

void setBGColor(uint32_t r, uint32_t g, uint32_t b)
{
  BGColor = secstrip.Color(r, g, b);
}
void setClearColor(uint32_t r, uint32_t g, uint32_t b)
{
  ClearColor = secstrip.Color(r, g, b);
}

void Background()
{
  for (uint16_t i = 0; i < SECPIXELS; i++)
  {
    secstrip.setPixelColor(i, BGColor);
    secstrip.show();
  }
  for (uint16_t i = 0; i < MINPIXELS; i++)
  {
    minstrip.setPixelColor(i, BGColor);
    minstrip.show();
  }
  for (uint16_t i = 0; i < HOURPIXELS; i++)
  {
    hourstrip.setPixelColor(i, BGColor);
    hourstrip.show();
  }
}

///////////////////////////////////////////
// LIDAR CONTROL FUNCTIONS
// Serail Prints for each distance measured, the corresponding LED mapping and brightness

void serialPrintRange(int distance, int pos, int ledcon)
{
  Serial.print("distance (cm): ");
  Serial.print(distance);
  Serial.print("\t\tledPos (#): ");
  Serial.print(pos);
  Serial.print("\t\tledBrightness: ");
  Serial.println(ledcon);
}


//LIDAR Scan Function
// Activated upon interrupt at pin 3 (int 1)

void scanLIDAR()
{
  clearLED();
  
  printMode(5);  //Print Mode name over LCD
  
  // declare local variables
  int i, k, n, dir;  // 16b signed
  int ledBrightness = 0;
  int ledcon;
  int closeDist = 100;
  int nearest = 0;
  int stepCount = 0;
  
  int nStep = 120; //120 steps for easier scaling

  // number of steps and direction
  n = abs( nStep );
  dir = constrain( nStep, -1, 1 );
  int dist = 0;

  //Switch On laser
  digitalWrite(LASER, HIGH);

  // count steps
  if ( n > 0 )
  {
    for ( k = 0; k < n; k++ )
    {
      // next row in table
      row = row + dir;
      if ( row < 0 ) {
        row = 7;
      }
      if ( row > 7 ) {
        row = 0;
      }

      // send new position
      abcd = table[ row ];
      digitalWrite( coilA, bitRead( abcd, 3 ) );
      digitalWrite( coilB, bitRead( abcd, 2 ) );
      digitalWrite( coilC, bitRead( abcd, 1 ) );
      digitalWrite( coilD, bitRead( abcd, 0 ) );

      distance = lidarGetRange();
      
      
      //shortest distance / closest object detection algorithm 
      if(distance < closeDist)
      {
        nearest = distance;
        closeDist = nearest;
        stepCount  = k;
      }
      
      //LED position mapping algorithm
      if (ledPos >= 1 && ledPos <= 6)
        distSwipeColor = secstrip.Color(227, 23, 23); //color
      else if (ledPos >= 7 && ledPos <= 13)
        distSwipeColor = secstrip.Color(227, 23, 91); //color

      else if (ledPos >= 14 && ledPos <= 20)
        distSwipeColor = secstrip.Color(227, 23, 128); //color
      else if (ledPos >= 21 && ledPos <= 27)
        distSwipeColor = secstrip.Color(227, 23, 172); //color

      else if (ledPos >= 28 && ledPos <= 34)
        distSwipeColor = secstrip.Color(217, 23, 227); //color

      else if (ledPos >= 35 && ledPos <= 41)
        distSwipeColor = secstrip.Color(176, 23, 227); //color

      else if (ledPos >= 42 && ledPos <= 48)
        distSwipeColor = secstrip.Color(145, 23, 227); //color

      else if (ledPos >= 49 && ledPos <= 55)
        distSwipeColor = secstrip.Color(87, 23, 227); //color

      else if (ledPos >= 56 && ledPos <= 61)
        distSwipeColor = secstrip.Color(23, 23, 227); //color

      else
        distSwipeColor = secstrip.Color(0, 0, 255); //color
      
      //LED brightness mapping algorithm
      dist = constrain(distance, 0, 255);
      ledPos = map(dist, 0, 255, 1, 60);
      ledBrightness = map(dist, 0, 255, 2, 60);
      ledcon = constrain(ledBrightness, 0, 59);

      scanColor = minstrip.Color(60 - ledcon, 0, 0);
      minstrip.setPixelColor(k / 2, scanColor);
      minstrip.show();


      for ( i = 0; i < HOURPIXELS; i++ )
      {
        hourstrip.setPixelColor(i, distSwipeColor);
      }
      hourstrip.show();


      for ( i = 0; i < SECPIXELS; i++ )
      {
        secstrip.setPixelColor(i, 0);
      }

      secstrip.setPixelColor(ledPos, distColor);
      secstrip.show();

      serialPrintRange(distance, ledPos, ledcon);

    }  // bottom of for
  }  // bottom of if

  delay(5000); //delay between back and forth scans

  for (i = 0; i < MINPIXELS; i++)
  {
    minstrip.setPixelColor(i, 0);
  }

  minstrip.show();

  nStep = -120;

  // number of steps and direction
  n = abs( nStep );
  dir = constrain( nStep, -1, 1 );

  // count steps
  if ( n > 0 )
  {
    for ( k = 0; k < n; k++ )
    {
      // next row in table
      row = row + dir;
      if ( row < 0 ) {
        row = 7;
      }
      if ( row > 7 ) {
        row = 0;
      }

      // send new position
      abcd = table[ row ];
      digitalWrite( coilA, bitRead( abcd, 3 ) );
      digitalWrite( coilB, bitRead( abcd, 2 ) );
      digitalWrite( coilC, bitRead( abcd, 1 ) );
      digitalWrite( coilD, bitRead( abcd, 0 ) );

      distance = lidarGetRange();
      
      if(distance < closeDist)
      {
        nearest = distance;
        closeDist = nearest;
        stepCount  = k;
      }

      if (ledPos >= 1 && ledPos <= 6)
        distSwipeColor = secstrip.Color(227, 23, 23); //color
      else if (ledPos >= 7 && ledPos <= 13)
        distSwipeColor = secstrip.Color(227, 23, 91); //color

      else if (ledPos >= 14 && ledPos <= 20)
        distSwipeColor = secstrip.Color(227, 23, 128); //color
      else if (ledPos >= 21 && ledPos <= 27)
        distSwipeColor = secstrip.Color(227, 23, 172); //color

      else if (ledPos >= 28 && ledPos <= 34)
        distSwipeColor = secstrip.Color(217, 23, 227); //color

      else if (ledPos >= 35 && ledPos <= 41)
        distSwipeColor = secstrip.Color(176, 23, 227); //color

      else if (ledPos >= 42 && ledPos <= 48)
        distSwipeColor = secstrip.Color(145, 23, 227); //color

      else if (ledPos >= 49 && ledPos <= 55)
        distSwipeColor = secstrip.Color(87, 23, 227); //color

      else if (ledPos >= 56 && ledPos <= 61)
        distSwipeColor = secstrip.Color(23, 23, 227); //color
      else
        distSwipeColor = secstrip.Color(0, 0, 255); //color

      dist = constrain(distance, 0, 255);
      ledPos = map(dist, 0, 255, 1, 60);
      ledBrightness = map(dist, 0, 255, 2, 100);
      ledcon = constrain(ledBrightness, 0, 59);

      scanColor = minstrip.Color(60 - ledcon, 0, 0);
      minstrip.setPixelColor((60 - (k / 2)), scanColor);
      minstrip.show();

      for ( i = 0; i < HOURPIXELS; i++ )
      {
        hourstrip.setPixelColor(i, distSwipeColor);
      }
      hourstrip.show();


      for ( i = 0; i < SECPIXELS; i++ )
      {
        secstrip.setPixelColor(i, 0);
      }

      secstrip.setPixelColor(ledPos, distColor);
      secstrip.show();

      serialPrintRange(distance, ledPos, ledcon);

    }  // bottom of for
  }  // bottom of if

  delay(5000);
  
  //point to nearest object
  printMode(6); //Print sub-mode
  minDist = stepCount; //pass on location of closest object
  moveStep(minDist,35); //call function to point at closest object
  
  //flash laser multiple times when pointing at closest object
  for(int z=0;z<3;z++)
  {
    digitalWrite(LASER, HIGH);
    delay(1000);
    digitalWrite(LASER,LOW);
    delay(1000);
  }
  
  //move back to home position
  minDist = - stepCount;
  moveStep(minDist,35);

  Background();
}

//////////////////////////////////////////////////////////////
// moveStep Function
// Moves to commanded position based on direction and magnitude of step value

void moveStep( int nStep, int delayStep ){

// declare local variables
  int i, n, dir;  // 16b signed

// number of steps and direction
  n = abs( nStep );
  dir = constrain( nStep, -1, 1 );

// count steps
  if( n > 0 ){
    for( i=0; i<n; i++ ){
      
// next row in table
      row = row + dir;
      if( row < 0 ){ row = 7; }
      if( row > 7 ){ row = 0; }

// send new position
      abcd = table[ row ];
      digitalWrite( coilA, bitRead( abcd, 3 ) );
      digitalWrite( coilB, bitRead( abcd, 2 ) );
      digitalWrite( coilC, bitRead( abcd, 1 ) );
      digitalWrite( coilD, bitRead( abcd, 0 ) );

//Serial.println(abcd);         // show time
      
// delay
      delay( delayStep );
    }  // bottom of for
  }  // bottom of if
}  // bottom of moveStep  

//System Shutdown function
void turnoff()
{
  setSecColor(0, 0, 0);
  setMinColor(0, 0, 0);
  setHourColor(0, 0, 0);
  setsecNoColor(0, 0, 0);
  setminNoColor(0, 0, 0);
  sethourNoColor(0, 0, 0);
  clearLED();
  showTime();
}


//LCD mode printing function
//Prints mode name based on unique integer value send to this function
//Pre-assigned mode-number mapping

void printMode(int mode)
{
  // clear screen
  lcd.write(0xFE);
  lcd.write(0x58);
  delay(10);   // we suggest putting delays after each command 
  
  // go 'home'
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(10);   // we suggest putting delays after each command 

  switch(mode)
  {
    case 1: 
      lcd.println("   TIME MODE   ");
      lcd.print("    ");
      lcd.print(hours);
      lcd.print(":");
      lcd.print(minutes);
      lcd.print(":");
      lcd.print(seconds);
      break;
    case 2:
      lcd.println(" RANDOMIZATION ");
      lcd.print("  COLOR SWIPE  ");
      break;
    case 3:
      lcd.println(" RANDOMIZATION ");
      lcd.print("   RAINBOW  ");
      break;
   case 4:
      lcd.println(" RANDOMIZATION ");
      lcd.print("  COLOR WHEEL ");
      break;
    case 5:
      lcd.println("LIDAR SCAN MODE");      
      break;
    case 6:
      lcd.println("  POINTING TO  ");
      lcd.print(" CLOSEST OBJECT");
      break;
    default:
      lcd.print("   WELCOME TO   ");
      lcd.println("INFINITY MIRROR");
      break;
  }
  
  delay(1000);
}
