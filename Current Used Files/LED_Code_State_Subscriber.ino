#include "LPD8806.h"
#include "SPI.h"
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>


//define logic state:
bool autoMode;

// Number of RGB LEDs in strand:
int nLEDs = 70;

// Chose 2 pins for output; can be any valid output pins:
int dataPin  = 2;
int clockPin = 3;

int izzyState = 0; //General State Machine for Loops
int controlRepeat = 0; //Conditional Variable to Avoid Blinking During Manual Mode, Starts in Manual Mode
int lastButton = 0; //Conditional Variable to Avoid Multiple Fires with Button Push
uint8_t batteryLife = 100; //Hard value for battery life --- temporary, eventually subscribe to battery 
uint8_t oldBatteryLife = 100; //Variable to compare battery life and determine if color change is needed


// First parameter is the number of LEDs in the strand.  The LED strips
// are 32 LEDs per meter but you can extend or cut the strip.  Next two
// parameters are SPI data and clock pins:
LPD8806 strip = LPD8806(nLEDs, dataPin, clockPin);

// You can optionally use hardware SPI for faster writes, just leave out
// the data and clock pin parameters.  But this does limit use to very
// specific pins on the Arduino.  For "classic" Arduinos (Uno, Duemilanove,
// etc.), data = pin 11, clock = pin 13.  For Arduino Mega, data = pin 51,
// clock = pin 52.  For 32u4 Breakout Board+ and Teensy, data = pin B2,
// clock = pin B1.  For Leonardo, this can ONLY be done on the ICSP pins.
//LPD8806 strip = LPD8806(nLEDs);

ros::NodeHandle  nh;

void messageCb( const std_msgs::Bool& autoState){   //State Callback
  if (autoState.data){
      izzyState = 1;
      controlRepeat = 0;
    }
  else {
      izzyState = 0;
    }
  }


void messageCb2( const std_msgs::UInt8& batteryPercent){
   oldBatteryLife = batteryLife;
   batteryLife = uint8_t(batteryPercent.data);
}
  
ros::Subscriber<std_msgs::Bool> stateSub("autoState", &messageCb );
ros::Subscriber<std_msgs::UInt8> batterySub("batteryPercent", &messageCb2);

void setup()
{ 
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);
  nh.initNode();
  nh.subscribe(stateSub);
    // Start up the LED strip
  nh.subscribe(batterySub);
  strip.begin();
  // Update the strip, to start they are all 'off'
  strip.show();
}



// Display States for Strip of LPD8806-based RGB LED
// Not compatible with Trinket/Gemma due to limited RAM

/*****************************************************************************/

// Hold all dots for full strip.
void colorHold(uint32_t c, uint8_t wait) {
  int i;
  
  // Start by turning all pixels off:
  for(i=0; i<strip.numPixels(); i++) strip.setPixelColor(i, 0);

  // Then turn on one pixel at a time:
  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c); // Set new pixel 'on'
    strip.show();              // Refresh LED states
    //delay(wait); DON'T USE DELAY IN COLOR FUNCTION!!!
  }
}



void loop() {
  nh.spinOnce();
  if (izzyState == 0){ // Manual Mode
      if (batteryLife >= 60){
        if (controlRepeat == 0){
          colorHold(strip.Color(  0,127,  0), 100); // Green for High Battery
          controlRepeat = 1;
        }
        if (oldBatteryLife<60){
          colorHold(strip.Color(  0,127,  0), 100); // Green for High Battery
        }
      }
      else if (20 < batteryLife && batteryLife < 60){
        if (controlRepeat == 0){
          colorHold(strip.Color( 127, 30,   0), 100); // Orange-Yellow for Medium Battery
          controlRepeat = 1;
        }
        if (oldBatteryLife<=20){
          colorHold(strip.Color( 127, 30,   0), 100); // Orange-Yellow for Medium Battery
        }
        if (oldBatteryLife>=60){
          colorHold(strip.Color( 127, 30,   0), 100); // Orange-Yellow for Medium Battery
        }
      }
      else{
        if (controlRepeat == 0){
          colorHold(strip.Color( 127,   0,   0), 100); // Red for Low Battery
          controlRepeat = 1;
        }
        if (oldBatteryLife>20){
          colorHold(strip.Color( 127,   0,   0), 100); // Red for Low Battery
        }
      }
      //digitalWrite(13, HIGH-digitalRead(13));   // blink the led 
  }
  if (izzyState == 1){ // Autonomous Mode
      if (batteryLife >= 60){
        colorHold(strip.Color(  0,127,  0), 100); // Green for High Battery 
      }
      else if (20 < batteryLife && batteryLife < 60){
        colorHold(strip.Color( 127, 30,   0), 100); // Orange-Yellow for Medium Battery
      }
      else{
        colorHold(strip.Color( 127,   0,   0), 100); // Red for Low Battery
      //digitalWrite(13, LOW-digitalRead(13));   // blink the led 
      }
  }
  //delay(1);
}



