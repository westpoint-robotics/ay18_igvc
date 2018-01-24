#include "LPD8806.h"
#include "SPI.h"
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

//define button type
bool leftButtonPressed;
bool rightButtonPressed;

// Number of RGB LEDs in strand:
int nLEDs = 4;

// Chose 2 pins for output; can be any valid output pins:
int dataPin  = 2;
int clockPin = 3;

int izzyState = 0; // General State Machine for Loops
int lastButton = 0; //Conditional Variable to Avoid Multiple Fires with Button Push
int batteryLife = 10; //Hard value for battery life --- temporary, eventually subscribe to battery 
int currentPixel = 0; //Pixel being lit up within colorChase
int pixelIteration = nLEDs; //Pixels in colorChase
unsigned long lastTime = millis(); //Initialize the timer for colorChase

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

void messageCb( const sensor_msgs::Joy& joy){
  if ((joy.buttons[1]) == 1 && lastButton == 0){
    
    if (izzyState == 0){
      izzyState = 1;
    }
    else{
      izzyState = 0;
    }
  }
  lastButton = joy.buttons[1];

}

  
ros::Subscriber<sensor_msgs::Joy> sub("joy", &messageCb );

void setup()
{ 
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
    // Start up the LED strip
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


/*// Chase one dot down the full strip.
void colorChase(uint32_t c, uint8_t wait) {
  int i;

  // Start by turning all pixels off:
  for(i=0; i<strip.numPixels(); i++) strip.setPixelColor(i, 0);

  // Then display one pixel at a time:
  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c); // Set new pixel 'on'
    strip.show();              // Refresh LED states
    strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
    delay(wait); //DON'T USE DELAY IN COLOR FUNCTION!!!
  }
  
  strip.show(); // Refresh to turn off last pixel
}*/


// Chase one dot down the full strip.
void colorChase(uint32_t c, uint8_t wait) {
  int i;

  // Start by turning all pixels off:
  for(i=0; i<strip.numPixels(); i++) strip.setPixelColor(i, 0);
  
  if (currentPixel >= pixelIteration){
    currentPixel = 0;
  }

  unsigned long currentTime = millis();
  if ((currentTime - lastTime) > 50) {
  // Then display one pixel at a time:
    if (currentPixel <= pixelIteration){
      strip.setPixelColor(currentPixel, c); // Set new pixel 'on'
      strip.show();              // Refresh LED states
      strip.setPixelColor(currentPixel, 0); // Erase pixel, but don't refresh!    
      currentPixel = currentPixel + 1;    
      lastTime = millis();
      //delay(wait); DON'T USE DELAY IN COLOR FUNCTION!!!
    }
  }
}



void loop() {
  nh.spinOnce();
  if (izzyState == 0){
      if (batteryLife >= 60){
        colorHold(strip.Color(  0,127,  0), 100); // Green for High Battery
      }
      else if (20 < batteryLife && batteryLife < 60){
        colorHold(strip.Color( 127, 127,   0), 100); // Yellow for Medium Battery
      }
      else{
        colorHold(strip.Color( 127,   0,   0), 100); // Red for Low Battery
      }
      //digitalWrite(13, HIGH-digitalRead(13));   // blink the led 
  }
  if (izzyState == 1){
      if (batteryLife >= 60){
        colorChase(strip.Color(  0,127,  0), 100); // Green for High Battery 
      }
      else if (20 < batteryLife && batteryLife < 60){
        colorChase(strip.Color( 127, 127,   0), 100); // Yellow for Medium Battery
      }
      else{
        colorChase(strip.Color( 127,   0,   0), 100); // Red for Low Battery
      //digitalWrite(13, LOW-digitalRead(13));   // blink the led 
      }
  }
  //delay(1);
}



