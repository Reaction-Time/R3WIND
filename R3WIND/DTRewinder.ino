#include <ClickEncoder.h>
#include <TimerOne.h>
#include <Stepper.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 6, 5, 4, 3); // setup lcd pins here

//magnet pins------------------------------------------------------------

#define magnetpin 2 // magnet signal pin
int dist = 0;
int magnet_detected = 0;
int magnet_detector = 0;
//Stepper pins------------------------------------------------------------

#define dirPin 8
#define stepPin 9
#define stepsPerRevolution 1600




//Encoder----------------------------------------------------------------
bool hasclicked = false;
bool lockout = false;
bool lockout2 = false;
bool buttonstat = false;
int wind;
int sensor;
ClickEncoder *encoder;
int16_t last, value;

void timerIsr() {
  encoder->service();
}















void setup() {

//Encoder----------------------------------------------------------------
Serial.begin(115200);
encoder = new ClickEncoder(A1, A0, A2);
Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 

 last = -1;
//sensor-----------------------------------------------------------------


  
   attachInterrupt(0, magnet_detect, RISING);//Initialize the intterrupt pin (Arduino digital pin 2)
   

  
//LCD SETUP---------------------------------------------------------------
    lcd.begin(16, 4);
      lcd.setCursor(1, 1);
 //        lcd.print("LCD Init-completed");
 //  delay(500);
 //   lcd.clear();
 //     lcd.print("R3winder Ready");
 //       lcd.setCursor(4, 1);
 //         lcd.print("wind filament");

lcd.clear();
     lcd.setCursor(0, 0);
       lcd.print("distance:");
         lcd.setCursor(0,-3); //x,y
          lcd.print("wind(M):");
          delay(100);
           lcd.print(value);

 
//Stepper setup------------------------------------------------------------

  pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:

 //ENCODER---------------------------------------
   
    
    value += encoder->getValue();
  
  if (value != last) {
    last = value;
    Serial.print("Encoder Value: ");
    Serial.println(value);
#ifdef WITH_LCD
    lcd.setCursor(0, 0);
    lcd.print("         ");
    lcd.setCursor(0, 0);
    lcd.print(value);
#endif
  }
  
  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open) {
    Serial.print("Button: ");
    #define VERBOSECASE(label) case label: Serial.println(#label); break;
    switch (b) {
      VERBOSECASE(ClickEncoder::Pressed);
      VERBOSECASE(ClickEncoder::Held)
      VERBOSECASE(ClickEncoder::Released)
      VERBOSECASE(ClickEncoder::Clicked)
      case ClickEncoder::DoubleClicked:
          Serial.println("ClickEncoder::DoubleClicked");
          encoder->setAccelerationEnabled(!encoder->getAccelerationEnabled());
          Serial.print("  Acceleration is ");
          Serial.println((encoder->getAccelerationEnabled()) ? "enabled" : "disabled");
        break;
    }
  }    









    
 if (lockout2 == false) //&& lockout2 == false) 
 // 
  {
     lcd.clear();
    lcd.setCursor(0, 0);
     lcd.print("distance:");
       lcd.setCursor(0,-3); //x,y
        lcd.print("wind(M):");
         delay(100);
         lcd.print(value);
           delay(100);
         }



if (b != ClickEncoder::Open && value > 0) 
{
  lockout2 = true;

    lcd.clear();
     lcd.setCursor(0, 0);
      lcd.print("winding in progress:");
      nema17();
      
      }






}
void  nema17() {

 magnet_detected = magnet_detector;
 

           while (magnet_detector < value) 
           {
            Serial.print(value);
          // These four lines result in 1 step:
    digitalWrite(dirPin, HIGH); //setting turning direction
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);


if (magnet_detector == value)
{
  
  lcd.clear();
     lcd.setCursor(0, 0);
       lcd.print("distance:");
         lcd.setCursor(0,-3); //x,y
          lcd.print("wind(M):");
          delay(100);
          value=0;
           lcd.print(value);

             value=0;
             magnet_detector = 0;
         lockout2 = false;
          loop();
             break;
}

    
}

//if (value == magnet_detect) 

//{    lcd.begin(16, 4);
//      lcd.setCursor(1, 1);
  //        lcd.print("LCD Init-completed");
  //  delay(500);
  //   lcd.clear();
  //     lcd.print("R3winder Ready");
  //       lcd.setCursor(4, 1);
  //         lcd.print("wind filament");

//lcd.clear();
//     lcd.setCursor(0, 0);
//       lcd.print("distance:");
//         lcd.setCursor(0,-3); //x,y
//          lcd.print("wind(M):");
//          delay(100);
//           lcd.print(value);

//           value=0;
 //          return;

 //}
  
  

}


void magnet_detect()//This function is called whenever a magnet/interrupt is detected by the arduino
 {
 magnet_detector = magnet_detector+1;
   Serial.print(magnet_detector);
 delay(400);
  }




//--------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// Rotary Encoder Driver with Acceleration
// Supports Click, DoubleClick, Long Click
//
// (c) 2010 karl@pitrich.com
// (c) 2014 karl@pitrich.com
// 
// Timer-based rotary encoder logic by Peter Dannegger
// http://www.mikrocontroller.net/articles/Drehgeber
// ----------------------------------------------------------------------------

#ifndef __have__ClickEncoder_h__
#define __have__ClickEncoder_h__

// ----------------------------------------------------------------------------

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"

// ----------------------------------------------------------------------------

#define ENC_NORMAL        (1 << 1)   // use Peter Danneger's decoder
#define ENC_FLAKY         (1 << 2)   // use Table-based decoder

// ----------------------------------------------------------------------------

#ifndef ENC_DECODER
#  define ENC_DECODER     ENC_NORMAL
#endif

#if ENC_DECODER == ENC_FLAKY
#  ifndef ENC_HALFSTEP
#    define ENC_HALFSTEP  1        // use table for half step per default
#  endif
#endif

// ----------------------------------------------------------------------------

class ClickEncoder
{
public:
  typedef enum Button_e {
    Open = 0,
    Closed,
    
    Pressed,
    Held,
    Released,
    
    Clicked,
    DoubleClicked
    
  } Button;

public:
  ClickEncoder(uint8_t A, uint8_t B, uint8_t BTN = -1, 
               uint8_t stepsPerNotch = 1, bool active = LOW);

  void service(void);  
  int16_t getValue(void);

#ifndef WITHOUT_BUTTON
public:
  Button getButton(void);
#endif

#ifndef WITHOUT_BUTTON
public:
  void setDoubleClickEnabled(const bool &d)
  {
    doubleClickEnabled = d;
  }

  const bool getDoubleClickEnabled()
  {
    return doubleClickEnabled;
  }
#endif

public:
  void setAccelerationEnabled(const bool &a)
  {
    accelerationEnabled = a;
    if (accelerationEnabled == false) {
      acceleration = 0;
    }
  }

  const bool getAccelerationEnabled() 
  {
    return accelerationEnabled;
  }

private:
  const uint8_t pinA;
  const uint8_t pinB;
  const uint8_t pinBTN;
  const bool pinsActive;
  volatile int16_t delta;
  volatile int16_t last;
  uint8_t steps;
  volatile uint16_t acceleration;
#if ENC_DECODER != ENC_NORMAL
  static const int8_t table[16];
#endif
#ifndef WITHOUT_BUTTON
  volatile Button button;
  bool doubleClickEnabled;
  bool accelerationEnabled;
#endif
};

// ----------------------------------------------------------------------------

#endif // __have__ClickEncoder_h__
