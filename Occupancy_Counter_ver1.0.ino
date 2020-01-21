#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_7segment matrix = Adafruit_7segment();

//PINOUT
//--------------------------------------------
//0 - 
//1 - 
//2 - 
//3 - 
//4 - 
//5 - 
//6 - Laser 1
//7 - Laser 2
//8 - RED button
//9 - RED button LED
//10 - GREEN button
//11 - GREEN button LED
//12 - External Connector
//13 - Indicator LED
//14 (A0)- Photoresistor 1a   
//15 (A1)- Photoresistor 1b
//16 (A2)- Photoresistor 2a
//17 (A3)- Photoresistor 2b
//18 (A4)- 7-Segment pin 3
//19 (A5)- 7-Segment pin 4

//7-Segment Wiring Instructions
//------------------------------------
//Turn display so numbers are upright
//Pin 1 - 5V
//Pin 2 - GND
//Pin 3 - A4
//Pin 4 - A5

//Pin Declarations
//------------------------------------
int photoResistorPin1a = 0; //(A0) Input pin for photoresistor 1a (outside room)
int photoResistorPin1b = 1; //(A1) Input pin for photoresistor 1b (outside room)
int photoResistorPin2a = 2; //(A2) Input pin for photoresistor 2a (inside room)
int photoResistorPin2b = 3; //(A3) Input pin for photoresistor 2b (inside room)

int laser1 = 6; //Output pin to control laser
int laser2 = 7; //Output pin to control laser

int buttonRED = 8;
int buttonREDled = 9;

int buttonGREEN = 10;
int buttonGREENled = 11;

int outputSignal = 12;
int indicatorLED = 13;


//Program Variables
//------------------------------------------------------------------------------

//Sensor Readings
int sensorTemp1 = 0;    //Temporary value of sensor reading
int sensorTemp2 = 0;    //Temporary value of sensor reading

//Sensor Comparison Values
int sensor1 = 0;        //Value from photoresistors 1 a&b
int sensor2 = 0;        //Value from photoresistors 2 a&b

//Treshold value
int threshold = 350;       //Light intensity threshold (replace with setup mode)

//Button Variables
int buttonValueG = 0;   //HIGH or LOW value read from the button
int buttonValueR = 0;   //HIGH or LOW value read from the button
int LEDcounter = 200;   //# of cycles delay before turning the LED off
int buttonGREENled_counter = 0; //Button counter value to be compared to LEDcounter
int buttonREDled_counter = 0;   //Button counter value to be compared to LEDcounter

//Conditions and Stages (entering/exiting & how far through the doorway)
int condition = 0;         //Condition number (0,1,or 2)
                                //Condition 0 -> Not in doorway
                                //Condition 1 -> Entering doorway from outside
                                //Condition 2 -> Entering doorway from inside
int stage = 0;             //Stage in the checking process
                                //Stage 0 -> No passage in progress
                                //Stage 1 -> First laser blocked
                                //Stage 2 -> Both lasers blocked
                                //Stage 3 -> Second laser blocked
//Counter Variables
int counter = 0;            //Counter to count the number of people in the room
int counterMAX = 1000;      //Counter cannot go above this value


void setup() {

  pinMode(photoResistorPin1a, INPUT);
  pinMode(photoResistorPin1b, INPUT);
  pinMode(photoResistorPin2a, INPUT);
  pinMode(photoResistorPin2b, INPUT);

  pinMode(buttonGREEN, INPUT);
  pinMode(buttonRED, INPUT);

  pinMode(buttonGREENled, OUTPUT);
  pinMode(buttonREDled, OUTPUT);

  pinMode(laser1, OUTPUT);
  pinMode(laser2, OUTPUT);

  pinMode(outputSignal, OUTPUT);
  pinMode(indicatorLED, OUTPUT);

  //Start the display
  matrix.begin(0x70);
  
  // ------------------------------------------------------------
  // Setup Timer 1 for an Interrupt Frequency 350Hz
  // ------------------------------------------------------------

  // TIMER 1 for interrupt frequency 350.00218751367197 Hz:
      cli(); // stop interrupts
      TCCR1A = 0; // set entire TCCR1A register to 0
      TCCR1B = 0; // same for TCCR1B
      TCNT1  = 0; // initialize counter value to 0
      // set compare match register for 350.00218751367197 Hz increments
      OCR1A = 45713; // = 16000000 / (1 * 350.00218751367197) - 1 (must be <65536)
      // turn on CTC mode
      TCCR1B |= (1 << WGM12);
      // Set CS12, CS11 and CS10 bits for 1 prescaler
      TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
      // enable timer compare interrupt
      TIMSK1 |= (1 << OCIE1A);
      sei(); // allow interrupts
  // ------------------------------------------------------------
 
}

void loop() {

  buttonValueG = digitalRead(buttonGREEN);
  buttonValueR = digitalRead(buttonRED);

  if(buttonValueG)
  {
    digitalWrite(buttonGREENled,HIGH);    //Turn LED on
    counter++;                            //Increment counter
    if(counter > counterMAX)              //Prevent the counter from going above the max
        counter--;
    matrix.print(counter,DEC);            //Write counter value to display
    matrix.writeDisplay();                //Update display value
    buttonValueG = 0;                     //Reset button value
    delay(500);                           //Wait before proceeding in the program
    digitalWrite(buttonGREENled,LOW);     //Turn LED off
  }

  if(buttonValueR)
  {
    digitalWrite(buttonREDled,HIGH);    //Turn LED on
    buttonValueR = 0;                   //Reset button value
    counter = 0;                        //Reset counter
    delay(500);                         //Wait before proceeding in the program
    digitalWrite(buttonREDled,LOW);     //Turn LED off
  }

  if(buttonGREENled_counter <= 0)
    digitalWrite(buttonGREENled, LOW);

  if(buttonREDled_counter <= 0)
    digitalWrite(buttonREDled, LOW);

  matrix.print(counter,DEC);
  matrix.writeDisplay();  

  if(counter > 0)
  {
    digitalWrite(indicatorLED, HIGH);
    digitalWrite(outputSignal, HIGH);
  }
  else
  {
    digitalWrite(indicatorLED, LOW);
    digitalWrite(outputSignal, LOW);
  }
}

// Interrupt Service Routine
// ----------------------------------------------------------
// This ISR does the following steps:
// 1. Turn on laser 1(Eventually), read a value from photoresistor 1,
//    and turn off the laser.
// 2. (Eventually) Turn on laser 2, read a value from photoresistor 2,
//    and turn off the laser. 
// 3. Send the sensor data over the serial line
// ----------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
  digitalWrite(laser1, HIGH);
  sensorTemp1 = analogRead(photoResistorPin1a); //Measure sensor a
  sensorTemp2 = analogRead(photoResistorPin1b); //Measure sensor b
  sensor1 = max(sensorTemp1,sensorTemp2); //Take the larger value
  
  digitalWrite(laser2, HIGH);
  sensorTemp1 = analogRead(photoResistorPin2a); //Measure sensor a
  sensorTemp2 = analogRead(photoResistorPin2b); //Measure sensor b
  sensor2 = max(sensorTemp1,sensorTemp2); //Take the larger value

  
  // Check Condition of Sensor Values
  // ------------------------------------------------ 
  if(condition == 0)
  {
    if(sensor1 < threshold)
    {
      condition = 1;
      stage = 1;   
    }
    else if(sensor2 < threshold)
    {
      condition = 2;
      stage = 1;
    }
  }
  else if(condition == 1)
  {
    switch(stage)
    {
      case 1: stage1(sensor1,sensor2);
              break;
      case 2: stage2(sensor1,sensor2);
              break;
      case 3: stage3(sensor1,sensor2);
              break;
    }
  }
  else if(condition == 2) // having this after the other direction may slow it down
  {
    switch(stage)
    {
      case 1: stage1(sensor2,sensor1);
              break;
      case 2: stage2(sensor2,sensor1);
              break;
      case 3: stage3(sensor2,sensor1);
              break;
    }
  }
  //If the green LED has been turned on, 
  //decrement the button LED counter
  if(buttonGREENled_counter > 0)
    buttonGREENled_counter--;

  //If the red LED has been turned on, 
  //decrement the button LED counter
  if(buttonREDled_counter > 0)
    buttonREDled_counter--;
}

//First laser is blocked
void stage1(int value1, int value2)
{
  if(value2 < threshold)
  {
    stage = 2;
  }
  else if(value1 > threshold)
  {
    condition = 0;    
    stage = 0;
  }
}

//Both lasers is blocked
void stage2(int value1, int value2)
{
  if(value1 > threshold)
  {
    stage = 3;
  }
  else if(value2 > threshold)
  {
    stage = 1;
  }
}

//Only second laser is blocked
void stage3(int value1, int value2)
{
  if(value2 > threshold)
  {
    
    if(condition == 1)
    {
      counter++;
      if(counter > counterMAX)              //Prevent counter from going above the max
        counter--;

      digitalWrite(buttonGREENled, HIGH);   //Turn LED on
      buttonGREENled_counter = LEDcounter;  //Set LED counter value to begin counting down    
    }
    else if(condition == 2)
    {
      counter--;
      
      if(counter < 0)
        counter++;

      digitalWrite(buttonREDled, HIGH);   //Turn LED on
      buttonREDled_counter = LEDcounter;  //Set LED counter value to begin counting down
    }

    condition = 0;
    stage = 0;    
  }
  else if(value1 < threshold)
  {
    stage = 2;
  }
}
