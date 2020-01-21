#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_7segment matrix = Adafruit_7segment();

//Arduino Uno PINOUT
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


//7-Segment Display PINOUT
//------------------------------------
//Turn display so numbers are upright
//Pin 1 - 5V
//Pin 2 - GND
//Pin 3 - A4
//Pin 4 - A5


//Pin Declarations
//------------------------------------------------------------------------------
int LDRpin1a = 0;         //(A0) Input pin for photoresistor 1a (outside room)
int LDRpin1b = 1;         //(A1) Input pin for photoresistor 1b (outside room)
int LDRpin2a = 2;         //(A2) Input pin for photoresistor 2a (inside room)
int LDRpin2b = 3;         //(A3) Input pin for photoresistor 2b (inside room)

int laser1 = 6;           //Output pin to control laser
int laser2 = 7;           //Output pin to control laser

int buttonRED = 8;        //Input pin for the RED button
int buttonREDled = 9;     //Output pin to control the RED button LED

int buttonGREEN = 10;     //Input pin for the GREEN button
int buttonGREENled = 11;  //Output pin to control the GREEN button LED

int outputSignal = 12;    //Output pin to control the relay
int indicatorLED = 13;    //Output pin to control the WHITE indicator LED 


//Program Variables
//------------------------------------------------------------------------------

//LDR Raw Values
int LDR1a = 0;                    //Current sensor reading
int LDR1b = 0;                    //Current sensor reading
int LDR2a = 0;                    //Current sensor reading
int LDR2b = 0;                    //Current sensor reading

//Sensor Comparison Values
int sensor1 = 0;                  //Value from photoresistors 1 a&b
int sensor2 = 0;                  //Value from photoresistors 2 a&b

//Sensor HIGH/LOW Values
int sensor1HIGH = 0;              //Sensor 1 value when the lasers are shining on it
int sensor2HIGH = 0;              //Sensor 2 value when the lasers are shining on it
int sensor1LOW = 0;               //Sensor 1 value when the lasers are blocked
int sensor2LOW = 0;               //Sensor 2 value when the lasers are blocked

//Treshold values
int threshold1 = 0;               //Light intensity threshold for sensor 1 (set during calibration)
int threshold2 = 0;               //Light intensity threshold for sensor 2 (set during calibration)

//Button Variables
int buttonValueG = 0;             //HIGH/LOW value read from the GREEN button
int buttonValueR = 0;             //HIGH/LOW value read from the RED button
int LEDcounter = 200;             //# of cycles delay before turning the LED off
int buttonGREENled_counter = 0;   //Button counter value to be compared to LEDcounter
int buttonREDled_counter = 0;     //Button counter value to be compared to LEDcounter
int button2 = 100;                //Delay time to wait for a second button press
int buttonDelay = 500 - button2;  //Delay time to wait after a button is pressed
int buttonMode = 0;               //Mode to specify which button was pressed
                                      // 0 = NEITHER pressed
                                      // 1 = GREEN   pressed
                                      // 2 = RED     pressed
                                      // 3 = BOTH    pressed

//Setup Variables
bool newConnection = true;        //Specify if troubleshooting mode has been enabled since powering on the device
bool troubleshoot = false;        //If the device cannot be calibrated, the user can enter troubleshooting mode
int troubleshootToggle = 1;       //Variable to toggle every time the device is set to troubleshooting mode
int troubleshootCounter = 0;      //Count how many times troubleshooting is opened
int stepDelay = 100;              //Delay time between setup steps

//Conditions and Stages
int condition = 0;                //Condition number
                                      //Condition 0 -> Not in doorway
                                      //Condition 1 -> Entering doorway from outside
                                      //Condition 2 -> Entering doorway from inside
int stage = 0;                    //Stage in the process of passing through
                                      //Stage 0 -> No passage in progress
                                      //Stage 1 -> Only First laser blocked
                                      //Stage 2 -> Both lasers blocked
                                      //Stage 3 -> Only Second laser blocked

//Arm Swinging Compensation
int delayCounter = 0;             //Delay Counter will count down from delayCounterValue
int delayCounterValue = 0.7*350;  //(# seconds delay) * 350Hz = number of delay cycles
int durationCounter = 0;          //This counter keeps track of how long the lasers are blocked
int durationMAX = 0;              //During the delay period, this records the max duration if multiple passes
int durationMAXvalue = 0;         //During the delay period, this records the value (+1 or -1) for the max duration


//Counter Variables
int counter = 0;                  //Counter to record the number of people in the room
int counterPrev = 0;              //Previous counter value
int counterMAX = 1000;            //Counter cannot go above this value



void setup() {

  // ------------------------------------------------------------
  //Set the Pin Modes
  // ------------------------------------------------------------
  pinMode(LDRpin1a, INPUT);
  pinMode(LDRpin1b, INPUT);
  pinMode(LDRpin2a, INPUT);
  pinMode(LDRpin2b, INPUT);

  pinMode(buttonGREEN, INPUT);
  pinMode(buttonRED, INPUT);

  pinMode(buttonGREENled, OUTPUT);
  pinMode(buttonREDled, OUTPUT);

  pinMode(laser1, OUTPUT);
  pinMode(laser2, OUTPUT);

  pinMode(outputSignal, OUTPUT);
  pinMode(indicatorLED, OUTPUT);


  // ------------------------------------------------------------
  //Start the display
  // ------------------------------------------------------------
  matrix.begin(0x70);


  // ------------------------------------------------------------
  // Device Setup
  // ------------------------------------------------------------
  
  // Entrance Animation
  // ------------------------------------------------------------
  animateLoading(50);  
  animateSetup();  
  animateCurtains(80);

  // Select the Entrance Direction
  // ------------------------------------------------------------
  selectDirection();

  // Aim the Lasers
  // ------------------------------------------------------------
  aimLasers();
  
  // Calibrate Sensors
  // ------------------------------------------------------------
  calibrateSensors();

  
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

void loop() 
{
  // ----------------------------------------------------
  // Check which buttons were pressed
  // ----------------------------------------------------
  buttonMode = checkButtons();

  // ----------------------------------------------------
  // Evaluate the Button Mode and Perform the Operations
  // ----------------------------------------------------
  switch (buttonMode)
  {
    //Only GREEN button was pressed
    case 1: 
      counterPrev = counter;                      //Save the previous counter value
      counter++;                                  //Increment counter
      if(counter > counterMAX)                    //Prevent the counter from going above the max
          counter--;
      matrix.print(counter,DEC);                  //Display counter value
      matrix.writeDisplay();                      //Update display
      delay(buttonDelay);                         //Wait before proceeding
      digitalWrite(buttonGREENled,LOW);           //Turn LED off
      break;

    //Only RED button was pressed
    case 2: 
      counterPrev = counter;                      //Save the previous counter value
      counter--;                                  //Decrement counter
      if(counter < 0)                             //Prevent the counter from going negative
          counter++;
      matrix.print(counter,DEC);                  //Display counter value
      matrix.writeDisplay();                      //Update display
      delay(buttonDelay);                         //Wait before proceeding
      digitalWrite(buttonREDled,LOW);             //Turn LED off
      break;

    //BOTH buttons were pressed
    case 3: 
      counter = 0;                                //Reset the counter to zero
      matrix.print(counter,DEC);                  //Display counter value
      matrix.writeDisplay();                      //Update display
      delay(buttonDelay);                         //Wait before proceeding
      digitalWrite(buttonGREENled,LOW);           //Turn GREEN LED off
      digitalWrite(buttonREDled,LOW);             //Turn RED LED off
      digitalWrite(indicatorLED, LOW);            //Turn indicator LED off
      digitalWrite(outputSignal, LOW);            //Turn outputSignal off
      
      //Wait for BOTH buttons to be released
      do
      {
        buttonValueG = digitalRead(buttonGREEN);  //Read the GREEN button
        buttonValueR = digitalRead(buttonRED);    //Read the RED button
        counter = 0;                              //Reset the counter again
        digitalWrite(buttonGREENled,LOW);         //Turn LED off if necessary
        digitalWrite(buttonREDled,LOW);           //Turn LED off if necessary
      }while(buttonValueG || buttonValueR);       //Stay in the loop as long as at least one button is pressed
      counter = 0;                                //Reset the counter again in case a person passed through 
      break;
  }

  // ------------------------------------------------
  // Turn the button LEDs off if it is time
  // ------------------------------------------------
  if(buttonGREENled_counter <= 0)
    digitalWrite(buttonGREENled, LOW);

  if(buttonREDled_counter <= 0)
    digitalWrite(buttonREDled, LOW);

  // ------------------------------------------------
  // Update the 7-Segment Display
  // ------------------------------------------------
  matrix.print(counter,DEC);
  matrix.writeDisplay();  

  // ------------------------------------------------
  // Update the Indicator LED and Output Signal
  // ------------------------------------------------
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

  // ------------------------------------------------
  // Send Serial Data
  // ------------------------------------------------  
  sendSerialData(); //Comment out this line to disable normal serial communication
  
}



// -------------------------------------------------------------
// Interrupt Service Routine
// -------------------------------------------------------------
// This ISR does the following steps:
// 1. Check the sensor values
// 2. Evaluate the sensor data
// 3. Proceed to the next stage
// -------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
  //Check the Sensors
  checkSensors();

  //Check Delay Counter and decrement if it is greater than zero
  if(delayCounter > 0)
    delayCounter--;
  
  // Check Condition of Sensor Values
  // ------------------------------------------------ 
  if(condition == 0)  //No one in the doorway
  {
    //ENTERING ROOM
    if(sensor1 < threshold1)
    {
      condition = 1;  //Person entering the room
      stage = 1;      //First laser blocked
    }
    //EXITING ROOM
    else if(sensor2 < threshold2)
    {
      condition = 2;  //Person entering the room
      stage = 1;      //First laser blocked
    }

    //If the lasers have been blocked, start the duration counter
    if(condition != 0)
      durationCounter++;
  }
  //ENTERING ROOM
  else if(condition == 1)
  {
    durationCounter++;  //Increment the duration counter
    
    switch(stage)
    {
      case 1: stage1(sensor1,sensor2,threshold1,threshold2);
              break;
      case 2: stage2(sensor1,sensor2,threshold1,threshold2);
              break;
      case 3: stage3(sensor1,sensor2,threshold1,threshold2);
              break;
    }
  }
  //EXITING ROOM
  else if(condition == 2) // having this after the other direction may slow it down
  {
    durationCounter++;  //Increment the duration counter
    
    switch(stage)
    {
      case 1: stage1(sensor2,sensor1,threshold2,threshold1);
              break;
      case 2: stage2(sensor2,sensor1,threshold2,threshold1);
              break;
      case 3: stage3(sensor2,sensor1,threshold2,threshold1);
              break;
    }
  }
  
  //If the GREEN LED has been turned on, 
  //decrement the button LED counter
  if(buttonGREENled_counter > 0)
    buttonGREENled_counter--;

  //If the RED LED has been turned on, 
  //decrement the button LED counter
  if(buttonREDled_counter > 0)
    buttonREDled_counter--;
}



// ------------------------------------------------------------
// Stage Functions
// ------------------------------------------------------------

//First laser is blocked
void stage1(int value1, int value2, int T1, int T2)
{
  //The person is continuing forward
  if(value2 < T2)
  {
    stage = 2;
  }
  //The person has backed up out of the doorway
  else if(value1 > T1)
  {
    condition = 0;    
    stage = 0;
  }
}

//Both lasers are blocked
void stage2(int value1, int value2, int T1, int T2)
{
  //The person is continuing forward
  if(value1 > T1)
  {
    stage = 3;
  }
  //The person has backed up
  else if(value2 > T2)
  {
    stage = 1;
  }
}

//Only second laser is blocked
void stage3(int value1, int value2, int T1, int T2)
{
  //The person has gone all the way through the doorway
  if(value2 > T2)
  {
    //If they was entering, the counter will increase
    if(condition == 1)
    {
      //If the delayCounter has run down, count normally
      if(delayCounter == 0)
      {
        counterPrev = counter;                //Save the previous counter value
        counter++;
        if(counter > counterMAX)              //Prevent counter from going above the max
          counter--;

        delayCounter = delayCounterValue;     //Set the delayCounter
        durationMAX = durationCounter;        //Save the duration counter value
        durationMAXvalue = 1;                 //Save the value from this duration
        durationCounter = 0;                  //Reset the duration counter
        
        digitalWrite(buttonGREENled, HIGH);   //Turn LED on
        buttonGREENled_counter = LEDcounter;  //Set LED counter value to begin counting down
      }
      //If the delayCounter is still going identify the max duration and correct any mistakes
      else
      {
        //If this time the lasers are blocked for longer,
        //update the durationMAXvalue to reflect this new value
        //and if the previous value was -1, correct the counter
        if(durationCounter > durationMAX)
        {
          durationMAX = durationCounter;        //Update durationMAX
          
          //If the counter was decremented last time, correct the error
          if(durationMAXvalue == -1)
          {
            //Correct the counter
            durationMAXvalue = 1;               //Save the new value
            counter = counterPrev;              //Set the counter back to the previous value
            counter++;                          //Increment the counter
            if(counter > counterMAX)            //Make sure the counter cannot go above the max
              counter = counterMAX;

            //Correct the LEDs
            digitalWrite(buttonREDled,  LOW);   //Turn RED LED off
            digitalWrite(buttonGREENled,HIGH);  //Turn GREEN LED on
            buttonREDled_counter = 0;           //Reset the red LED counter
            buttonGREENled_counter = LEDcounter;//Set LED counter value to begin counting down
          }
          durationCounter = 0;                  //Reset the duration counter
        }
      }
    }
    //If they was exiting, the counter will decrease
    else if(condition == 2)
    {
      //If the delay counter has run down, count normally
      if(delayCounter == 0)
      {
        counterPrev = counter;              //Save the previous counter value
        counter--;
        if(counter < 0)                     //Prevent counter from going negative
          counter = 0;

        delayCounter = delayCounterValue;   //Set the delayCounter
        durationMAX = durationCounter;      //Save the duration counter value
        durationMAXvalue = -1;              //Save the value from this duration
        durationCounter = 0;                //Reset the duration counter
  
        digitalWrite(buttonREDled, HIGH);   //Turn LED on
        buttonREDled_counter = LEDcounter;  //Set LED counter value to begin counting down
      }
      //If the delayCounter is still going identify the max duration and correct any mistakes
      else
      {
        //If this time the lasers were blocked for longer,
        //update the durationMAXvalue to reflect this new value
        //and if the previous value was +1, correct the counter
        if(durationCounter > durationMAX)
        {
          durationMAX = durationCounter;        //Update durationMAX
          
          //If the counter was incremented last time, correct the error
          if(durationMAXvalue == 1)
          {
            //Correct the counter
            durationMAXvalue = -1;              //Save the new value
            counter = counterPrev;              //Set the counter back to the previous value
            counter--;                          //Decrement the counter
            if(counter < 0)                     //Make sure the counter cannot be negative
              counter = 0;

            //Correct the LEDs
            digitalWrite(buttonGREENled,LOW);   //Turn GREEN LED off
            digitalWrite(buttonREDled, HIGH);   //Turn RED LED on
            buttonGREENled_counter = 0;         //Reset the green LED counter
            buttonREDled_counter = LEDcounter;  //Set LED counter value to begin counting down
          }
          durationCounter = 0;                  //Reset the duration counter
        }
      }
    }

    condition = 0;        //Reset the condition to zero (doorway empty)
    stage = 0;            //Reset the stage to zero (doorway empty)
  }
  //The person has backed up
  else if(value1 < T1)
  {
    stage = 2;
  }
}



// ------------------------------------------------------------
// Setup Functions
// ------------------------------------------------------------

//This function allows the user to select the entrance direction
void selectDirection()
{
  digitalWrite(buttonGREENled,HIGH);  //Turn LED on

  int defaultSetup = 1;   //Default setup(1 = TRUE) or alternate setup (0 = FALSE)

  //This loop will execute until the GREEN button is pressed
  do {
    
    //Check the RED button
    buttonValueR = digitalRead(buttonRED);

    //If the RED button is pressed, toggle the setup
    if(buttonValueR)
    {
      defaultSetup = !defaultSetup; //Toggle defaultSetup
      
      //Blink the GREEN LED to confirm that the setup was changed
      digitalWrite(buttonGREENled,LOW);
      delay(80);
      digitalWrite(buttonGREENled,HIGH);
      delay(80);
      digitalWrite(buttonGREENled,LOW);
      delay(80);
      digitalWrite(buttonGREENled,HIGH);

      //Check the RED button again
      buttonValueR = digitalRead(buttonRED);
      
      //Wait for the RED button to be released
      while(buttonValueR)
      {
        animateDoorway(defaultSetup,100);
        delay(200);
        buttonValueR = digitalRead(buttonRED);
      }
    }

    //While the RED button is not being pressed,
    //blink the RED LED and animate the display
    digitalWrite(buttonREDled,HIGH);    //Turn LED on 
    animateDoorway(defaultSetup,100);   //Animate the direction
    digitalWrite(buttonREDled,LOW);     //Turn LED off
    delay(200);

    //Check the GREEN button
    buttonValueG = digitalRead(buttonGREEN);
  } while(!buttonValueG);

  clearDisplay();

  //If default setup is selected
  //Nothing changes

  //If alternate setup is selected, reassign the LDR pins
  if(!defaultSetup)
  { 
    //Set sensor and laser pins to alternate configuration
    LDRpin1a = 2; //(A2) 
    LDRpin1b = 3; //(A3) 
    LDRpin2a = 0; //(A0) 
    LDRpin2b = 1; //(A1) 
    laser1 = 7;
    laser2 = 6;
  } 

  //Confirm that the GREEN button was pressed
  digitalWrite(buttonGREENled,LOW);     //Turn LED off

  //Wait until the GREEN button is released
  waitGREENbutton();

  //delay(stepDelay); //Short delay before proceeding
}


//This function allows the user to aim the lasers
//If the RED button is pressed, they can go back to the previous step
void aimLasers()
{
  bool goToNextStep = false;

  //The program will stay in this loop until 
  //the GREEN button is pressed to go on to the next step
  do {
    
    // Turn on lasers for aiming
    digitalWrite(laser1,HIGH);
    digitalWrite(laser2,HIGH);
  
    // Turn LEDs on
    digitalWrite(buttonGREENled,HIGH);
    digitalWrite(buttonREDled,HIGH);
     
    // Wait for the buttons to be pressed
    do {
      animateLasers(80);            //Animate the display
      buttonMode = checkButtons();  //Check for button presses
      
    } while(buttonMode == 0 || buttonMode == 3);

    //Turn the LEDs off
    digitalWrite(buttonGREENled,LOW);     //Turn LED off
    digitalWrite(buttonREDled,LOW);       //Turn LED off

    //If the GREEN button was pressed, go to next step
    if(buttonMode == 1)
    {    
      //Display the lasers aimed
      displayLasersON();
      delay(200);
      clearDisplay();

      //Wait until the green button is released
      waitGREENbutton();

      //Go to the next step
      goToNextStep = true;
    }
    
    //If the RED button was pressed, go back to the previous step
    else if(buttonMode == 2)
    {
      // Turn lasers off
      digitalWrite(laser1,LOW);
      digitalWrite(laser2,LOW);

      //Clear the display
      clearDisplay();

      //Wait until the red button is released
      waitREDbutton();

      //Go back to select the direction again
      selectDirection();
      
      //delay(stepDelay); //Short delay before proceeding
    }
  } while(!goToNextStep);

  //delay(stepDelay); //Short delay before proceeding
}


//This function calibrates the sensors
void calibrateSensors()
{
  //Measure HIGH level light intensity
  //------------------------------------
  startCalibration();

  //Wait for the sensors to be blocked
  //-----------------------------------
  do {

    //Check for button presses
    //-------------------------
    buttonMode = checkButtons();        //Check for button presses

    
    //If the RED button is pressed, GO BACK ONE STEP to aim the lasers again
    //-----------------------------------------------------------------------
    if(buttonMode == 2)
    {
      digitalWrite(buttonREDled,LOW);   //Turn the RED led off
      waitREDbutton();                  //Wait for the button to be released
      
      aimLasers();                      //Aim the lasers again
      startCalibration();               //Start the calibration again
    }
    
    //If the user presses BOTH buttons, ENTER TROUBLESHOOTING MODE
    //-------------------------------------------------------------
    else if(buttonMode == 3)
    {
      digitalWrite(buttonGREENled,LOW); //Turn GREEN led off
      digitalWrite(buttonREDled,LOW);   //Turn RED led off
      waitBOTHbuttons();                //Wait for the buttons to be released

      troubleshootSensors();            //Enter troubleshooting mode
      
      aimLasers();                      //Aim the lasers again
      startCalibration();               //Start the calibration again
    }

    //Check the sensors
    //------------------
    checkSensorsMAX();                  //Check the max sensor values
    animateBlockLasers(80);             //Animate the lasers being blocked
    
  } while(sensor1 > threshold1 && sensor2 > threshold2);


  //Measure LOW level light intensity and calculate the thresholds
  //-------------------------------------------------------------------
  finishCalibration();
  
  //Wait for the sensors to be unblocked
  //-------------------------------------
  do {
    checkSensors();
  } while(sensor1 < threshold1 && sensor2 < threshold2);
}


// This function executes the first steps of calibration
void startCalibration()
{
  checkSensorsMIN();                //Check the min sensor values

  sensor1HIGH = sensor1;            //Copy sensor 1 to sensor1HIGH
  sensor2HIGH = sensor2;            //Copy sensor 2 to sensor2HIGH
  threshold1 = sensor1HIGH - 150;   //Temporarily set the threshold
  threshold2 = sensor2HIGH - 150;   //Temporarily set the threshold

  digitalWrite(buttonREDled,HIGH);  //Turn on RED LED (cue for user to block beams)
}


// This function executes the last steps of calibration
void finishCalibration()
{
  //Display the lasers being blocked
  //------------------------------------
  displayLasersBlocked();
  delay(500);
  
  //Check the sensors
  //------------------------------------
  checkSensors();                             //Check the mean sensor values
  sensor1LOW = sensor1;                       //Copy sensor1 to sensor1LOW
  sensor2LOW = sensor2;                       //Copy sensor2 to sensor2LOW

  //Calculate thresholds
  //------------------------------------
  threshold1 = mean(sensor1HIGH,sensor1LOW);  //Average of HIGH and LOW
  threshold2 = mean(sensor2HIGH,sensor2LOW);  //Average of HIGH and LOW

  //Blink the GREEN LED (cue for user to remove their hand)
  //------------------------------------
  digitalWrite(buttonREDled,LOW);
  digitalWrite(buttonGREENled,HIGH);
  delay(500);
  digitalWrite(buttonGREENled,LOW);

  clearDisplay();
}



// ------------------------------------------------------------
// Troubleshooting Functions
// ------------------------------------------------------------

// This function puts the device in troubleshooting mode
void troubleshootSensors()
{
  sendSerialGreeting();                     //Notify the computer that the device is ready to send troubleshoot data
  
  troubleshoot = true;                      //Enable troubleshooting mode
  counter = -2;                             //Activation code for troubleshoot data
  digitalWrite(buttonGREENled,HIGH);        //Turn the GREEN LED on
  digitalWrite(buttonREDled,HIGH);          //Turn the RED LED on

  //Notify the computer that the device is in troubleshooting mode
  checkSensors();
  for(int k = 0;k < 3;k++)
  {
    sendSerialData();
    delay(5);
  }

  bool toggleLasers = true;
  
  //Troubleshooting Loop
  do {

    //Check sensors and send data to the computer
    sendTroubleshootingData(100);

    //Check the buttons
    buttonMode = checkButtons();

    //Return to the program if the GREEN button is pressed
    if(buttonMode == 1)
    {                                 
      troubleshoot = false;             //Disable troubleshooting mode

      //Notify computer that troubleshooting mode is disabled
      sendSerialFarewell();

      digitalWrite(buttonGREENled,LOW); //Turn GREEN LED off
      waitGREENbutton();                //Wait for the GREEN button to be released
    }
    //Toggle the lasers on/off if the RED button is pressed
    else if(buttonMode == 2)
    {
      toggleLasers = !toggleLasers;
      digitalWrite(laser1,toggleLasers);
      digitalWrite(laser2,toggleLasers);

      digitalWrite(buttonREDled,LOW);   //Turn RED LED off
    }
  }while(troubleshoot == true);
}

//This function animates the display and sends a data stream to the computer.
//The data is sent 12 times during the function, between steps in the animation.
void sendTroubleshootingData(int d)
{
  //Bitmasks
  uint8_t bitmask = B01000000; //g
  uint8_t clearDigit = B00000000;

  digitalWrite(buttonREDled,LOW);
  checkSensors();
  sendSerialData();
  matrix.writeDigitRaw(4,bitmask);
  matrix.writeDisplay();
  delay(d/2);

  checkSensors();
  sendSerialData();
  delay(d/2);

  checkSensors();
  sendSerialData();
  matrix.writeDigitRaw(3,bitmask);
  matrix.writeDisplay();
  delay(d/2);

  checkSensors();
  sendSerialData();
  delay(d/2);

  digitalWrite(buttonREDled,HIGH);
  checkSensors();
  sendSerialData();
  matrix.writeDigitRaw(1,bitmask);
  matrix.writeDisplay();
  delay(d/2);

  checkSensors();
  sendSerialData();
  delay(d/2);

  checkSensors();
  sendSerialData();
  matrix.writeDigitRaw(0,bitmask);
  matrix.writeDisplay();
  delay(d/2);

  checkSensors();
  sendSerialData();
  delay(d/2);

  digitalWrite(buttonREDled,LOW);
  checkSensors();
  sendSerialData();
  clearDisplay();
  delay(d/2);

  checkSensors();
  sendSerialData();
  delay(d/2);

  checkSensors();
  sendSerialData();
  delay(d/2);

  checkSensors();
  sendSerialData();
  delay(d/2);
  digitalWrite(buttonREDled,HIGH);
}



// ------------------------------------------------------------
// Sensor Functions
// ------------------------------------------------------------

// This function checks the average value of each sensor
void checkSensors()
{
  LDR1a = analogRead(LDRpin1a); //Measure LDR 1a
  LDR1b = analogRead(LDRpin1b); //Measure LDR 1b
  sensor1 = mean(LDR1a,LDR1b);  //Take the average value
  
  LDR2a = analogRead(LDRpin2a); //Measure LDR 2a
  LDR2b = analogRead(LDRpin2b); //Measure LDR 2b
  sensor2 = mean(LDR2a,LDR2b);  //Take the average value
}

// This function checks the min value of each sensor
void checkSensorsMIN()
{
  LDR1a = analogRead(LDRpin1a); //Measure LDR 1a
  LDR1b = analogRead(LDRpin1b); //Measure LDR 1b
  sensor1 = min(LDR1a,LDR1b);   //Take the minimum value
  
  LDR2a = analogRead(LDRpin2a); //Measure LDR 2a
  LDR2b = analogRead(LDRpin2b); //Measure LDR 2b
  sensor2 = min(LDR2a,LDR2b);   //Take the minimum value
}

// This function checks the max value of each sensor
void checkSensorsMAX()
{
  LDR1a = analogRead(LDRpin1a); //Measure LDR 1a
  LDR1b = analogRead(LDRpin1b); //Measure LDR 1b
  sensor1 = max(LDR1a,LDR1b);   //Take the maximum value

  LDR2a = analogRead(LDRpin2a); //Measure LDR 2a
  LDR2b = analogRead(LDRpin2b); //Measure LDR 2b
  sensor2 = max(LDR2a,LDR2b);   //Take the maximum value
}



// ------------------------------------------------------------
// Button Functions
// ------------------------------------------------------------

// This function reads the buttons and checks which buttons were pressed.
// 0 = NEITHER pressed
// 1 = GREEN   pressed
// 2 = RED     pressed
// 3 = BOTH    pressed
int checkButtons()
{
  int bMode = 0; //The button mode to return
  
  // Read the button states
  // ------------------------------------------------
  buttonValueG = digitalRead(buttonGREEN);
  buttonValueR = digitalRead(buttonRED);
  
  // Check which buttons were pushed
  // ------------------------------------------------
  if(buttonValueG)                          //GREEN button pressed
  {
    delay(button2);                         //Wait to see if the other button is pressed
    digitalWrite(buttonGREENled,HIGH);      //Turn the GREEN LED on
    buttonValueR = digitalRead(buttonRED);  //Read the RED button
    //Check the RED button
    if(buttonValueR)                        //RED button also pressed
    {
      digitalWrite(buttonREDled,HIGH);      //Turn the RED LED on
      bMode = 3;                            //Set the button mode to 3 (BOTH buttons pressed)
    }
    else                                    //Only GREEN button pressed
    {
      bMode = 1;                            //Set the button mode to 1 (GREEN button pressed)
    }
  }
  
  else if(buttonValueR)                     //RED button pressed
  {
    delay(button2);                         //Wait to see if the other button is pressed
    digitalWrite(buttonREDled,HIGH);        //Turn the RED LED on
    buttonValueG = digitalRead(buttonGREEN);//Read the GREEN button
    if(buttonValueG)                        //GREEN button also pressed
    {
      digitalWrite(buttonGREENled,HIGH);    //Turn GREEN LED on
      bMode = 3;                            //Set the button mode to 3 (BOTH buttons pressed)
    }
    else                                    //Only RED button pressed
    {
      bMode = 2;                            //Set the button mode to 2 (RED button pressed)
    }
  }

  return bMode;
}

// This function waits for the GREEN button to be released
void waitGREENbutton()
{
  do {
    buttonValueG = digitalRead(buttonGREEN);
  }while(buttonValueG);
}

// This function waits for the RED button to be released
void waitREDbutton()
{
  do {
    buttonValueR = digitalRead(buttonRED);
  }while(buttonValueR);
}

// This function waits for BOTH buttons to be released
void waitBOTHbuttons()
{
  do {
    buttonValueG = digitalRead(buttonGREEN);
    buttonValueR = digitalRead(buttonRED);
  }while(buttonValueG || buttonValueR);
}



// ------------------------------------------------------------
// Other Functions
// ------------------------------------------------------------

//This function sends the data to the serial port
void sendSerialData()
{
   Serial.begin(9600); //Start serial communication (BAUD rate 9600)
  
   String strNum1 = String(sensor1);
   String strNum2 = String(sensor2);
   String strNum3 = String(threshold1);
   String strNum4 = String(threshold2);
   String strNum5 = String(counter);
   String strNum6 = String(LDR1a);
   String strNum7 = String(LDR1b);
   String strNum8 = String(LDR2a);
   String strNum9 = String(LDR2b);
   String str = strNum1 + ',' + strNum2 + ',' + strNum3 + ',' 
              + strNum4 + ',' + strNum5 + ',' + strNum6 + ','
              + strNum7 + ',' + strNum8 + ',' + strNum9 + '\n';
   Serial.print(str);

   Serial.end();      //End serial communication
}

//This function sends a greeting to the computer to prepare it for troubleshoot data
void sendSerialGreeting()
{
  counter = -1;                           //Send the activation code
  for(int k = 0;k < 5;k++)
  {
    sendSerialData();
    delay(5);
  }
  counter = 0;
}

//This function send a farewell to the computer to end troubleshoot mode
void sendSerialFarewell()
{
  counter = -3;
  for(int k = 0;k < 3;k++)
  {
    sendSerialData();
    delay(5);
  }
  counter = 0;                      //Set the counter back to zero
}

// This function calculates the average of two numbers
int mean(int num1,int num2)
{
  return (num1+num2)/2;
}



// ------------------------------------------------------------
// Animation Functions
// ------------------------------------------------------------


//This function makes an animation on 
//the display that goes around in a circle
void animateLoading(int d)
{
  //Bitmasks
  uint8_t intro_1 = B00000001; //a
  uint8_t intro_2 = B00000011; //ab
  uint8_t intro_3 = B00000110; //bc
  uint8_t intro_4 = B00001100; //cd
  uint8_t intro_5 = B00001000; //d
  uint8_t intro_6 = B00011000; //de
  uint8_t intro_7 = B00110000; //ef
  uint8_t intro_8 = B00100000; //f

  uint8_t clearDigit = B00000000;
  
  matrix.writeDigitRaw(0,intro_1);    //1
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(1,intro_1);    //2
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(0,clearDigit); //3
  matrix.writeDigitRaw(3,intro_1);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(1,clearDigit); //4
  matrix.writeDigitRaw(4,intro_1);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(3,clearDigit); //5
  matrix.writeDigitRaw(4,intro_2);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(4,intro_3);    //6
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(4,intro_4);    //7
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(4,intro_5);    //8
  matrix.writeDigitRaw(3,intro_5);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(4,clearDigit); //9
  matrix.writeDigitRaw(1,intro_5);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(3,clearDigit); //10
  matrix.writeDigitRaw(0,intro_5);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(1,clearDigit); //11
  matrix.writeDigitRaw(0,intro_6);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(0,intro_7);    //12
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(0,intro_8);    //13
  matrix.writeDisplay();
  delay(d);
  clearDisplay();
}

//This function displays setup (SEt-UP) 
//until the green button is pressed
void animateSetup()
{
  //Bitmasks for SEtUP (setup)
  uint8_t bitmask_S = B01101101;
  uint8_t bitmask_E = B01111001;
  uint8_t bitmask_t = B01111000;
  uint8_t bitmask_U = B00111110;
  uint8_t bitmask_P = B01110011;

  int buttonVal = 0;
  
  digitalWrite(buttonGREENled,HIGH);
  matrix.writeDigitRaw(1,bitmask_S);
  matrix.writeDisplay();
  delay(50);
  matrix.writeDigitRaw(3,bitmask_E);
  matrix.writeDisplay();
  delay(50);
  matrix.writeDigitRaw(4,bitmask_t);
  matrix.writeDisplay();
  delay(800);

  buttonVal = digitalRead(buttonGREEN); //Read button
  if(!buttonVal)
  {
    clearDisplay();
    digitalWrite(buttonGREENled,LOW);
    matrix.writeDigitRaw(1,bitmask_U);
    matrix.writeDigitRaw(3,bitmask_P);
    matrix.writeDisplay();
    delay(800);
    clearDisplay();
  }

  buttonVal = digitalRead(buttonGREEN); //Check button
  while(!buttonVal)
  {
    digitalWrite(buttonGREENled,HIGH);
    matrix.writeDigitRaw(1,bitmask_S);
    matrix.writeDigitRaw(3,bitmask_E);
    matrix.writeDigitRaw(4,bitmask_t);
    matrix.writeDisplay();
    delay(800);

    buttonVal = digitalRead(buttonGREEN); //Read button
    if(!buttonVal)
    {
      clearDisplay();
      digitalWrite(buttonGREENled,LOW);
      matrix.writeDigitRaw(1,bitmask_U);
      matrix.writeDigitRaw(3,bitmask_P);
      matrix.writeDisplay();
      delay(800);
      clearDisplay();
    }

    buttonVal = digitalRead(buttonGREEN); //Check button
  }
  
  clearDisplay();

  //Wait until the GREEN button is released
  do {
    buttonVal = digitalRead(buttonGREEN);
  } while(buttonVal);
}

//This function "opens the curtains" to the device setup
void animateCurtains(int d)
{
  //Bitmasks
  uint8_t intro_1 = B00000110; //bc
  uint8_t intro_2 = B00110000; //ef

  matrix.writeDigitRaw(1,intro_1);
  matrix.writeDigitRaw(3,intro_2);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(1,intro_2);
  matrix.writeDigitRaw(3,intro_1);
  matrix.writeDisplay();
  delay(d);
  clearDisplay();
  matrix.writeDigitRaw(0,intro_1);
  matrix.writeDigitRaw(4,intro_2);
  matrix.writeDisplay();
  delay(d);
  clearDisplay();
}

//This function animates the direction to enter a room
void animateDoorway(int dirCode,int d)
{
  //Bitmasks
  uint8_t sideL1 = B01000010; //bg
  uint8_t sideR1 = B01100000; //fg
  uint8_t sideL0 = B01000100; //cg
  uint8_t sideR0 = B01010000; //eg
  uint8_t top    = B00000001; //a
  uint8_t middle = B01000000; //g
  uint8_t bottom = B00001000; //d
  uint8_t clearDigit = B00000000;
  
  matrix.writeDigitRaw(0,sideL1);
  matrix.writeDigitRaw(4,sideR1);
  matrix.writeDisplay();

  //Default Setup
  if(dirCode == 1)
  {
    matrix.writeDigitRaw(0,sideL1);
    matrix.writeDigitRaw(4,sideR1);
    matrix.writeDisplay();
    
    matrix.writeDigitRaw(1,top);
    matrix.writeDigitRaw(3,top);
    matrix.writeDisplay();
    delay(d);
    matrix.writeDigitRaw(1,middle);
    matrix.writeDigitRaw(3,middle);
    matrix.writeDisplay();
    delay(d);
    matrix.writeDigitRaw(1,bottom);
    matrix.writeDigitRaw(3,bottom);
    matrix.writeDisplay();
    delay(d);
  }
  //Alternate Setup
  else if(dirCode == 0)
  {
    matrix.writeDigitRaw(0,sideL0);
    matrix.writeDigitRaw(4,sideR0);
    matrix.writeDisplay();
    
    matrix.writeDigitRaw(1,bottom);
    matrix.writeDigitRaw(3,bottom);
    matrix.writeDisplay();
    delay(d);
    matrix.writeDigitRaw(1,middle);
    matrix.writeDigitRaw(3,middle);
    matrix.writeDisplay();
    delay(d);
    matrix.writeDigitRaw(1,top);
    matrix.writeDigitRaw(3,top);
    matrix.writeDisplay();
    delay(d);
  }

  matrix.writeDigitRaw(1,clearDigit);
  matrix.writeDigitRaw(3,clearDigit);
  matrix.writeDisplay();
}

//This function animates the lasers shining
void animateLasers(int d)
{
  //Bitmasks
  uint8_t bitmask = B00001001; //ad
  uint8_t clearDigit = B00000000;

  matrix.writeDigitRaw(0,bitmask);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(1,bitmask);
  matrix.writeDigitRaw(0,clearDigit);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(3,bitmask);
  matrix.writeDigitRaw(1,clearDigit);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(4,bitmask);
  matrix.writeDigitRaw(3,clearDigit);
  matrix.writeDisplay();
  delay(d);

  clearDisplay();
  delay(2*d);
}

//This function displays the lasers on
void displayLasersON()
{
  //Bitmasks
  uint8_t bitmask = B00001001; //ad

  matrix.writeDigitRaw(0,bitmask);
  matrix.writeDigitRaw(1,bitmask);
  matrix.writeDigitRaw(3,bitmask);
  matrix.writeDigitRaw(4,bitmask);
  matrix.writeDisplay();
}

//This function animates the lasers being blocked
void animateBlockLasers(int d)
{
  //Bitmasks
  uint8_t bitmask_lasers = B00001001; //ad
  uint8_t bitmask_block  = B00001111; //abcd
  uint8_t clearDigit = B00000000;

  matrix.writeDigitRaw(0,bitmask_lasers);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(1,bitmask_lasers);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(3,bitmask_lasers);
  matrix.writeDisplay();
  delay(d);
  matrix.writeDigitRaw(3,bitmask_block);
  matrix.writeDisplay();
  delay(2.5*d);
  
  clearDisplay();
  delay(d);
}

//This function displays the lasers already blocked
void displayLasersBlocked()
{
  //Bitmasks
  uint8_t bitmask_lasers = B00001001; //ad
  uint8_t bitmask_block  = B00001111; //abcd
  uint8_t clearDigit = B00000000;

  matrix.writeDigitRaw(0,bitmask_lasers);
  matrix.writeDigitRaw(1,bitmask_lasers);
  matrix.writeDigitRaw(3,bitmask_block);
  matrix.writeDisplay();
}

//This function clears everything from the display
void clearDisplay()
{
  matrix.writeDigitRaw(0,0);
  matrix.writeDigitRaw(1,0);
  matrix.writeDigitRaw(3,0);
  matrix.writeDigitRaw(4,0);
  matrix.drawColon(false);

  matrix.writeDisplay();
}
