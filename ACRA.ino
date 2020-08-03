/// CODE FOR AUTOMATED COLLABORATIVE ROBOTIC ARM(A.C.R.A) ///

#include<Wire.h> 
#include <EEPROM.h>
#include<CapacitiveSensor.h>
#include <Adafruit_PWMServoDriver.h>

#define SAMPLE_DELAY 25 // in ms, 50ms seems good

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Capsense touch sensors. There are three of these currently placed on ACRA.
// You can have as many as you want, but make sure that there enough digital ports on the microcontroller.
CapacitiveSensor touch = CapacitiveSensor(4,6);      
CapacitiveSensor touch1 = CapacitiveSensor(4,5);     
CapacitiveSensor touch2 = CapacitiveSensor(4,7);

// Assigning three buttons(Record, Play, Resume) or toggle switches to digital inputs 8,9,10 on Mega2560
uint8_t recordButtonPin = 8;   
uint8_t playButtonPin = 9;
uint8_t resumeButtonPin= 10;

// Assigning potentiometer feedbacks to the analog inputs
uint8_t feedbackPin = A0;       
uint8_t feedbackPin1 = A1;
uint8_t feedbackPin2 = A2;
uint8_t feedbackPin3 = A3;
uint8_t feedbackPin4 = A4;

uint8_t ledPin = 13;

long val,val1,val2;
uint8_t pos=0;

  
  
void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  pinMode(recordButtonPin, INPUT);
  digitalWrite(recordButtonPin, HIGH); // Done to avoid any pullup resistors. Credits to Adafruit.
  pinMode(playButtonPin, INPUT);
  digitalWrite(playButtonPin, HIGH);
  pinMode(resumeButtonPin,INPUT);
  digitalWrite(resumeButtonPin,HIGH);
  pinMode(ledPin, OUTPUT);
  
  Serial.println("Initialization successful!!! You can now record or playback ACRA's movements");
}



void loop() 
{
 if (! digitalRead( recordButtonPin)) {
   // Wait for release, to reduce switch bouncing
   delay(10);                                 
   while (! digitalRead(recordButtonPin));
   // Button pressed
   delay(20);                                
  recordServo(feedbackPin,feedbackPin1,feedbackPin2,feedbackPin3,feedbackPin4, recordButtonPin);
  }
 
  if (! digitalRead(playButtonPin)) {
   // Wait for release, to reduce switch bouncing 
   delay(10);                                
   while (! digitalRead(playButtonPin));
   // Button pressed
   delay(20);                                
  playServo(playButtonPin);
 }
     // Moving ACRA through potentiometers
     moveWaist(A0,0);      
     moveShoulder(A1,2);
     moveElbow(A2,4);
     moveWrist(A3,6);
     moveGripper(A4,8);
}


///////  PLAYBACK FUNCTION TO PLAY ACRA's MOVEMENTS FROM EEPROM  /////// 
void playServo( uint8_t buttonPin) {

  // Allocating separate addresses for each joint movement 
  uint16_t waist_addr = 0;        
  uint16_t shoulder_addr = 400;
  uint16_t elbow_addr = 800;
  uint16_t wrist_addr = 1200;
  uint16_t gripper_addr = 1600;
  
  Serial.println("Playing ACRA movements from earlier recording");

  while (digitalRead(buttonPin)) {    
    uint16_t waist = EEPROM.read(waist_addr);
    uint16_t shoulder = EEPROM.read(shoulder_addr);
    uint16_t elbow = EEPROM.read(elbow_addr);
    uint16_t wrist = EEPROM.read(wrist_addr);
    uint16_t gripper = EEPROM.read(gripper_addr);   

  Serial.print(waist);Serial.print(" ");Serial.print("waist_addr: ");Serial.println(waist_addr);
  Serial.print(shoulder);Serial.print(" ");Serial.print("shoulder_addr: ");Serial.println(shoulder_addr);
  Serial.print(elbow);Serial.print(" ");Serial.print("elbow_addr: ");Serial.println(elbow_addr);
  Serial.print(wrist);Serial.print(" ");Serial.print("wrist_addr: ");Serial.println(wrist_addr);
  Serial.print(gripper);Serial.print(" ");Serial.print("gripper_addr: ");Serial.println(gripper_addr);
  Serial.println();

    // If flag(last bit of 8-bit address) is detected then break from loop
    if (waist== 255||shoulder == 255||elbow == 255||wrist == 255||gripper == 255) 
    break; 
    
    // Values mapped from 8-bit size to appropriate servo values
    waist=map(waist,0, 254, 160, 470);            
    shoulder = map(shoulder, 0, 254, 325, 425);
    elbow = map(elbow, 0, 254, 325, 452);
    wrist = map(wrist, 0, 254, 300, 470);
    gripper = map(gripper, 0, 254, 240, 300);

    // Check if touch sensors have been activated
    stopServo();  
  
    // Moving servos with stored EEPROM values
    pwm.setPWM(0,0,waist);     
    pwm.setPWM(2,0,shoulder);
    pwm.setPWM(4,0,elbow);
    pwm.setPWM(6,0,wrist);
    pwm.setPWM(8,0,gripper); 
      
    delay(SAMPLE_DELAY);
    
    // Moving onto the next memory address
    waist_addr++;   
    shoulder_addr++;
    elbow_addr++;
    wrist_addr++;
    gripper_addr++;
    
    // If end of each address is reached then break out of loop
    if (waist_addr == 398||shoulder_addr == 798||elbow_addr==1198||wrist_addr == 1598||gripper_addr == 1998) 
    break; 
  }
  Serial.println("Done");
  delay(250);  
}


///////  RECORD FUNCTION TO RECORD ACRA's MOVEMENTS AND STORE INSIDE EEPROM  ///////
void recordServo(uint8_t analogPin,uint8_t analogPin1,uint8_t analogPin2,uint8_t analogPin3,uint8_t analogPin4, uint8_t buttonPin){

  // Allocating separate addresses for each joint movement
  uint16_t waist_addr = 0;               
  uint16_t shoulder_addr = 400;
  uint16_t elbow_addr =800 ;
  uint16_t wrist_addr = 1200;
  uint16_t gripper_addr=1600;
 
  Serial.println("Recording");
  digitalWrite(ledPin, HIGH);
  
  pinMode(analogPin, INPUT);
  pinMode(analogPin1, INPUT);
  pinMode(analogPin2, INPUT);
  pinMode(analogPin3, INPUT);
   
  while (digitalRead(buttonPin)) {
     // Reading potentiometer values of each servo
     uint16_t waist = analogRead(analogPin);        
     uint16_t shoulder = analogRead(analogPin1);
     uint16_t elbow = analogRead(analogPin2);
     uint16_t wrist = analogRead(analogPin3);
     uint16_t gripper = analogRead(analogPin4); 
    
     // Moving servos
     moveWaist(A0,0);  
     moveShoulder(A1,2);
     moveElbow(A2,4);
     moveWrist(A3,6);
     moveGripper(A4,8);
    
     delay(25);
     Serial.print("Read analog: ");
     
     // Mapping values to fit 8-bit size of registers
     waist = map(waist, 0 , 1023, 0, 254);     
     shoulder= map(shoulder,0 , 1023, 0, 254);
     elbow= map(elbow,0 , 1023, 0, 254);
     wrist= map(wrist,0 , 1023, 0, 254);
     gripper= map(gripper,0 , 1023, 0, 254);

     Serial.print(waist);Serial.print(" ");Serial.print("waist_addr: ");Serial.println(waist_addr);
     Serial.print(shoulder);Serial.print(" ");Serial.print("shoulder_addr: ");Serial.println(shoulder_addr);
     Serial.print(elbow);Serial.print(" ");Serial.print("elbow_addr: ");Serial.println(elbow_addr);
     Serial.print(wrist);Serial.print(" ");Serial.print("wrist_addr: ");Serial.println(wrist_addr);
     Serial.print(gripper);Serial.print(" ");Serial.print("gripper_addr: ");Serial.println(gripper_addr);
     Serial.println();
     
     // Storing values into EEPROM
     EEPROM.write(waist_addr, waist);       
     EEPROM.write(shoulder_addr, shoulder);
     EEPROM.write(elbow_addr, elbow);
     EEPROM.write(wrist_addr, wrist); 
     EEPROM.write(gripper_addr, gripper);
     
     // Moving onto the next memory address
     waist_addr++;  
     shoulder_addr++;
     elbow_addr++;
     wrist_addr++;
     gripper_addr++;

     // If the end of memory address is reached then break out of loop 
     if (waist_addr == 398||shoulder_addr == 798||elbow_addr == 1198||wrist_addr == 1598||gripper_addr == 1998) 
     break;
     delay(SAMPLE_DELAY);
  }

  // Assigning the last bit of the register as a flag,to reduce the amount of memory used if the recording has finished. 
  if (waist_addr != 398) EEPROM.write(waist_addr, 255);   
  if (shoulder_addr != 798) EEPROM.write(shoulder_addr, 255);
  if (elbow_addr != 1198) EEPROM.write(elbow_addr, 255);
  if (wrist_addr != 1598) EEPROM.write(wrist_addr, 255);
  if (gripper_addr != 1998) EEPROM.write(gripper_addr, 255);

  digitalWrite(ledPin, LOW);

  Serial.println("Done");
  delay(250);
}


///////  FUNCTION TO CHECK STATUS OF TOUCH SENSORS  /////// 
void stopServo()
{
   val=0;val1=0;val2=0; 

   // Reading values of touch sensors
   val= touch.capacitiveSensor(30);                 
   val1= touch1.capacitiveSensor(30);
   val2= touch2.capacitiveSensor(30);
   
   Serial.print("value: ");Serial.print(val);Serial.print("\t");Serial.print(val1);Serial.print("\t");Serial.println(val2);

   if(val>=150 ||val1>=150 ||val2>=150 && pos==0)
    {
    // Serial.print("Entered if statement");        // Print statement for debugging purposes only
     digitalWrite(ledPin,HIGH);
     pos=1;
     delay(25);
    }
 
   delay(10);

   // Stay in this loop until resume button is pressed
   while(pos)
    {
    // Serial.println("In while loop");             // Print statement for debugging purposes only
    if(! digitalRead(resumeButtonPin))
     {
      delay(10); 
      digitalWrite(ledPin,LOW);
      pos=0;
      delay(25);
      break;
     }
    } 
   delay(25);
}


void moveWrist(int controlIn, int motorOut)
{
  int pulse_wide, pulse_width, potVal;
  
  // Read values from potentiometer
  potVal = analogRead(controlIn);
  
  // Convert to pulse width
  pulse_width = map(potVal, 0, 1023, 300, 470);
   
  //Control Motor
  pwm.setPWM(motorOut, 0, pulse_width);
}


void moveWaist(int controlIn, int motorOut)
{
  int pulse_wide, pulse_width, potVal;
  
  // Read values from potentiometer
  potVal = analogRead(controlIn);
  
  // Convert to pulse width
  pulse_width = map(potVal, 0, 1023, 160, 470);
  
  //Control Motor
  pwm.setPWM(motorOut, 0, pulse_width);
}


void moveShoulder(int controlIn, int motorOut)
{
  int pulse_wide, pulse_width, potVal;
  
  // Read values from potentiometer
  potVal = analogRead(controlIn);
  
  // Convert to pulse width
  pulse_width = map(potVal, 0, 1023, 325, 425);
  
  //Control Motor
  pwm.setPWM(motorOut, 0, pulse_width);
}


void moveElbow(int controlIn, int motorOut)
{
  int pulse_wide, pulse_width, potVal;
  
  // Read values from potentiometer
  potVal = analogRead(controlIn);
  
  // Convert to pulse width
  pulse_width = map(potVal, 0, 1023, 325, 452);
  
  //Control Motor
  pwm.setPWM(motorOut, 0, pulse_width);
}


void moveGripper(int controlIn,int motorOut)
{
   
  int pulse_wide, pulse_width, potVal;
  
  // Read values from potentiometer
  potVal = analogRead(controlIn);
  
  // Convert to pulse width
  pulse_width = map(potVal, 0, 1023, 240, 300 );
  
  //Control Motor
  pwm.setPWM(motorOut, 0, pulse_width);
}  
