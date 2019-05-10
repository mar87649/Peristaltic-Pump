#include <LiquidCrystal.h>
#include <MegunoLink.h>
#include <CommandHandler.h>
#include <TCPCommandHandler.h>
#include <ArduinoTimer.h>
#include <CircularBuffer.h>
#include <EEPROMStore.h>
#include <Filter.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);
ExponentialFilter<float> ADCFilter(100, 0);

#define enA 11
#define in1 12
#define in2 13
#define enB 10
#define in3 9
#define in4 8

//Polynomial Regression variables
  double a = - 98.28571429;
  double b =   28.28571429;
  double c = - 1.428571429;
//Display variables
  double input_voltage = 0.0;
  double flow_rate = 0.0;
//motor control variables
  int state=0;
  const int controlPin =  2;
//Timing Variables
  unsigned long startMillis;
  unsigned long currentMillis = 0;
  unsigned long previousMillis = 0;
  long int1 = 5*1000;
  long int2 = 7*1000;
  long interval = 95*1000;
  int motorstate = 1;
  int motorState1;
  int motorState2;
  int counter = 0;
  long state1 = 0;
  long state2 = 0;
//Conversion & other variables
  float temp=0.0;
  float r1=10000.0;
  float r2=1100.0;
  int analog_value;
  float analogM1=0;
  float analogM2=0;
  float smooth_input;
  const int numReadings = 10;
  int readings[numReadings];      // the readings from the analog input
  int readIndex = 0;              // the index of the current reading
  int total = 0;                  // the running total
  int average = 0;                // the average
/////////////////////////////////////////////////////////////////////////
void setup()
{
//Serial Comm setup
  Serial.begin(9600);
//Pin setup
  pinMode(controlPin, INPUT); //toggle switch
  pinMode(enA, OUTPUT);       //motor pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);       //motor pins
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
// Set initial motor rotation direction
  int motorState1 = LOW;
  int motorState2 = LOW;
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
//Filter setup
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {readings[thisReading] = 0;}
  lcd.init();                      // initialize the lcd 
  lcd.init();
  lcd.backlight();
}
//////////////////////////////////////////////////////////////////////////////////
void loop()
{   
//Speed Control
  SpeedControl(); 
//conversion for filter
  int potValue = analogRead(A0);                    // Read potentiometer value
  int pwmOutput = map(potValue, 0, 1023, 0 , 255);  // Map the potentiometer value from 0 to 255
  analogM1 = analogRead(A1);
  analogM2 = analogRead(A2);
  analog_value = LargestOf(analogM1,analogM2);
  smooth_input = smooth(analog_value);
  ADCFilter.Filter(smooth_input);
  temp = (ADCFilter.Current() * 5.0) / 1024.0; 
  
//Serial monitor info
  Serial.print("v= ");
  Serial.println(input_voltage);
  Serial.print("f= ");
  Serial.println(flow_rate);
  TimePlot Plot;
  Plot.SendData("Raw", smooth_input);
  Plot.SendData("Filtered", ADCFilter.Current());

 
//motor states
  state = digitalRead(controlPin);
  if(state==1){     
    Timing2();   //turn off motors every 90 sec
    if (counter == 90){
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    } else{
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    counter = 0;
    currentMillis = millis(); 
    state1 = currentMillis;
  }



//Timing
//Timing();

//Display
  input_voltage = calcVoltage_pot(pwmOutput);
  flow_rate = calcFlow_pot(input_voltage);
  Display2();
  
delay(50);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float calcVoltage_pot(float pwmOutput){
  input_voltage = RoundTenth(pwmOutput*0.03529);
  return input_voltage;
}

float calcFlow_pot(float input_voltage){
  if (input_voltage < 5){
    flow_rate = 0;
  }else{
      flow_rate = -.1875000009*pow(input_voltage, 4)
                  + 5.995370399*pow(input_voltage, 3)
                  - 71.77083365*pow(input_voltage, 2)
                  + 386.236774*input_voltage
                  - 763.1507962;
  }
  flow_rate = RoundTenth(flow_rate);
  return flow_rate;
}

float RoundTenth(float a){
  a = (a*10) + .5;
  a = float(int(a));
  a = a/10;
  return a;
}

float LargestOf(int a, int b){
  if (a>b){
    return a;
  }else{
    return b;
  }  
}

void SpeedControl(){
  int potValue = analogRead(A0);                    // Read potentiometer value
  int pwmOutput = map(potValue, 0, 1023, 0 , 255);  // Map the potentiometer value from 0 to 255
  analogWrite(enA, pwmOutput);                      // Send PWM signal to L298N Enable pin 
  analogWrite(enB, pwmOutput);                      // Send PWM signal to L298N Enable pin
}

void Timing(){
  currentMillis = millis();  
  
  if(currentMillis - state1 > int1) { //if 90 sec have passed
    state1 = currentMillis; 
    State1();                         //turn motor A off and motor B on
  }
  if(currentMillis - state2 > int2) { //if 2 sec after state
    state1 = state1 + 2000;
    state2 = currentMillis; 
    State2();                         //turn motor B off and motor A on
  }
}


void Timing2(){
  currentMillis = millis();    
  if(currentMillis - state1 > 90000) { //if 90 sec have passed
    state1 = currentMillis; //reset counter
    counter = 90;         
  }
}

void Display(){
  lcd.setCursor(0, 0);
  lcd.print("Flow Rate: ");
  lcd.print(flow_rate);  
  //lcd.print("ml/min ");
  lcd.setCursor(0, 1);
  lcd.print("Voltage: ");
  lcd.print(input_voltage);
}

void Display2(){
  lcd.setCursor(0,0);
  lcd.print("Flow: ");
  lcd.print(flow_rate);  
  lcd.print(" ml/s");
  lcd.setCursor(0,1);
  lcd.print("Volt: ");
  lcd.print(input_voltage);
  lcd.print(" V");

}

void State1(){
    motorState1 = LOW;         // Turn off motor 1
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    motorState2 = HIGH;        // Turn on  motor 2
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

void State2(){
    motorState1 = HIGH;        // Turn on  motor 1
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    motorState2 = LOW;         // Turn off motor 2
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

float smooth(float data) {
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = data;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
  return average;
}
