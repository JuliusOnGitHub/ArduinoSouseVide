#include <PID_AutoTune_v0.h>

#include <NexaCtrl.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

#define TX_PIN 12
#define RX_PIN 8

#define LED_PIN 13

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

// Your remote id which can be it can be "sniffed" with the 
// reciever example at http://playground.arduino.cc/Code/HomeEasy
//const static unsigned long controller_id = 1234591;
const static unsigned long controller_id = 9741442;
unsigned int device = 14;

NexaCtrl nexaCtrl(TX_PIN, RX_PIN);

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
int SampleTime = 5000;
double Kp = 636.62;
double Ki = 0.68;
double Kd = 149173.65;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID_ATune myAutoTune(&Input, &Output);

boolean autotune = false;
boolean running = false;
boolean cookerOn = false;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
  Setpoint = 56;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, SampleTime);
  myPID.SetSampleTime(SampleTime);
  
  Output = 2500;
  myAutoTune.SetNoiseBand(1);
  myAutoTune.SetControlType(1);
  myAutoTune.SetOutputStep(2500.0);
  myAutoTune.SetLookbackSec(60);
}

void loop()
{
  getHeaterTemperature();
  processCommands();
  processCooker();
}

void processCommands(){
	String content = readAvailableData();
    content.toUpperCase();
    int index = 0;
    String command = getValue(content, ';', index);
    while (command != ""){
      processCommand(command);
      command = getValue(content, ';', ++index);
    }
}

float toDouble(String value)
{
  char buf[20];
  value.toCharArray(buf, sizeof(buf));
  return atof(buf); 
}

String readAvailableData(){
  String  content = "";
  char character;
  while(Serial.available()) {
      character = Serial.read();
      content.concat(character);
  }
  return content;
}

void processCommand(String command)
{
  if(command == "STOP")
  {
    running = false;
  }
  else if(command == "START")
  {
    running = true;
  }
  else if(command.startsWith("SET"))
  {
    String variable = getValue(command, ' ', 1);
    String value = getValue(command, ' ', 2);
    if (variable == "SETPOINT"){
      Setpoint = toDouble(value);
    }
    if (variable == "KP"){
      Kp = toDouble(value);
    }
    if (variable == "KI"){
      Ki = toDouble(value);
    }
    if (variable == "KD"){
      Kd = toDouble(value);
    }
    if (variable == "RUNNING"){
      running = value == "1";
    }
    if (variable == "AUTOTUNE"){
      autotune = value == "1";
    }  
  }
}

void processCooker(){
 if(running){
    if(autotune)
    {
      autoTune();
    }
    else
    {
      pid();
    }
    printValues();
    controlHeater((int)Output);
  }
  else
  {
    printValues();
    if(cookerOn)
    {
      turnOff();
    }
    delay(5000);
  }
}

void pid(){
    myPID.Compute();
}

void autoTune(){
    if(myAutoTune.Runtime())
    {
      autotune = false;    
      Kp = myAutoTune.GetKp();
      Ki = myAutoTune.GetKi();
      Kd = myAutoTune.GetKd();
      myPID.SetTunings(Kp, Ki, Kd);
    }
}

void printValues(){
    Serial.print("$");
    printCsv(Input); 
    printCsv(Output);
    printCsv(Setpoint);
    printCsv(Kp); 
    printCsv(Ki);
    printCsv(Kd);
    Serial.print(autotune);
    Serial.print(";");
    Serial.print(running);
    Serial.println("#");
}

void printCsv(double value){
  Serial.print(value);
  Serial.print(";");
}

void getHeaterTemperature(){
  sensors.requestTemperatures();
  Input = sensors.getTempCByIndex(0);
}

void controlHeater(int timeOn){
  int timeOff = SampleTime - timeOn;

  if(timeOn > 0){
    turnOn();
    delay(timeOn);
  }
  
  if(timeOff > 0){
    turnOff();
    delay(timeOff);
  }
}

void turnOff(){
    nexaCtrl.DeviceOff(controller_id, device);
    digitalWrite(LED_PIN, LOW);        
    cookerOn = false;
}

void turnOn(){
    nexaCtrl.DeviceOn(controller_id, device);
    digitalWrite(LED_PIN, HIGH);        
    cookerOn = true;
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  String result = found>index ? data.substring(strIndex[0], strIndex[1]) : "";
  String endString(separator);
  if(result.endsWith(endString)){
    return result.substring(0, result.length() - 1);
  }

  return result;
}

