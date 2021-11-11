// This code was written to implement an inverter control system based on avr achitecture
// has ability to isolate output and shut down the system when a fault such as high temperature, short circuits etc are detected
// do not change parameters youre not told to change.
//other details are in the code.
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 4); // rx and tx respectively for user debug preferrably bluetooth
String readString, DeviceState, BatteryType;
String inverterID = "AVT101";
const float B12vParameters[] = {10.0, 12.7, 240.0, 150.0, 190.0, 239.0, 9}; //{low voltage, full-charge voltage, normal output voltage, short circuit voltage, over load voltage, under load voltage, oscillator enable pin}
const float B24vParameters[] = {22.0, 24.7, 240.0, 150.0, 190.0, 239.0, 7};//{low voltage, full-charge voltage, normal output voltage, short circuit voltage, over load voltage, under load voltage, oscillator enable pin}
const float B36vParameters[] = {34.0, 36.7, 240.0, 150.0, 190.0, 239.0, 12};//{low voltage, full-charge voltage, normal output voltage, short circuit voltage, over load voltage, under load voltage, oscillator enable pin}
const float B48vParameters[] = {46.0, 48.7, 240.0, 150.0, 190.0, 239.0, 6};//{low voltage, full-charge voltage, normal output voltage, short circuit voltage, over load voltage, under load voltage, oscillator enable pin}
const int battery = 4; // Analog pin A4, connects to the voltage divider for measuring battery voltage. should have a value of (19k, 1k) pairs,
                        // change the values using scalar multiplication e.g  2 * (19k, 1k), 3 * (19k, 1k) etc (in each case (38k, 2k) and (57k, 3k));
const int output = 5; // Analog pin A5, connects to the voltage divider for ac voltage after rectification. should have a value of (59k, 1k) pairs,
                        // change the values using scalar multiplication e.g  2 * (59k, 1k), 3 * (59k, 1k) etc (in each case (118k, 2k) and (177k, 3k));
const int temp = 3; // Analog pin A3, connects to a thermistor or infrared sensor near the mosfets or in the inverter;    
const int threshold = 800; // normal temperature; note, this is not in celsius or farenheit. change it to any number between 0 and 1023;                  
const int buzzer = 8; // connects to an active buzzer, a blue led should also be connected to this pin
const int redLed = 13; // connects to an active buzzer, a blue led should also be connected to this pin
const int isolator = 10; // connects to the isolator/relays driver mosfets
const int heatSinkFan = 11; // connects to a heat sink fan driver mosfet
const int normalSpeed = 128; // normal speed of heat sink fan. change this to any number between 0 and 255;
const int highSpeed = 254; // high speed of heat sink fan. change this to any number between 0 and 255;
unsigned long lastTime;
unsigned long resendTime = 3000;
bool isSeen = false;
const float maxBattery = 100.0;
int batteryWarningCount;
int inverterWarningCount;
int maximumCount = 3;
bool fault = false;
int enable;
float chargeVoltage;
float lowVoltage;
float maxVoltage = 300.0;
float normalAcOutputVoltage; 
float shortCircuitVoltage; 
float overLoadVoltage;  
float underLoadVoltage; 

class Beeper
{
  int ledPin;
  long onTime;
  long offTime;

  int ledState;
  unsigned long previousMillis;
  public:
  Beeper(int pin, long on, long off)
  {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);

    onTime = on;
    offTime = off;
    ledState = LOW;
    previousMillis = 0;
  }
  void Update()
  {
    unsigned long currentMillis = millis();
    if((ledState == HIGH) && (currentMillis - previousMillis >= onTime))
    {
      ledState = LOW;
      previousMillis = currentMillis;
      digitalWrite(ledPin, ledState);
    }
    else if ((ledState == LOW) && (currentMillis - previousMillis >= offTime))
    {
      ledState = HIGH;
      previousMillis = currentMillis;
      digitalWrite(ledPin, ledState);
    }
  }
};

void dot(int pin) {
  digitalWrite(pin, HIGH);
  delay(250);
  digitalWrite(pin, LOW);
  delay(250);
}

void dash(int pin) {
  digitalWrite(pin, HIGH);
  delay(1000);
  digitalWrite(pin, LOW);
  delay(250);
}

float fmap(float val, float fromLow, float fromHigh, float toLow, float toHigh)
 {
  float norm = (val - fromLow) / (fromHigh - fromLow);
  float lerp = norm * (toHigh - toLow) + toLow;
  return lerp;
 }
int BatteryPercentage()
 {
  int voltage = measureBatteryVoltage();
  int percentage = fmap(voltage, lowVoltage, chargeVoltage, 0, 100.0);
  if (percentage < 1)
  {
    return 0;
  }
  else {
    return percentage;
  }
 }

 float measureBatteryVoltage() {
  float ADCval = analogRead(battery);
  float Voltage = (ADCval * maxBattery) / 1023.0;
  return Voltage;
 }

 float measureOutputVoltage()
 {
  float ADCval = analogRead(output);
  float Volt = (ADCval * maxVoltage) / 1023.0;
  return Volt;
 }

bool IsUnderVoltage()
{
  int perVal = BatteryPercentage();
  if(perVal <= 18)
  {
    return true;
  }
  else {
    return false;
  }
}

bool IsCritical()
{
  int criticVal = BatteryPercentage();
  if(criticVal < 15)
  {
    return true;
  }
  else {
    return false;
  }
}

bool IsOverVoltage()
{
  float voltVal = measureOutputVoltage();
  if(voltVal > normalAcOutputVoltage)
  {
    return true;
  }
  else {
    return false;
  }
}

bool IsShortCircuit()
{
  float shortVal = measureOutputVoltage();
  if(shortVal <= shortCircuitVoltage)
  {
    return true;
  }
  else {
    return false;
  }
}

bool IsOverLoad()
{
  float overVal = measureOutputVoltage();
  if(overVal < overLoadVoltage)
  {
    return true;
  }
  else {
    return false;
  }
}

bool IsUnderLoad()
{
  float underVal = measureOutputVoltage();
  if(underVal >= underLoadVoltage && underVal <= normalAcOutputVoltage)
  {
    return true;
  }
  else {
    return false;
  }
}
bool temperatureIsHigh()
{
  if(analogRead(temp) > threshold)
  {
    return true;
  }
  else {
    return false;
  }
}

void normalBeep()
{
  dot(buzzer); dot(buzzer); // . .
}

void criticalBeep()
{
  dash(buzzer); dash(buzzer); // _ _
}
void underLoadBeep()
{
  dash(buzzer); dash(buzzer); dot(buzzer); // _ _ .
}
void overLoadBeep()
{
  dot(buzzer); dash(buzzer); dash(buzzer); // . _ _
}
void underVoltageBeep()
{
  dot(buzzer); dash(buzzer); dot(buzzer); dash(buzzer); // . _ . _
}
void overVoltageBeep()
{
  dash(buzzer); dot(buzzer); dash(buzzer); dot(buzzer); // _ . _ .
}
void shortCircuitBeep()
{
  dash(buzzer); dash(buzzer); dash(buzzer); dash(buzzer); dot(buzzer); // _ _ _ _ .
}
void highTemperatureBeep()
{
  dot(buzzer); dot(buzzer); dot(buzzer); dot(buzzer); dash(buzzer); // . . . . _
}

void enableOscillator()
{
  pinMode(enable, OUTPUT);
  digitalWrite(enable, HIGH);
  analogWrite(heatSinkFan, normalSpeed); 
}

void disableOscillator()
{
  pinMode(enable, OUTPUT);
  digitalWrite(enable, LOW);
  analogWrite(heatSinkFan, 0); 
}
void enableOutput()
{
  pinMode(isolator, OUTPUT);
  digitalWrite(isolator, 1);
}
void disableOutput()
{
  pinMode(isolator, OUTPUT);
  digitalWrite(isolator, 0);
}
void Check_and_Set_Battery_Parameters()
{
  if(!Is48vBattery() && !Is36vBattery() && !Is24vBattery() && Is12vBattery())
  {
    mySerial.println(">> 12v battery detected, setting parameters");
    lowVoltage = B12vParameters[0];
    chargeVoltage = B12vParameters[1];
    normalAcOutputVoltage = B12vParameters[2];
    shortCircuitVoltage = B12vParameters[3];
    underLoadVoltage = B12vParameters[4];
    overLoadVoltage = B12vParameters[5];
    enable = int(B12vParameters[6]);
    BatteryType = "12 Volts";
    resetBatteryWarnings();
  }
  else if(!Is48vBattery() && !Is36vBattery() && Is24vBattery() && !Is12vBattery())
  {
    mySerial.println(">> 24v battery detected, setting parameters");
    lowVoltage = B24vParameters[0];
    chargeVoltage = B24vParameters[1];
    normalAcOutputVoltage = B24vParameters[2];
    shortCircuitVoltage = B24vParameters[3];
    underLoadVoltage = B24vParameters[4];
    overLoadVoltage = B24vParameters[5];
    enable = int(B24vParameters[6]);
    BatteryType = "24 Volts";
    resetBatteryWarnings();
  }
  else if(!Is48vBattery() && Is36vBattery() && !Is24vBattery() && !Is12vBattery())
  {
    mySerial.println(">> 36v battery detected, setting parameters");
    lowVoltage = B36vParameters[0];
    chargeVoltage = B36vParameters[1];
    normalAcOutputVoltage = B36vParameters[2];
    shortCircuitVoltage = B36vParameters[3];
    underLoadVoltage = B36vParameters[4];
    overLoadVoltage = B36vParameters[5];
    enable = int(B36vParameters[6]);
    BatteryType = "36 Volts";
    resetBatteryWarnings();
  }
  else if(Is48vBattery() && !Is36vBattery() && !Is24vBattery() && !Is12vBattery())
  {
    mySerial.println(">> 48v battery detected, setting parameters");
    lowVoltage = B48vParameters[0];
    chargeVoltage = B48vParameters[1];
    normalAcOutputVoltage = B48vParameters[2];
    shortCircuitVoltage = B48vParameters[3];
    underLoadVoltage = B48vParameters[4];
    overLoadVoltage = B48vParameters[5];
    enable = int(B48vParameters[6]);
    BatteryType = "48 Volts";
    resetBatteryWarnings();
  }
  else {
    mySerial.println(">> Battery not recognised");
    batteryWarningCount++;
    delay(1000);
    if (batteryWarningCount >= maximumCount)
    {
      batteryWarningCount = 0;
      fault = true;
      mySerial.println(">> Going offline");
      return;
    }
    Check_and_Set_Battery_Parameters();
  }
}
bool Is12vBattery()
{
  float batt = measureBatteryVoltage();
  if (batt >= B12vParameters[0] && batt <= B12vParameters[1])
  {
    return true;
  }
  else {
    return false;
  }
}
bool Is24vBattery()
{
  float batt = measureBatteryVoltage();
  if (batt >= B24vParameters[0] && batt <= B24vParameters[1])
  {
    return true;
  }
  else {
    return false;
  }
}
bool Is36vBattery()
{
  float batt = measureBatteryVoltage();
  if (batt >= B36vParameters[0] && batt <= B36vParameters[1])
  {
    return true;
  }
  else {
    return false;
  }
}
bool Is48vBattery()
{
  float batt = measureBatteryVoltage();
  if (batt >= B48vParameters[0] && batt <= B48vParameters[1])
  {
    return true;
  }
  else {
    return false;
  }
}
void faultCount()
{
  inverterWarningCount++;
  if (inverterWarningCount >= maximumCount)
  {
    resetInverterWarnings();
    fault = true;
  }
}
void resetInverterWarnings()
{
  inverterWarningCount = 0;
  isSeen = false;
}
void resetBatteryWarnings()
{
  batteryWarningCount = 0;
}
void Begin()
{
  if(!IsCritical() && !temperatureIsHigh())
  {
    enableOscillator();
    resetInverterWarnings();
  }
  else if(!IsCritical() && temperatureIsHigh())
  {
    disableOutput();
    disableOscillator();
    analogWrite(heatSinkFan, highSpeed);
    highTemperatureBeep();
    mySerial.println(">> Temperature is high!");
    inverterWarningCount++;
    if (inverterWarningCount >= maximumCount)
    {
      resetInverterWarnings();
      fault = true;
      mySerial.println(">> Going offline");
      DeviceState = "High Temperature";
      return;
    }
    Begin();
  }
  else {
    criticalBeep();
    disableOutput();
    disableOscillator();
    mySerial.println(">> Battery is at critical level!");
    inverterWarningCount++;
    if (inverterWarningCount >= maximumCount)
    {
      resetInverterWarnings();
      fault = true;
      mySerial.println(">> Going offline");
      DeviceState = "Battery is Critical";
      return;
    }
    Begin();
  }
}

void Debug()
{
  while(Serial.available())
{
  readString = Serial.readStringUntil(';');
}
if(readString.length() > 0)
{
  if(readString == "reset" || readString == "Reset" || readString == "RESET")
  {
    Serial.println(">> OK");
    delay(1000);
    resetInverterWarnings();
    fault = false;
    digitalWrite(redLed, 0);
    Serial.println(">> Reset succesfull!");
    delay(1000);
  }
  else if (readString == "log" || readString == "Log" || readString == "LOG")
  {
    Serial.println(">> OK");
    delay(1000);
    Serial.println(">> ******LOG****** <<");
    Serial.println(">> Battery Type: " + BatteryType);
    Serial.println(">> Battery: Voltage = " + String(measureBatteryVoltage()) + "V, " + "Percentage = " + String(BatteryPercentage()) + '%');
    enableOscillator();
    delay(1000);
    Serial.println(">> Output: " + String(measureOutputVoltage()) + "VAC");
    disableOscillator();
    int tem = analogRead(temp);
    if(tem > threshold)
    {
      Serial.println(">> Temperature is above threshold, about " + String(tem) + " / 1023 units");
    }
    else {
      Serial.println(">> Temperature is " + String(tem) + " / 1023 units");
    }
  }
  else if (readString == "state" || readString == "State" || readString == "STATE")
  {
    Serial.println(">> OK");
    delay(1000);
    Serial.println(">> ******STATE****** <<");
    Serial.println(">> " + DeviceState);
  }
  else if (readString == "ok" || readString == "Ok" || readString == "OK")
  {
    isSeen = true;
    Serial.println(">> " + readString);
    Serial.println(">> Thanks!");
  }
  readString = "";
}
}
bool timeElapsed()
{
  unsigned long currentTime = millis();
  if(currentTime - lastTime >= resendTime)
  {
    lastTime = currentTime;
    return true;
  }
  else {
    return false;
  }
}
void faultAlert()
{
  if(!isSeen && timeElapsed())
  {
    Serial.println(inverterID + ": fault mode");
  }
}

Beeper beep(buzzer, 500, 500);
Beeper beeper(buzzer, 200, 5000);
Beeper warningFlash(redLed, 500, 500);
void setup() {
Serial.begin(9600); // Serial for manufacturer/engineer debug
mySerial.begin(9600); // Serial for user debug
mySerial.println(">> hi"); // change hi to your greeting, debugging or initialization message.
delay(1000);
pinMode(buzzer, OUTPUT);
digitalWrite(buzzer, 1);
delay(5000);
digitalWrite(buzzer, 0);
delay(1000);
mySerial.println(">> Checking Battery's Voltage");
Check_and_Set_Battery_Parameters();
while(fault)
{
  faultAlert();
  Debug();
  beep.Update();
  DeviceState = "Battery paramater unknown";
}
mySerial.println(">> Done!");
pinMode(enable, OUTPUT);
pinMode(redLed, OUTPUT);
digitalWrite(enable, LOW);
pinMode(heatSinkFan, OUTPUT);
analogWrite(heatSinkFan, highSpeed);
}

void loop() {
while(!fault)
{
  Begin();
  if(IsUnderVoltage())
  {
    underVoltageBeep();
    disableOutput();
    faultCount();
    DeviceState = "Under Voltage";
    mySerial.println(">> undervoltage!");
  }
  else if(IsOverVoltage())
  {
    overVoltageBeep();
    disableOutput();
    faultCount();
    DeviceState = "Over Voltage";
    mySerial.println(">> overvoltage!");
  }
  else if(IsUnderLoad())
  {
    delay(10000);
    underLoadBeep();
    disableOutput();
    faultCount();
    DeviceState = "Under Load";
    mySerial.println(">> underload!");
  }
  else if(IsOverLoad() && !IsShortCircuit())
  {
    delay(5000);
    overLoadBeep();
    disableOutput();
    faultCount();
    DeviceState = "Over Loaded";
    mySerial.println(">> overload!");
  }
  else if(IsShortCircuit())
  {
    shortCircuitBeep();
    disableOutput();
    faultCount();
    DeviceState = "Short Circuit";
    mySerial.println(">> short circuit!");
    delay(5000);
  }
  else {
    beeper.Update();
    enableOutput(); 
    resetInverterWarnings();
    DeviceState = "Normal";
    mySerial.println(">> working!");
  }
  Debug();
}
disableOutput();
disableOscillator();
faultAlert();
warningFlash.Update();
Debug();
}
 
