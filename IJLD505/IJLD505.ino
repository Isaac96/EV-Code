#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <mcp_can.h>
#include <INA226.h>
#include <EEPROMAnything.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FrequencyTimer2.h>
#include "globals.h"
#include "chademo.h"
/*
  Notes on what needs to be done:
  - Timing analysis showed that the USB, CANBUS, and BT routines take up entirely too much time. They can delay processing by
   almost 100ms! Minor change for test.

  - Investigate what changes are necessary to support the Cortex M0 processor in the Arduino Zero

  - Interrupt driven CAN has a tendency to lock up. It has been disabled for now - It locks up even if the JLD is not sending anything
  but it seems to be able to actually send as much as you want. Only interrupt driven reception seems to make things die.

  Note about timing related code: The function millis() returns a 32 bit integer that specifies the # of milliseconds since last start up.
  That's all well and good but 4 billion milliseconds is a little less than 50 days. One might ask "so?" well, this could potentially run
  indefinitely in some vehicles and so would suffer rollover every 50 days. When this happens improper code will become stupid. So, try
  to implement any timing code like this:
  if ((millis() - someTimeStamp) >= SomeInterval) do stuff

  Such a code block will do the proper thing so long as all variables used are unsigned long / uint32_t variables.

*/

//#define DEBUG_TIMING	//if this is defined you'll get time related debugging messages
//#define CHECK_FREE_RAM //if this is defined it does what it says - reports the lowest free RAM found at any point in the sketch

template<class T> inline Print &operator <<(Print &obj, T arg) {
  obj.print(arg);  //Sets up serial streaming Serial<<someshit;
  return obj;
}


OneWire ds(SENSOR);
DallasTemperature sensors(&ds);

//These have been moved to eeprom. After initial compile the values will be read from EEPROM.
//These thus set the default value to write to eeprom upon first start up
#define MAX_CHARGE_V	400
#define MAX_CHARGE_A	101
#define TARGET_CHARGE_V	390
#define MIN_CHARGE_A	60
#define INITIAL_SOC 100
#define CAPACITY 60
#define RESET_V 400
#define MAX_TEMP 50



INA226 ina;
const unsigned long Interval = 50;
unsigned long Time = 0;
unsigned long PreviousMillis = 0;
unsigned long CurrentMillis = 0;
float Voltage = 0;
float Current = 0;
float Power = 0;
int Count = 0;
byte Command = 0; // "z" will reset the AmpHours and KiloWattHours counters
volatile uint8_t bStartConversion = 0;
volatile uint8_t bGetTemperature = 0;
volatile uint8_t timerIntCounter = 0;
volatile uint8_t timerFastCounter  = 0;
volatile uint8_t timerChademoCounter = 0;
volatile uint8_t sensorReadPosition = 255;
uint8_t tempSensorCount = 0;
bool tempAlarm = 0;
uint8_t overtempSensor;
int32_t canMsgID = 0;
unsigned char canMsg[8];
unsigned char Flag_Recv = 0;
volatile uint8_t debugTick = 0;

int16_t lowestFreeRAM = 2048;

EESettings settings;
#define EEPROM_VALID	0xDA

int ptrBuffer;
char cmdBuffer[80];
bool handlingEvent;
int state;
enum CONSOLE_STATE
{
  STATE_ROOT_MENU
};

void MCP2515_ISR()
{
  //CAN.handleInt();
  Flag_Recv = 1;
}
 
void timer2Int()
{
  timerFastCounter++;
  timerChademoCounter++;
  if (timerChademoCounter >= 3)
  {
    timerChademoCounter = 0;
    if (chademo.bChademoMode  && chademo.bChademoSendRequests) chademo.bChademoRequest = 1;
  }

  if (timerFastCounter == 8)
  {
    debugTick = 1;
    timerFastCounter = 0;
    timerIntCounter++;
    if (timerIntCounter < 10)
    {
      bGetTemperature = 1;
      sensorReadPosition++;
    }
    if (timerIntCounter == 10)
    {
      bStartConversion = 1;
      sensorReadPosition = 255;
    }
    if (timerIntCounter == 18)
    {
      timerIntCounter = 0;
    }
  }
}

void setup()
{
  //first thing configure the I/O pins and set them to a sane state
  pinMode(IN0, INPUT);
  pinMode(IN1, INPUT_PULLUP);
  pinMode(OUT0, OUTPUT);
  pinMode(OUT1, OUTPUT);
  digitalWrite(OUT0, LOW);
  digitalWrite(OUT1, LOW);
  pinMode(A1, OUTPUT); //KEY - Must be HIGH
  pinMode(A0, INPUT); //STATE
  digitalWrite(A1, HIGH);
  pinMode(3, INPUT_PULLUP); //enable weak pull up on MCP2515 int pin connected to INT1 on MCU

  Serial.begin(115200);

  sensors.begin();
  sensors.setWaitForConversion(false); //we're handling the time delay ourselves so no need to wait when asking for temperatures

  CAN.begin(CAN_500KBPS);
  attachInterrupt(1, MCP2515_ISR, FALLING);     // start interrupt

  ina.begin();//69
  ina.configure(INA226_AVERAGES_16, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);//  ina.configure(INA226_AVERAGES_16, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
  ina.calibrate(0.0000833333, 600);
 
  EEPROM_readAnything(256, settings);
  if (settings.valid != EEPROM_VALID) //not proper version so reset to defaults
  {
    Serial.println(F("Settings invalid, re-reading"));
    settings.valid = EEPROM_VALID;
    settings.ampHours = 0.0;
    settings.kiloWattHours = 0.0;
    settings.currentCalibration = 600.0 / 0.050; //600A 50mv shunt
    settings.voltageCalibration = (100000.0 * 830000.0 / 930000.0 + 1000000.0) / (100275.0 * 830000.0 / 930000.0); // (Voltage Divider with (100k in parallel with 830k) and 1M ) 
    settings.packSizeKWH = 24.0; //just a random guess. Maybe it should default to zero though?
    settings.maxChargeAmperage = MAX_CHARGE_A;
    settings.maxChargeVoltage = MAX_CHARGE_V;
    settings.targetChargeVoltage = TARGET_CHARGE_V;
    settings.minChargeAmperage = MIN_CHARGE_A;
    settings.SOC = INITIAL_SOC;
    settings.capacity = CAPACITY;
    settings.debuggingLevel = 2;
    settings.resetVoltage = RESET_V;
    settings.maxChargeTemp = MAX_TEMP;
    EEPROM_writeAnything(256, settings);

  }

  settings.debuggingLevel = 2; //locked in to max debugging for now.

  //attachInterrupt(0, Save, FALLING);
  FrequencyTimer2::setPeriod(25000); //interrupt every 25ms
  FrequencyTimer2::setOnOverflow(timer2Int);

  if (settings.debuggingLevel > 0)
  {
    Serial.print(F("Found "));
    tempSensorCount = sensors.getDeviceCount();
    Serial.print(tempSensorCount);
    Serial.println(F(" temperature sensors."));
  }

  chademo.setTargetAmperage(settings.maxChargeAmperage);
  chademo.setTargetVoltage(settings.targetChargeVoltage);
}

void loop()
{
  //Serial.println(millis());
  uint8_t pos;
  CurrentMillis = millis();
  uint8_t len;
  CAN_FRAME inFrame;
  float tempReading;

#ifdef DEBUG_TIMING
  if (debugTick == 1)
  {
    debugTick = 0;
    Serial.println(millis());
  }
#endif

  chademo.loop();
  if (CurrentMillis - PreviousMillis >= Interval)
  {
    Time = CurrentMillis - PreviousMillis;
    PreviousMillis = CurrentMillis;

    Count++;
    //Serial.print("Reading ");
    Voltage = ina.readBusVoltage() * settings.voltageCalibration;
    Current = ina.readShuntCurrent() * 4;
    settings.ampHours += Current * (float)Time / 1000.0 / 3600.0;
    Power = Voltage * Current / 1000.0;
    settings.kiloWattHours += Power * (float)Time / 1000.0 / 3600.0;
    settings.SOC = ((settings.capacity - settings.ampHours) / settings.capacity) * 100;
    if (Voltage > settings.resetVoltage) {
      Serial.print(Voltage);
      Serial.print(F("V; Reset = "));
      Serial.println(settings.resetVoltage);
      Serial.println(F("Reset Ah & Wh"));
      settings.ampHours = 0.0;
      settings.kiloWattHours = 0.0;
    }
    //Serial.println("Read info");
    chademo.doProcessing();

    if (Count >= 50)
    {
      Count = 0;
      USB();

      if (!chademo.bChademoMode) //save some processor time by not doing these in chademo mode
      {
        CANBUS();
      }
      else if (settings.debuggingLevel > 0)
      {
        Serial.print(F("Chademo Mode: "));
        Serial.println(chademo.getState());
      }
      Save();
#ifdef CHECK_FREE_RAM
      Serial.print(F("Lowest free RAM: "));
      Serial.println(lowestFreeRAM);
#endif
    }
  }
  //if (CAN.GetRXFrame(inFrame)) {
  if (Flag_Recv || (CAN.checkReceive() == CAN_MSGAVAIL)) {
    Flag_Recv = 0;
    CAN.receiveFrame(inFrame);
    //Serial.print("IN CAN: ");
    //Serial.println(inFrame.id, HEX);
    chademo.handleCANFrame(inFrame);
  }

  //digitalWrite(OUT1, HIGH);

  if (bStartConversion == 1)
  {
    bStartConversion = 0;
    sensors.requestTemperatures();
  }
  if (bGetTemperature)
  {
    bGetTemperature = 0;
    pos = sensorReadPosition;
    if (pos < tempSensorCount)
    {
      sensors.isConnected(pos);
      tempReading = sensors.getTempCByIndex(pos);
      if (tempReading > settings.maxChargeTemp) {
        tempAlarm = true;
        overtempSensor = pos;
        Serial.println(F("Sensor overtemp"));
        Serial.println(pos);
      } else if (pos = overtempSensor) {
        tempAlarm = false;
      }
      if (chademo.bChademoMode && tempAlarm)
      {
        Serial.println(F("Temperature fault! Aborting charge!"));
        chademo.setBattOverTemp();
        chademo.setDelayedState(CEASE_CURRENT, 10);
      }

      /*
        if (settings.debuggingLevel > 0)
        {
      	Serial.print(F("  Temp sensor:"));
      	Serial.print(pos);
      	Serial.print(": ");
      	Serial.print(tempReading);
      	Serial.print("/");
      	Serial.print(sensors.getMinTempC(pos));
      	Serial.print("/");
      	Serial.print(sensors.getMaxTempC(pos));
      	Serial.print("/");
      	Serial.println(sensors.getAvgTempC(pos));
        }*/
    }
  }
  checkRAM();
  //digitalWrite(OUT1, LOW);
}

void serialEvent() {
  int incoming;
  incoming = Serial.read();
  if (incoming == -1) { //false alarm....
    return;
  }

  if (incoming == 10 || incoming == 13) { //command done. Parse it.
    handleConsoleCmd();
    ptrBuffer = 0; //reset line counter once the line has been processed
  } else {
    cmdBuffer[ptrBuffer++] = (unsigned char) incoming;
    if (ptrBuffer > 79)
      ptrBuffer = 79;
  }
}

void handleConsoleCmd() {
  handlingEvent = true;

  if (state == STATE_ROOT_MENU) {
    if (ptrBuffer == 1) { //command is a single ascii character
      handleShortCmd();
    } else { //if cmd over 1 char then assume (for now) that it is a config line
      handleConfigCmd();
    }
  }
  handlingEvent = false;
}
void handleConfigCmd() {
  int i;
  int newValue;

  cmdBuffer[ptrBuffer] = 0; //make sure to null terminate
  String cmdString = String();
  unsigned char whichEntry = '0';
  i = 0;

  while (cmdBuffer[i] != '=' && i < ptrBuffer) {
    cmdString.concat(String(cmdBuffer[i++]));
  }
  i++; //skip the =
  if (i >= ptrBuffer)
  {
    Serial.println(F("Command needs a value..ie AH=30"));
    Serial.println(F(""));
    return; //or, we could use this to display the parameter instead of setting
  }
  // strtol() is able to parse also hex values (e.g. a string "0xCAFE"), useful for enable/disable by device id
  newValue = strtol((char *) (cmdBuffer + i), NULL, 0);

  cmdString.toUpperCase();
  if (cmdString == String("AH")) {
    Serial.print(F("Setting AH to "));
    Serial.println(newValue);
    settings.capacity = newValue;
    Save();
    //set value
  } else if (cmdString == String("RESET")) {
    Serial.print(F("Setting Reset Voltage to "));
    Serial.println(newValue);
    //set value. save eeprom
    settings.resetVoltage = newValue;
    Save();
  } else if (cmdString == String("CHADEMOV")) {
    Serial.print(F("Setting Charge Voltage to "));
    Serial.println(newValue);
    settings.targetChargeVoltage = newValue;
    Save();
    //set value, save eeprom
  }
  else if (cmdString == String("CHADEMOA")) {
    Serial.print(F("Setting Charge Current to "));
    Serial.println(newValue);
    settings.maxChargeAmperage = newValue;
    Save();
    //set value, save eeprom
  }
  else if (cmdString == String("CHADEMOEND")) {
    Serial.print(F("Setting End Current to "));
    Serial.println(newValue);
    settings.minChargeAmperage = newValue;
    Save();
    //set value, save eeprom
  }
  else if (cmdString == String("KWH")) {
    Serial.print(F("Setting Pack Size to "));
    Serial.println(newValue);
    settings.packSizeKWH = newValue;
    Save();
    //set value, save eeprom
  }
  else if (cmdString == String("MAXV")) {
    Serial.print(F("Setting Absolute Maximum Voltage to "));
    Serial.println(newValue);
    settings.maxChargeVoltage = newValue;
    Save();
    //set value, save eeprom
  }
  else if (cmdString == String("MAXTEMP")) {
    Serial.print(F("Setting Maximum Charge Temperature to "));
    Serial.println(newValue);
    settings.maxChargeTemp = newValue;
    Save();
    //set value, save eeprom
  }
}
void handleShortCmd() {

  switch (cmdBuffer[0]) {
    case 'h':
    case '?':
    case 'H':
      printMenu();
      break;
    case 'z':
      Serial.println(F("Reset Ah & Wh"));
      settings.ampHours = 0.0;
      settings.kiloWattHours = 0.0;
      Save();
      break;
    case '+':
      settings.voltageCalibration += 0.004;
      Serial.println (settings.voltageCalibration, 5);
      Save();
      break;
    case '-': //set all outputs high
      settings.voltageCalibration -= 0.004;
      Serial.println (settings.voltageCalibration, 5);
      Save();
      break;

  }
}

void printMenu() {}
void Save()
{
  EEPROM_writeAnything(256, settings);
}

void USB()
{
  Serial.print (F("JLD505: "));
  Serial.print (Voltage, 3);
  Serial.print (F("v "));
  Serial.print (Current, 2);
  Serial.print (F("A "));
  Serial.print (settings.ampHours, 1);
  Serial.print (F("Ah "));
  Serial.print (Power, 1);
  Serial.print (F("kW "));
  Serial.print (settings.kiloWattHours, 1);
  Serial.print (F("kWh "));
  Serial.print (settings.capacity, 1);
  Serial.print (F("Ah total "));
  Serial.print (settings.SOC, 1);
  Serial.println (F("% SOC"));

  if (handlingEvent == false) {
    if (Serial.available()) {
      serialEvent();
    }
  }

  checkRAM();
}


void CANBUS()
{
  CAN_FRAME outFrame;
  outFrame.id = 0x404;
  outFrame.length = 8;
  outFrame.priority = 2;
  outFrame.rtr = 0;
  outFrame.extended = 0;

  uint16_t currINT = abs(Current * 10);
  outFrame.data.byte[0] = highByte((int)(Voltage * 10)); // Voltage High Byte
  outFrame.data.byte[1] = lowByte((int)(Voltage * 10)); // Voltage Low Byte
  outFrame.data.byte[2] = highByte(currINT); // Current High Byte
  outFrame.data.byte[3] = lowByte(currINT); // Current Low Byte
  outFrame.data.byte[4] = highByte((int)(settings.ampHours * 10)); // AmpHours High Byte
  outFrame.data.byte[5] = lowByte((int)(settings.ampHours * 10)); // AmpHours Low Byte
  outFrame.data.byte[6] = settings.capacity; // Not Used
  outFrame.data.byte[7] = settings.SOC; // Not Used
  //CAN.EnqueueTX(outFrame);
  CAN.sendFrame(outFrame);

  outFrame.id = 0x505;
  outFrame.length = 8;
  uint16_t Pwr = abs(Power * 10);
  uint16_t KWH = abs(settings.kiloWattHours * 10);
  outFrame.data.byte[0] = highByte(Pwr); // Power High Byte
  outFrame.data.byte[1] = lowByte(Pwr); // Power Low Byte
  outFrame.data.byte[2] = highByte(KWH); // KiloWattHours High Byte
  outFrame.data.byte[3] = lowByte(KWH); // KiloWattHours Low Byte
  outFrame.data.byte[4] = (sensors.getTempC(0)) + 40;
  outFrame.data.byte[5] = (sensors.getTempC(1)) + 40;
  outFrame.data.byte[6] = (sensors.getTempC(2)) + 40;
  outFrame.data.byte[7] = (sensors.getTempC(3)) + 40;
  //CAN.EnqueueTX(outFrame);
  CAN.sendFrame(outFrame);

  checkRAM();
}

void timestamp()
{
  int milliseconds = (int) (millis() / 1) % 1000 ;
  int seconds = (int) (millis() / 1000) % 60 ;
  int minutes = (int) ((millis() / (1000 * 60)) % 60);
  int hours   = (int) ((millis() / (1000 * 60 * 60)) % 24);

  Serial.print(F(" Time:"));
  Serial.print(hours);
  Serial.print(F(":"));
  Serial.print(minutes);
  Serial.print(F(":")); 
  Serial.print(seconds);
  Serial.print(F("."));
  Serial.println(milliseconds);
}

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void checkRAM()
{
  int freeram = freeRam();
  if (freeram < lowestFreeRAM) lowestFreeRAM = freeram;
}
