#include <FlexCAN.h>

uint32_t cellVoltage[16][12];
uint16_t temp[2][12];
uint8_t dataIn[39];
uint8_t BMSCommand[7] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
uint8_t BMSCommand2[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
uint8_t muxChannel = 0;
uint8_t muxPins[4] = {23, 22, 21, 20};

uint8_t packetNum = 0;
uint8_t moduleOutIndex = 0;

uint8_t moduleStatus[12];
uint16_t moduleMaxVoltage[12];
uint16_t moduleMinVoltage[12];
uint32_t moduleTotalVoltage[12];

uint32_t maxVoltage = 0;
uint32_t minVoltage = 0;

uint32_t aveVoltage = 0;

uint32_t totalVoltage = 0;

uint32_t minVoltageLim = 3300;
uint32_t maxVoltageLim = 4200;
uint32_t balVoltageLim = 4150;
uint32_t tempLimit = 50;

uint32_t interval = 200;
uint32_t prevTime;

uint32_t printInt = 5700;
uint32_t prevPrint;

uint32_t CANInt = 500;
uint32_t prevCAN;

bool bmsConnected[12];
uint8_t moduleCount;

bool HVC;
bool LVC;
bool BVC;
bool TMPC;

/*Module status frames - max and min voltages and total voltage
   Address is 0x700+module ID
   Bytes 0,1,2,3 are MSB and LSB of max and min voltage
   4,5,6 are MSB, middle and LSB of total voltage
   7 is empty for now
*/

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(9600);
  Serial.begin(115200);
  Serial.println("LLTBMS06-Teensy");
  for (int i = 0; i < 4; i++) {
    pinMode(muxPins[i], OUTPUT);
  }
  pinMode(28, OUTPUT);//enable pin; this is specific to the CAN adapter I used
  digitalWrite(28, LOW);//it needs a strong pull to ground to enable transmissions
  Can0.begin(500000, 1, 1);//500K, alternate pins
}

void loop() {
  unsigned long timeNow = millis();
  if (timeNow - prevTime > interval) {
    for (int i = 0; i < 12; i++) {
      setChannel(i);
      sendRecvData();
      //getTempData();
    }
    prevTime = millis();
  }
  timeNow = millis();
  if (timeNow - prevPrint > printInt) {
    Serial.println("MIN  AVE  MAX  TOTAL");
    Serial.print(minVoltage);
    Serial.print(" ");
    Serial.print(aveVoltage);
    Serial.print(" ");
    Serial.print(maxVoltage);
    Serial.print(" ");
    Serial.println(totalVoltage);
    for (int i = 0; i < 12; i++) {
      Serial.print(bmsConnected[i]);
    }
    Serial.println();
    Serial.print(HVC);
    Serial.print(LVC);
    Serial.print(BVC);
    Serial.println(TMPC);
    calcMinMax();
    prevPrint = millis();
  }
  timeNow = millis();
  if (timeNow - prevCAN > CANInt) {
    sendBMSData();
    prevCAN = millis();
  }
}

void sendRecvData() {
  for (int i = 0; i < 7; i++) {
    Serial2.write(BMSCommand[i]);
  }
  Serial2.flush();
  delay(50);
  if (Serial2.peek() == 0xDD) {
    for (int i = 0; i < 39; i++) {
      dataIn[i] = Serial2.read();
      for (int i = 0; i < 16; i++) {
        cellVoltage[i][muxChannel] = (dataIn[(i * 2) + 4] * 256) + (dataIn[(i * 2) + 5]);
      }
    }
  }
  while (Serial2.available()) {
    Serial2.read();
  }
}

void getTempData() {
  for (int i = 0; i < 7; i++) {
    Serial2.write(BMSCommand2[i]);
  }
  Serial2.flush();
  delay(50);
  if (Serial2.peek() == 0xDD) {
    for (int i = 0; i < 34; i++) {
      dataIn[i] = Serial2.read();
      //Serial.print(" 0x");
      //Serial.print(dataIn[i], HEX);
    }
    //Serial.println();
  }
  temp[0][muxChannel] = ((dataIn[27] * 256) + dataIn[28] - 2731) / 10;
  temp[1][muxChannel] = ((dataIn[29] * 256) + dataIn[30] - 2731) / 10;
  while (Serial2.available()) {
    Serial2.read();
  }
}

void setChannel(uint8_t chan) {
  muxChannel = chan;
  for (int i = 0; i < 4; i++) {
    digitalWrite(muxPins[i], bitRead(chan, 3 - i));
  }
}
void calcMinMax() {
  HVC = 0;
  LVC = 0;
  BVC = 0;

  minVoltage = 5000;//impossible value
  maxVoltage = 1000;//impossible value
  totalVoltage = 0;
  aveVoltage = 0;
  moduleCount = 0;
  for (int i = 0; i < 12; i++) {
    bmsConnected[i] = 0;
    for (int j = 0; j < 16; j++) {
      if (cellVoltage[j][i] > 1000) {
        bmsConnected[i] = 1;
        totalVoltage += cellVoltage[j][i];
        if (cellVoltage[j][i] > maxVoltage) {
          maxVoltage = cellVoltage[j][i];
        }
        if (cellVoltage[j][i] < minVoltage) {
          minVoltage = cellVoltage[j][i];
        }
      }
    }
    if (bmsConnected[i]) {
      moduleCount++;
    }
    calcModuleInfo(i);
  }
  aveVoltage = totalVoltage / (moduleCount * 16);//16 cells per module, 2 systems in parallel.
  totalVoltage = totalVoltage / 2;
  if (maxVoltage >= maxVoltageLim) {
    HVC = 1;
  }
  if (minVoltage <= minVoltageLim) {
    LVC = 1;
  }
  if (maxVoltage >= balVoltageLim) {
    BVC = 1;
  }
  TMPC = 0;
  for (int i = 1; i < 12; i++) {
    if ((temp[1][i] >= tempLimit) || (temp[0][i] >= tempLimit)) {
      TMPC = 1;
      Serial.println(temp[1][i]);
      Serial.println(temp[0][i]);
      Serial.println(i);
    }
  }
  for (int i = 0; i < 12; i++) {
    for (int j = 0; j < 16; j++) {
      cellVoltage[j][i] = 0;
    }
  }
}
void sendBMSData() {
  if (moduleCount < 12) {
    TMPC = 1;
  }
  if (packetNum < 6) {
    packetNum++;
    TMPC = 0;
    BVC = 0;
    LVC = 0;
    HVC = 0;
  }
  static CAN_message_t BMSFrame;
  BMSFrame.ext = 0;
  BMSFrame.id = 0x50f;
  BMSFrame.len = 5;
  BMSFrame.buf[0] = 0x00;//status; 1 = hvc, 2 = lvc, 4 = bvc
  BMSFrame.buf[1] = 0x00;//0
  BMSFrame.buf[2] = 0x00;//fault; 4 = overtemp/undertemp
  BMSFrame.buf[3] = 0x00;//0
  BMSFrame.buf[4] = 0x00;//0
  if (HVC) {
    bitSet(BMSFrame.buf[0], 0);
  }
  if (LVC) {
    bitSet(BMSFrame.buf[0], 1);
  }
  if (BVC) {
    bitSet(BMSFrame.buf[0], 2);
  }
  if (TMPC) {
    bitSet(BMSFrame.buf[2], 2);//therm overtemp
  }

  static CAN_message_t EVCCFrame;
  EVCCFrame.ext = 1;
  EVCCFrame.id = 0x01dd0001;
  EVCCFrame.len = 5;
  EVCCFrame.buf[0] = 0x00;//status; 1 = hvc, 2 = lvc, 4 = bvc
  EVCCFrame.buf[1] = 0x00;//0
  EVCCFrame.buf[2] = 0x00;//fault; 4 = overtemp/undertemp
  EVCCFrame.buf[3] = 0x00;//0
  EVCCFrame.buf[4] = 0x00;//0
  if (HVC) {
    bitSet(EVCCFrame.buf[0], 0);
  }
  if (LVC) {
    bitSet(EVCCFrame.buf[0], 1);
  }
  if (BVC) {
    bitSet(EVCCFrame.buf[0], 2);
  }
  if (TMPC) {
    bitSet(EVCCFrame.buf[2], 2);//therm overtemp
  }

  uint32_t roundVolts = totalVoltage / 1000;
  static CAN_message_t infoFrame;
  infoFrame.ext = 0;
  infoFrame.id = 0x50e;
  infoFrame.len = 8;
  infoFrame.buf[0] = 0x00;//status; 1 = hvc, 2 = lvc, 4 = bvc
  infoFrame.buf[1] = 0x00;//fault; 1 = overtemp/undertemp
  infoFrame.buf[2] = lowByte(roundVolts);//voltage LSB
  infoFrame.buf[3] = highByte(roundVolts);//voltage MSB
  infoFrame.buf[4] = lowByte(maxVoltage);
  infoFrame.buf[5] = highByte(maxVoltage);
  infoFrame.buf[6] = lowByte(minVoltage);
  infoFrame.buf[7] = highByte(minVoltage);

  if (HVC) {
    bitSet(infoFrame.buf[0], 0);
  }
  if (LVC) {
    bitSet(infoFrame.buf[0], 1);
  }
  if (BVC) {
    bitSet(infoFrame.buf[0], 2);
  }

  if (TMPC) {
    bitSet(infoFrame.buf[0], 3);//therm overtemp
  }
  Can0.write(BMSFrame);
  delay(20);
  Can0.write(EVCCFrame);
  delay(20);
  Can0.write(infoFrame);
  //send the module frames
  sendModuleInfo(moduleOutIndex);
  moduleOutIndex++;
  sendModuleInfo(moduleOutIndex);
  if (moduleOutIndex == 11) {
    moduleOutIndex = 0;
  }
  else {
    moduleOutIndex++;
  }
}

void calcModuleInfo(int sendIndex) {
  //first, calculate the module information.
  moduleTotalVoltage[sendIndex] = 0;
  moduleMinVoltage[sendIndex] = 5000;
  moduleMaxVoltage[sendIndex] = 1000;
  for (int j = 0; j < 16; j++) {
    if (cellVoltage[j][sendIndex] > 1000) {
      moduleTotalVoltage[sendIndex] += cellVoltage[j][sendIndex];
      if (cellVoltage[j][sendIndex] > moduleMaxVoltage[sendIndex]) {
        moduleMaxVoltage[sendIndex] = cellVoltage[j][sendIndex];
      }
      if (cellVoltage[j][sendIndex] < moduleMinVoltage[sendIndex]) {
        moduleMinVoltage[sendIndex] = cellVoltage[j][sendIndex];
      }
    }
  }
  Serial.println(sendIndex);
}
void sendModuleInfo(int sendIndex) {
  static CAN_message_t moduleFrame;
  moduleFrame.ext = 0;
  moduleFrame.id = 1792 + sendIndex;
  moduleFrame.len = 8;
  moduleFrame.buf[0] = lowByte(moduleMinVoltage[sendIndex]);
  moduleFrame.buf[1] = highByte(moduleMinVoltage[sendIndex]);
  moduleFrame.buf[2] = lowByte(moduleMaxVoltage[sendIndex]);
  moduleFrame.buf[3] = highByte(moduleMaxVoltage[sendIndex]);
  moduleFrame.buf[4] = moduleTotalVoltage[sendIndex];
  moduleFrame.buf[5] = moduleTotalVoltage[sendIndex] >> 8;
  moduleFrame.buf[6] = moduleTotalVoltage[sendIndex] >> 16;
  moduleFrame.buf[7] = 0x00;
  Can0.write(moduleFrame);
}
