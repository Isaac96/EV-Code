/*
   Nissan Leaf BMS data retrieval and interpretation; adapted from
   "Object Oriented CAN example for Teensy 3.6 with Dual CAN buses
   By Collin Kidder. Based upon the work of Pawelsky and Teachop"

   By Isaac Kelly

   The BnMS object handles all communication and reads frames as they come in
   Bus at 500K; alternate pins used with a pretty nice little CAN adapter from Croatia
   WARNING: This requires a slightly modified version of the FlexCAN library, unless you can tell me how to skip an input variable to a function


   This code is mostly interrupt-driven, especially when used in parallel with Leaf Spy
*/

#include <FlexCAN.h>
#include "LeafBMS.h"
unsigned long EVCCTimerMain = 0;
unsigned long timeNow = 0;
LeafBMS BMS;//make a BMS object

// -------------------------------------------------------------
void setup(void)
{
  delay(1000);
  //Serial.println("FinalBMSCode");//just so I know which code is uploaded

  pinMode(28, OUTPUT);//enable pin; this is specific to the CAN adapter I used
  digitalWrite(28, LOW);//it needs a strong pull to ground to enable transmissions
  Can0.begin(500000, 1, 1);//500K, alternate pins
  pinMode(13, OUTPUT);

  Can0.attachObj(&BMS);

  BMS.attachGeneralHandler();
}

// -------------------------------------------------------------
void loop(void)
{

  timeNow = millis();
  if (timeNow - BMS.heartbeat > BMS.heartbeatInterval) {//if there hasn't been data for a while, the BMS isn't connected anymore
    BMS.heartbeat = timeNow;
    BMS.isConnected = false;
  }
  if ((timeNow - EVCCTimerMain > BMS.EVCCInterval) && BMS.isConnected) {//Is it time to tell the EVCC what's going on? If the bms is disconnected let the evcc time out
    EVCCTimerMain = timeNow;
    BMS.sendBMSData();

    //Serial.print("Sending data to EVCC; min voltage is: ");
    //Serial.print(BMS.lowestVolt);
    //Serial.print("v; Max voltage: ");
    //Serial.println(BMS.highestVolt);
    digitalWrite(13, HIGH);
  }
  if (timeNow - BMS.vTimer > BMS.vInterval)   {//group 3 has vmin and vmax
    BMS.vTimer = timeNow;//cell data is the main focus of this software
    BMS.getGroup(3);//group 3 has what we need
  }
  if (timeNow - BMS.cellTimer > BMS.cellInterval)   {//better go grab the cell data
    BMS.cellTimer = timeNow;//cell data is the main focus of this software
    BMS.getGroup(2);//group 2 has cell data
    digitalWrite(13, LOW);
  }
  if (timeNow - BMS.SOCTimer > BMS.SOCInterval) {//probably need the SOC data by now
    BMS.SOCTimer = timeNow;
    BMS.getGroup(1);//group 1 has pack info data
  }
  if (timeNow - BMS.tempTimer > BMS.tempInterval) {//looks like temps need updating
    BMS.tempTimer = timeNow;
    BMS.getGroup(4);//group 4 holds temperatures
  }
  if (timeNow - BMS.debugTimer > 2000)   {
    BMS.debugTimer = timeNow;
    //printCSV();
    //printCellInfo();

  }
  if (Serial.available()) {
    while (Serial.available()) {
      Serial.read();
    }
    printCSV();
//printCellInfo();
    //BMS.getGroup(Serial.parseInt());//if you send it a number from 1-6 it will retrieve that CAN group
  }
  if (BMS.isConnected) {//there has been data in the last 10 seconds
    if (BMS.highestVolt >= BMS.maxCellVoltage) {
      //Serial.println("A cell is too high, stopping charge");//HVC triggered; stop charging
      BMS.HVC = true;
    }
    else if (BMS.highestVolt > BMS.balanceCellVoltage) {
      BMS.BVC = true;
    }
    else if (BMS.smoothLowestVolt <= BMS.minCellVoltage) {
      //Serial.println("Cell is too low. Turn off da car");//LVC triggered; stop driving
      BMS.LVC = true;
    }
    else {
      BMS.LVC = false;
      BMS.HVC = false;
      BMS.BVC = false;
    }
  }
}
void printCellInfo() {
  int tempVoltage = 0;
  Serial.print("Lowest cell: ");
  Serial.print(BMS.lowestVolt);
  Serial.print("mV, #");
  Serial.println(BMS.lowestCell);
  Serial.print("Highest cell: ");
  Serial.print(BMS.highestVolt);
  Serial.print("mV, #");
  Serial.println(BMS.highestCell);
  Serial.print("Difference: ");
  Serial.println(BMS.cellDiff);
  for (int i = 0; i < 96; i++) {
#ifdef DEBUG
    Serial.print("Cell number: ");
    Serial.print(i + 1);
    Serial.print(" Voltage: ");
    Serial.println(BMS.cellVoltages[i]);
#endif
    tempVoltage += BMS.cellVoltages[i];
  }

  BMS.totalVoltage = tempVoltage / 100;
  Serial.print("Total voltage: ");
  Serial.println(BMS.totalVoltage);
  if (!BMS.isConnected) {
    Serial.println("We appear to have lost connection");
  }
}

void printCSV() {
  Serial.print("f");
  for (int i = 0; i < 96; i++) {
    Serial.print(",");
    Serial.print(BMS.cellVoltages[i]);
  }
}
