/*
   Nissan Leaf BMS data retrieval and interpretation; adapted from
   "Object Oriented CAN example for Teensy 3.6 with Dual CAN buses
   By Collin Kidder. Based upon the work of Pawelsky and Teachop"

   By Isaac Kelly

   The BMS object handles all communication and reads frames as they come in
   Bus at 500K; alternate pins used with a pretty nice little CAN adapter from Croatia
   WARNING: This requires a slightly modified version of the FlexCAN library, unless you can tell me how to skip an input variable to a function


   This code is mostly interrupt-driven, especially when used in parallel with Leaf Spy
*/

#include <FlexCAN.h>
#include "LeafBMS.h"

LeafBMS BMS;//make a BMS object
unsigned long startupDelay = 10000;

// -------------------------------------------------------------
void setup(void)
{
  delay(1000);
  Serial.println(F("FinalBMSCode"));//just so I know which code is uploaded

  pinMode(28, OUTPUT);//enable pin; this is specific to the CAN adapter I used
  digitalWrite(28, LOW);//it needs a strong pull to ground to enable transmissions

  Can0.begin(500000, 1, 1);//500K, alternate pins

  Can0.attachObj(&BMS);

  BMS.attachGeneralHandler();
}

// -------------------------------------------------------------
void loop(void)
{

  int tempVoltage = 0;
  long timeNow = millis();
  if (timeNow - BMS.heartbeat > BMS.heartbeatInterval) {//if there hasn't been data for a while, the BMS isn't connected anymore
    BMS.heartbeat = timeNow;
    BMS.isConnected = false;
  }
  if (timeNow - BMS.EVCCTimer > BMS.EVCCInterval && BMS.isConnected) {//Is it time to tell the EVCC what's going on? If the bms is disconnected let the evcc time out
    BMS.EVCCTimer = timeNow;
    if (millis() > startupDelay) {
      BMS.sendBMSData();
    }
  }
  if (timeNow - BMS.vTimer > BMS.vInterval)   {//group 3 has vmin and vmax
    BMS.vTimer = timeNow;//cell data is the main focus of this software
    BMS.getGroup(3);//group 3 has what we need
    Serial.println("Getting vData");
  }
  if (timeNow - BMS.cellTimer > BMS.cellInterval)   {//better go grab the cell data
    BMS.cellTimer = timeNow;//cell data is the main focus of this software
    BMS.getGroup(2);//group 2 has cell data
  }
  if (timeNow - BMS.SOCTimer > BMS.SOCInterval) {//probably need the SOC data by now
    BMS.SOCTimer = timeNow;
    BMS.getGroup(1);//group 1 has pack info data
  }
  if (timeNow - BMS.tempTimer > BMS.tempInterval) {//looks like temps need updating
    BMS.tempTimer = timeNow;
    BMS.getGroup(4);//group 4 holds temperatures
  }
  if (timeNow - BMS.debugTimer > BMS.debugInterval)   {

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
    }    BMS.debugTimer = timeNow;
  }
  if (Serial.available()) {
    BMS.getGroup(Serial.parseInt());//if you send it a number from 1-6 it will retrieve that CAN group
  }
  if (BMS.isConnected) {//there has been data in the last 10 seconds
    if (BMS.highestVolt >= BMS.maxCellVoltage) {
      //Serial.println("A cell is too high, stopping charge");//HVC triggered; stop charging
      BMS.HVC = true;
    }
    else if (BMS.lowestVolt <= BMS.minCellVoltage) {
      //Serial.println("Cell is too low. Turn off da car");//LVC triggered; stop driving
      BMS.LVC = true;
    }
    else {
      BMS.LVC = false;
      BMS.HVC = false;
    }
  }
}
