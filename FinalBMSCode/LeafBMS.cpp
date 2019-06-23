#include "LeafBMS.h"


void LeafBMS::sendBMSData() {
  static CAN_message_t EVCCFrame;
  EVCCFrame.ext = 0;
  EVCCFrame.id = 0x50f;
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
  //if (!isConnected) {
  //bitSet(EVCCFrame.buf[2], 2);//therm overtemp
  //}
  //Serial.println(EVCCFrame.buf[1], BIN);

  static CAN_message_t infoFrame;
  infoFrame.ext = 0;
  infoFrame.id = 0x50e;
  infoFrame.len = 8;
  infoFrame.buf[0] = 0x00;//status; 1 = hvc, 2 = lvc, 4 = bvc
  infoFrame.buf[1] = 0x00;//fault; 1 = overtemp/undertemp
  infoFrame.buf[2] = lowByte(totalVoltage);//voltage LSB
  infoFrame.buf[3] = highByte(totalVoltage);//voltage MSB
  infoFrame.buf[4] = lowByte(highestVolt);
  infoFrame.buf[5] = highByte(highestVolt);
  infoFrame.buf[6] = lowByte(lowestVolt);
  infoFrame.buf[7] = highByte(lowestVolt);

  if (HVC) {
    bitSet(infoFrame.buf[0], 0);
  }
  if (LVC) {
    bitSet(infoFrame.buf[0], 1);
  }
  if (BVC) {
    bitSet(infoFrame.buf[0], 2);
  }

  Can0.write(EVCCFrame);
  delay(20);
  Can0.write(infoFrame);
  //Serial.println("Sent off data");

  EVCCTimer = 0;
}

void LeafBMS::gotFrame(CAN_message_t &frame, int mailbox)//Any frame received by the CAN hardware will be sent to this
{
#ifdef CANDEBUG
  printFrame(frame, mailbox);
#endif
  if (frame.id == 0x7bb) {//it's a response to something
    sortDataFrame(frame, mailbox);
  }
}

void LeafBMS::printFrame(CAN_message_t &frame, int mailbox)//prints received frames for debug purposes; only shows BMS messages
{
  if (frame.id == 0x7bb || frame.id == 0x79b) {
    Serial.print("ID: ");
    Serial.print(frame.id, HEX);
    Serial.print(" Data: ");
    for (int c = 0; c < frame.len; c++)
    {
      Serial.print(frame.buf[c], HEX);
      Serial.write(' ');
    }
    Serial.println();
  }
  else {}
}
void LeafBMS::sortDataFrame(CAN_message_t &frame, int mailbox)//first layer of sorting
{
  if ((frame.buf[0] == 0x10) && (frame.buf[3] == 0x04)) {//first packet, temp packet
    parseTempFrame(frame);
  }
  else if ((frame.buf[0] == 0x10) && (frame.buf[3] == 0x02)) {//first packet, cell packet
    parseCellFrame(frame);
  }
  else if ((frame.buf[0] == 0x10) && (frame.buf[3] == 0x01)) {//first packet, cell packet
    parseSOCFrame(frame);
  }
  else if ((frame.buf[0] == 0x10) && (frame.buf[3] == 0x03)) {//first packet, v packet
    parseVFrame(frame);
  }
  else if (gotTempFrame) {//currently grabbing temperature data
    parseTempFrame(frame);
  }
  else if (gotFirstCellFrame) {//currently grabbing cell data
    parseCellFrame(frame);
  }
  else if (gotSOCFrame) {//currently grabbing SOC data
    parseSOCFrame(frame);
  }
  else if (gotVFrame) {
    parseVFrame(frame);
  }
  else {}
}
void LeafBMS::parseTempFrame(CAN_message_t &frame) {//grabs data from temperature packets
  if ((frame.buf[0] == 0x10) && (frame.buf[3] == 0x04)) {//it's both the first packet and a temperature packet
    gotTempFrame = 1;
    tempData[0] = frame.buf[6];
  }
  else if (frame.buf[0] == 0x21) {//second packet and frame 1 of temp has just been received
    tempData[1] = frame.buf[2];
    tempData[2] = frame.buf[5];
  }
  else if (frame.buf[0] == 0x22) {
    gotTempFrame = 0;

    tempData[3] = frame.buf[1];
    for (int i = 0; i < 4; i++) {
    }
    tempTimer = millis();
  }
  else {}
}
//given a frame of cell data from group 2, this will figure out which it is and then store the data
//in the cellData[] array. When the last frame from the group is received, the data is converted to cell voltages
void LeafBMS::parseCellFrame(CAN_message_t &frame) {
  int index = 0;
  if (frame.buf[0] == 0x10) {//it's the first packet; this function only gets cell packets anyways
    gotFirstCellFrame = 1;
    for (int i = 0; i < 4; i++) {
      cellData[i] = frame.buf[i + 4];
    }
  } else {
    for (int i = 32; i < 48; i++) {//figures out where we are in the group
      if (frame.buf[0] == i && !gotHalfCellFrame) {//we're still in the first half of the data
        index = i - 31;
        for (int i = 1; i < 8; i++) {
          int place = (index * 7 - 11 + i);//do some complicated math that I don't really understand. All I know is, it works.
          cellData[place] = frame.buf[i];
        }
      } else if (frame.buf[0] == i && gotHalfCellFrame) {//second half is coming in
        index = i - 15;
        for (int i = 1; i < 8; i++) {
          int place = (index * 7 - 11 + i);//The 11 was determined empirically. This code does work
          cellData[place] = frame.buf[i];
        }
      } else {}
    }
  }
  if (index == 16) {//We got the first half of the group
    gotHalfCellFrame = 1;
  } else if (index == 29) {//it's the last one! Yay! Reset everything
    gotHalfCellFrame = 0;
    gotFirstCellFrame = 0;
    highestVolt = cellVoltages[0];//reset the max and min voltages
    lowestVolt = cellVoltages[0];
    for (int i = 0; i < 96; i++) {
      cellVoltages[i] = (cellData[i * 2] * 256) + cellData[(i * 2) + 1]; //convert from 2 bytes of hexadecimal to the actual voltage * 1000; add 15 idk why, some weirdness with this bms
      if (cellVoltages[i] > highestVolt) {
        highestVolt = cellVoltages[i];// it's the highest yet so store it
        highestCell = i + 1;
      }
      else if (cellVoltages[i] < lowestVolt) {
        lowestVolt = cellVoltages[i];//it's the lowest yet so store it
        lowestCell = i + 1;
      }
      else {}
      cellDiff = highestVolt - lowestVolt;//figure out the total difference; this is a good measure of pack health
    }
    if (lowestVolt > 0) {
      runs++;
      if (runs > 10) {
        runs = 10;
      }
      movingIndex++;
      if (movingIndex >= 10) {
        movingIndex = 0;
      }
      lowestVoltArray[movingIndex] = lowestVolt;
      for (int i = 0; i < 9; i++) {
        smoothLowestVolt += lowestVoltArray[i];
      }
      //Serial.println(lowestVolt);
      //Serial.println(smoothLowestVolt);
      if (runs == 10) {
        smoothLowestVolt = smoothLowestVolt / 10;
        //Serial.println("Using real smooth values");
      } else {
        smoothLowestVolt = lowestVolt;
      }
      //Serial.println(smoothLowestVolt);
      //Serial.println(movingIndex);
    }
    cellTimer = millis();//we got a full group, reset the counter
    heartbeat = millis();
    isConnected = true;
  } else {}
}

void LeafBMS::parseVFrame(CAN_message_t &frame) {
  int index = 0;
  if (frame.buf[0] == 0x10) { //it's the first packet and thus only contains a few bytes of data
    gotVFrame = 1;
    for (int i = 0; i < 4; i++) {
      vData[i] = frame.buf[i + 4];
    }
  } else {
    index = frame.buf[0] - 32;
    for (int a = 1; a < 8; a++) {//the rest of the packets are mostly data
      vData[(index * 7) - 4 + a] = frame.buf[a];
    }
  }

  if (index == 4) {
    gotVFrame = 0;
    vLowestVolt = vData[13] + (vData[12] * 256);
    vHighestVolt = vData[11] + (vData[10] * 256);
    //Serial.print("Lowest Volt: ");
    //Serial.println(vLowestVolt);
    //Serial.print("Highest Volt: ");
    //Serial.println(vHighestVolt);
    vCellDiff = vHighestVolt - vLowestVolt;
    vTimer = millis();

  }
}
void LeafBMS::parseSOCFrame(CAN_message_t &frame) {
  int index = 0;
  if (frame.buf[0] == 0x10) { //it's the first packet and thus only contains a few bytes of data
    gotSOCFrame = 1;
    for (int i = 0; i < 4; i++) {
      SOCData[i] = frame.buf[i + 4];
    }
  } else {
    index = frame.buf[0] - 32;
    for (int a = 1; a < 8; a++) {//the rest of the packets are mostly data
      SOCData[(index * 7) - 4 + a] = frame.buf[a];
    }
  }

  if (index == 5) {
    gotSOCFrame = 0;
    SOCTimer = millis();
    AH = (SOCData[33] * 65536 + SOCData[34] * 256 + SOCData[35]) / 10000.0000;
    SOH = (SOCData[26] * 256 + SOCData[27]) / 100.00;
    SOC = (SOCData[29] * 65536 + SOCData[30] * 256 + SOCData[31]);
  }
}

//Sends frames to the BMS requesting specific data sets
void LeafBMS::getGroup(int group) {
  static CAN_message_t allLineReq;
  static CAN_message_t groupReq;

  //this command requests all additional frames in
  allLineReq.ext = 0;//add'l lines
  allLineReq.id = 0x79b;
  allLineReq.len = 8;
  allLineReq.buf[0] = 0x30;
  allLineReq.buf[1] = 0x00;
  allLineReq.buf[2] = 0x00;
  allLineReq.buf[3] = 0x00;
  allLineReq.buf[4] = 0x00;
  allLineReq.buf[5] = 0x00;
  allLineReq.buf[6] = 0x00;
  allLineReq.buf[7] = 0x00;

  groupReq.ext = 0;//group 1 Request
  groupReq.id = 0x79b;
  groupReq.len = 8;
  groupReq.buf[0] = 0x02;
  groupReq.buf[1] = 0x21;
  groupReq.buf[2] = group;
  groupReq.buf[3] = 0x00;
  groupReq.buf[4] = 0x00;
  groupReq.buf[5] = 0x00;
  groupReq.buf[6] = 0x00;
  groupReq.buf[7] = 0x00;

  Can0.write(groupReq);
  delay(25);
  Can0.write(allLineReq);
  delay(25);
}
