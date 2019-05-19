#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsgIn;
struct can_frame canMsgOut;
MCP2515 k500(9);
MCP2515 k250(10);


void setup() {
  //while (!Serial);
  Serial.begin(115200);
  SPI.begin();

  k500.reset();
  k500.setBitrate(CAN_500KBPS, MCP_8MHZ);
  k500.setNormalMode();

  k250.reset();
  k250.setBitrate(CAN_250KBPS, MCP_8MHZ);
  k250.setNormalMode();

  Serial.println("CANRelay");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (k500.readMessage(&canMsgIn) == MCP2515::ERROR_OK) {
    if (canMsgIn.can_id == 0x50f) {
      //Serial.print(canMsgIn.can_id, HEX); // print ID
     // Serial.print(" ");
      //Serial.print(canMsgIn.can_dlc, HEX); // print DLC
      //Serial.println(" ");

      for (int i = 0; i < canMsgIn.can_dlc; i++)  { // print the data

        //Serial.print(canMsgIn.data[i], HEX);
        //Serial.print(" ");
        canMsgOut.data[i] = canMsgIn.data[i];

      }

      //Serial.println();


      //Serial.println(canMsgOut.can_id, HEX); // print ID
      canMsgOut.can_dlc = canMsgIn.can_dlc;
      canMsgOut.can_id = 0x01dd0001 + 0x80000000;
      k250.sendMessage(&canMsgOut);
    }
  }
    if (k250.readMessage(&canMsgIn) == MCP2515::ERROR_OK) {
    if (canMsgIn.can_id == 0x1d4) {
      Serial.print(canMsgIn.can_id, HEX); // print ID
      Serial.print(" ");
      Serial.print(canMsgIn.can_dlc, HEX); // print DLC
      Serial.println(" ");

      for (int i = 0; i < canMsgIn.can_dlc; i++)  { // print the data

        Serial.print(canMsgIn.data[i], HEX);
        Serial.print(" ");
        canMsgOut.data[i] = canMsgIn.data[i];

      }

      Serial.println();


      Serial.println(canMsgOut.can_id, HEX); // print ID
      canMsgOut.can_dlc = canMsgIn.can_dlc;
      canMsgOut.can_id = canMsgIn.can_id;
      k500.sendMessage(&canMsgOut);
    }
  }
}
