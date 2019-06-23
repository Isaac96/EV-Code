#ifndef LEAFBMS_H_
#define LEAFBMS_H_
#include <Arduino.h>
#include <FlexCAN.h>

//#define DEBUG
//#define CANDEBUG


class LeafBMS : public CANListener {
  public:

    void getGroup(int group);//requests data from BMS

    void printFrame(CAN_message_t &frame, int mailbox);//debug function which spits out frames to the serial port
    void gotFrame(CAN_message_t &frame, int mailbox); //overrides the parent version so we can actually do something

    void sortDataFrame(CAN_message_t &frame, int mailbox);//sorts data frames and passes them to parsing functions
    void parseTempFrame(CAN_message_t &frame);//interprets temperature frames
    void parseCellFrame(CAN_message_t &frame);//interprets cell frames
    void parseSOCFrame(CAN_message_t &frame);//interprets pack info frames
    void parseVFrame(CAN_message_t &frame);//interprets pack info frames

    void sendBMSData();//sends data to EVCC w/ID 0x50e

    byte tempData[4];//data from BMS, which is also the actual temperatures
    byte cellData[200];//raw cell data from BMS
    int cellVoltages[96];//the actual voltages
    uint16_t highestVolt;//highest cell voltage
    uint16_t vHighestVolt;//highest cell voltage
    int highestCell;
    uint16_t lowestVolt;//lowest cell voltage
    uint32_t smoothLowestVolt;//smoother value
    uint16_t lowestVoltArray[10];//array for moving average
    int movingIndex = 0;
    int runs = 0;
    uint16_t vLowestVolt;//lowest cell voltage
    int lowestCell;
    int cellDiff;//difference between highest and lowest cells
    int vCellDiff;//difference between highest and lowest cells
    byte SOCData[39];//group 1 info; includes SOC, SOH, Ah remaining etc.
    byte vData[32];//group 4 info; has vMin, vMax, etc.
    uint16_t totalVoltage;

    float SOC = 0;//state of charge of pack
    float SOH = 0;//state of health; probably useless
    float AH = 0;//AH remaining; probably also useless

    bool gotTempFrame = 0;//states of data retrieval
    bool gotFirstCellFrame = 0;
    bool gotHalfCellFrame = 0;
    bool gotSOCFrame = 0;
    bool gotVFrame = 0;
    bool firstRun = false;

    bool isConnected = false;//whether there has been data recently
    bool HVC = false;
    bool LVC = false;
    bool BVC = false;
    bool allowCharge = false;
    bool allowDischarge = false;

    unsigned long tempTimer;//counters for data retrieval timing
    unsigned long vTimer;
    unsigned long cellTimer;
    unsigned long SOCTimer;
    unsigned long debugTimer;
    unsigned long EVCCTimer;
    unsigned long heartbeat;//the time since the last cell data; makes sure wires stay connected
    unsigned long SOCInterval = 3000;
    unsigned long cellInterval = 3000;
    unsigned long vInterval = 3000;
    unsigned long tempInterval = 10000;
    unsigned long debugInterval = 1000;
    unsigned long heartbeatInterval = 10000;
    unsigned long EVCCInterval = 500;

    int maxCellVoltage = 4200;//configure the max and min voltages you wish
    int minCellVoltage = 3300;
    int balanceCellVoltage = 4150;
};

#endif
