// Demo: NMEA2000 library. Send different NMEA 2000 messages as example to the bus.
//
// Original MessageSender sent different messages mostly with 1000 ms or 100 ms period. The
// new version sends all messages with their right (NMEA2000 specified) period. Messages also
// has different offsets to avoid filling up buffer at once. NMEA2000 bus can handle about
// 2.5 frame/ms. So two single frame messages could be sent at same time, but e.g. 7 frame
// GNSS position data takes about 3 ms. With unlapping offsets there should be only one message
// on buffer at time
//
// If you need to test new message for testing, write sender function - see. e.g. SendN2kTemperatureExt
// around line 320 (may change after update). Then add function to N2kSendMessages (somewhere line 470 )
// vector in similar way as SendN2kTemperatureExt. Set message period value according to message
// definition - some periods has been listed also on NMEA2000.cpp.
//
// Messages sending will be done by SendN2kMessages, which loops through message sender vector and sets
// loop NextSend scheduler to next nearest sent time. In this way function does not need every time
// loop through all messages. When it is time to send some messages, it loops them, sends necessary messages 
// and sets NextSend to next nearest time. In ESP32 message sending could be own task and NextSend
// could be used just delay to leave time for other tasks.
//
// MessageSender has simple command line interface. With simple commands you e.g. disable all messages and
// then enable them one by one. To get available commands type ? and line feed to console to get help.

#include <Arduino.h>
//#define N2k_SPI_CS_PIN 53    // Pin for SPI select for mcp_can
//#define N2k_CAN_INT_PIN 21   // Interrupt pin for mcp_can
//#define USE_MCP_CAN_CLOCK_SET 8  // Uncomment this, if your mcp_can shield has 8MHz chrystal
//#define ESP32_CAN_TX_PIN GPIO_NUM_16 // Uncomment this and set right CAN TX pin definition, if you use ESP32 and do not have TX on default IO 16
//#define ESP32_CAN_RX_PIN GPIO_NUM_17 // Uncomment this and set right CAN RX pin definition, if you use ESP32 and do not have RX on default IO 4

#define ESP32_CAN_RX_PIN GPIO_NUM_22
#define ESP32_CAN_TX_PIN GPIO_NUM_23


//#define NMEA2000_ARDUINO_DUE_CAN_BUS tNMEA2000_due::CANDevice1    // Uncomment this, if you want to use CAN bus 1 instead of 0 for Arduino DUE
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <N2kMaretron.h>
#include "navigate.h"



using tN2kSendFunction=void (*)();

// Structure for holding message sending information
struct tN2kSendMessage {
  tN2kSendFunction SendFunction;
  const char *const Description;
  unsigned long NextSend;
  unsigned long Period;
  unsigned long Offset;
  bool Enabled;

  void Enable(bool state);
  void SetNextUpdate();
};

extern tN2kSendMessage N2kSendMessages[];
extern size_t nN2kSendMessages;

static unsigned long N2kMsgSentCount=0;
static unsigned long N2kMsgFailCount=0;
static bool ShowSentMessages=false;
static bool ShowStatistics=false;
static bool Sending=true;
static bool EnableForward=false;
static unsigned long NextStatsTime=3000;
static unsigned long StatisticsPeriod=2000;

Position wp1 = { 51.9379066059753, 1.491858780192354, NULL};
Position wp2 = { 51.90313504276786, 1.3797427676025804, NULL};
Position wp3 = { 51.909854375508516, 1.3451469694320215, NULL};
Boat boat = Boat();

// Forward declarations for functions
void CheckCommand(); 
void CheckLoopTime();
bool IsTimeToUpdate(unsigned long NextUpdate);
void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period, unsigned long Offset=0);


void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  N2kMsg.Print(&Serial);
}

// *****************************************************************************
void setup() {
  // Initialize serial port.
  Serial.begin(115200);
  delay(2000); // Give a bit time for console to be ready

  // Reserve enough buffer for sending all messages. 
  // This does not work on small memory devices like Uno or Mega
  // This version should work with smaller buffer, since message sending has been offsetted.
  NMEA2000.SetN2kCANSendFrameBufSize(250);
  // Set Product information
  NMEA2000.SetProductInformation("00000001", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Message sender example",  // Manufacturer's Model ID
                                 "1.1.1.30 (2021-02-05)",  // Manufacturer's Software version code
                                 "1.1.1.0 (2021-02-05)" // Manufacturer's Model version
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );
  // Uncomment 3 rows below to see, what device will send to bus                           
   NMEA2000.SetForwardStream(&Serial);  // PC output to default serial port
   NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
   NMEA2000.SetForwardOwnMessages(false); // Do not print own messages.


  NMEA2000.SetN2kCANReceiveFrameBufSize(150);
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  // We act as real node on bus. Some devices does not show messages, if they can not request information.
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,22);
  //NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(EnableForward); // Disable all msg forwarding to USB (=Serial)
  NMEA2000.Open();



  wp1.next = &wp2;
  wp2.next = &wp3;
  wp3.next = &wp1;
  boat.setRoute(&wp1);

  Serial.println("Starting message sender!");
  Serial.println("  Message sending to bus starts in 10 seconds.");
  Serial.println("  Type ?<lf> for available commands.");
}





// *****************************************************************************
void SendN2kMsg(const tN2kMsg &N2kMsg) {
  if ( NMEA2000.SendMsg(N2kMsg) ) {
    N2kMsgSentCount++;
  } else {
    N2kMsgFailCount++;
  }
  if ( ShowSentMessages ) N2kMsg.Print(&Serial);
}

// *****************************************************************************
// Function check is it time to send message. If it is, message will be sent and
// next send time will be updated.
// Function always returns next time it should be handled.
unsigned long CheckSendMessage(tN2kSendMessage &N2kSendMessage) {
  if ( !N2kSendMessage.Enabled ) return millis()+2000;

  if ( IsTimeToUpdate(N2kSendMessage.NextSend) ) {
    if ( N2kSendMessage.NextSend!=0 ) { // do not send on first round
      N2kSendMessage.SendFunction();
    }
    N2kSendMessage.SetNextUpdate();
  }

  return N2kSendMessage.NextSend;
}

// *****************************************************************************
// Function send enabled messages from tN2kSendMessage structure according to their
// period+offset.
void SendN2kMessages() {
  static unsigned long NextSend=10000;

  if ( IsTimeToUpdate(NextSend) ) {
    unsigned long NextMsgSend;
    NextSend=millis()+2000;
    for ( size_t i=0; i<nN2kSendMessages; i++ ) {
      NextMsgSend=CheckSendMessage(N2kSendMessages[i]);
      if ( NextMsgSend<NextSend ) NextSend=NextMsgSend;
    }
  }
}

void Navigate() {
  static unsigned long NextNav=0;

  if ( IsTimeToUpdate(NextNav) ) {
    NextNav=millis()+1000;
    boat.navigate(2000); // 2x rate
  }

}




// *****************************************************************************
void loop() {
  Navigate();
  if ( Sending ) SendN2kMessages();
  NMEA2000.ParseMessages();
  CheckCommand();

  if ( ShowStatistics ) {
    if ( IsTimeToUpdate(NextStatsTime) ) {
      SetNextUpdate(NextStatsTime,StatisticsPeriod);
      char buf[200];
      snprintf(buf,200,"\nMessages sent:%lu, sent failed:%lu\n",N2kMsgSentCount,N2kMsgFailCount);
      Serial.print(buf);
    }

    CheckLoopTime();
  }
}


double periodicValue(double mean, double amplitude, double periodS) {
    return mean + amplitude*sin(((2.0*PI)/(periodS*1000))*millis());
}


// *****************************************************************************
double ReadCabinTemp() {

  return CToKelvin(periodicValue(21.11,3,100)); // Read here the true temperature e.g. from analog input
}

// *****************************************************************************
double ReadWaterTemp() {
  return CToKelvin(periodicValue(15.5,5,200)); // Read here the true temperature e.g. from analog input
}

// *****************************************************************************
void SendN2kAttitude() {
  tN2kMsg N2kMsg;
  SetN2kAttitude(N2kMsg,1,DegToRad(periodicValue(0,3.1,30)),DegToRad(periodicValue(0,3.1,20)),DegToRad(periodicValue(0,8,15)));
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kSystemTime() {
  tN2kMsg N2kMsg;
  SetN2kSystemTime(N2kMsg,1,17555,62000);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kLocalOffset() {
  tN2kMsg N2kMsg;
  SetN2kLocalOffset(N2kMsg,17555,62000,120);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kRudder() {
  tN2kMsg N2kMsg;
  SetN2kRudder(N2kMsg,DegToRad(periodicValue(0,5,30)),1,N2kRDO_MoveToStarboard,DegToRad(-5));
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kGNSS() {
  tN2kMsg N2kMsg;
  unsigned long now = millis();
  uint16_t daysSince1970 = 18973+(now/(24*3600000));
  double secondsSinceMidnight = (now%(24*3600000))/1000;

  SetN2kGNSS(N2kMsg,1,daysSince1970,secondsSinceMidnight,boat.pos.lat,boat.pos.lon,10.5,N2kGNSSt_GPS,N2kGNSSm_GNSSfix,12,0.8,0.5,15,1,N2kGNSSt_GPS,15,2);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kGNSSDOPData() {
  tN2kMsg N2kMsg;
  SetN2kGNSSDOPData(N2kMsg,1,N2kGNSSdm_Auto,N2kGNSSdm_Auto,1.2,-0.8,N2kDoubleNA);
  SendN2kMsg(N2kMsg);
}

void SendN2kDistanceLog() {
  tN2kMsg N2kMsg;

         unsigned long now = millis();
         uint16_t daysSince1970 = 18973+(now/(24*3600000));
         double secondsSinceMidnight = (now%(24*3600000))/1000;
         uint32_t log =  boat.log;// 1852  + (1852.0*now)/(6000);
         uint32_t  tripLog = boat.log; //(1852.0*now)/(6000);

  SetN2kDistanceLog(N2kMsg,daysSince1970,secondsSinceMidnight, log, tripLog);
  SendN2kMsg(N2kMsg);

}

// *****************************************************************************
void SendN2kGNSSSatsInView() {
  tN2kMsg N2kMsg;
  // Init message
  SetN2kGNSSSatellitesInView(N2kMsg,0xff,N2kDD072_RangeResidualsWereUsedToCalculateData);
  // Add satellites
  tSatelliteInfo SatelliteInfo;
  SatelliteInfo.PRN=21;
  SatelliteInfo.Elevation=DegToRad(44.94);
  SatelliteInfo.Azimuth=DegToRad(162.67);
  SatelliteInfo.SNR=47.6;
  SatelliteInfo.RangeResiduals=7951.32;
  SatelliteInfo.UsageStatus=N2kDD124_UsedInSolutionWithoutDifferentialCorrections;
  AppendSatelliteInfo(N2kMsg,SatelliteInfo);
  SatelliteInfo.PRN=19;
  SatelliteInfo.Elevation=DegToRad(27.39);
  SatelliteInfo.Azimuth=DegToRad(187.74);
  SatelliteInfo.SNR=44.1;
  SatelliteInfo.RangeResiduals=10105.22;
  SatelliteInfo.UsageStatus=N2kDD124_UsedInSolutionWithoutDifferentialCorrections;
  AppendSatelliteInfo(N2kMsg,SatelliteInfo);
  SatelliteInfo.PRN=31;
  SatelliteInfo.Elevation=DegToRad(26.70);
  SatelliteInfo.Azimuth=DegToRad(106.00);
  SatelliteInfo.SNR=43.9;
  SatelliteInfo.RangeResiduals=10177.9;
  SatelliteInfo.UsageStatus=N2kDD124_UsedInSolutionWithoutDifferentialCorrections;
  AppendSatelliteInfo(N2kMsg,SatelliteInfo);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendUserDatumData() {
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(129045L);
  N2kMsg.Priority=6;
  N2kMsg.Add4ByteDouble(3.25,1e-2); // Delta X
  N2kMsg.Add4ByteDouble(-3.19,1e-2); // Delta Y
  N2kMsg.Add4ByteDouble(1.1,1e-2); // Delta Z
  N2kMsg.AddFloat(DegToRad(0.123)); // Rotation in X
  N2kMsg.AddFloat(DegToRad(-0.0123)); // Rotation in Y
  N2kMsg.AddFloat(DegToRad(0.00123)); // Rotation in Z
  N2kMsg.AddFloat(1.001); // Scale
  N2kMsg.Add4ByteDouble(N2kDoubleNA,1e-7); // Ellipsoid Semi-major Axis
  N2kMsg.AddFloat(15.23456); // Ellipsoid Flattening Inverse
  N2kMsg.Add4ByteUInt(0xffffffff);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendTideStationData() {
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(130320L);
  N2kMsg.Priority=6;
  N2kMsg.AddByte(0xc0); // Mode+Tide Tendency+reserved
  N2kMsg.Add2ByteUInt(18498); // Measurement Date
  N2kMsg.Add4ByteUDouble(43210,0.0001); // Measurement Time
  N2kMsg.Add4ByteDouble(60.436616000,1e-7); // Station location, latitude
  N2kMsg.Add4ByteDouble(22.237820000,1e-7); // Station location, longitude
  N2kMsg.Add2ByteDouble(3.25,1e-3); // Tide level
  N2kMsg.Add2ByteUDouble(1.56,1e-2); // Tide level standard deviation
  N2kMsg.AddVarStr("00022");
  N2kMsg.AddVarStr("Test");
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendSalinity() {
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(130321L);
  N2kMsg.Priority=6;
  N2kMsg.AddByte(0xf1); // Mode+reserved
  N2kMsg.Add2ByteUInt(18498); // Measurement Date
  N2kMsg.Add4ByteUDouble(43210,0.0001); // Measurement Time
  N2kMsg.Add4ByteDouble(60.436616000,1e-7); // Station location, latitude
  N2kMsg.Add4ByteDouble(22.237820000,1e-7); // Station location, longitude
//  N2kMsg.Add4ByteDouble(N2kDoubleNA,1e-7); // Station location, latitude
//  N2kMsg.Add4ByteDouble(N2kDoubleNA,1e-7); // Station location, longitude
  N2kMsg.AddFloat(34.56789); // Salinity
  N2kMsg.Add2ByteUDouble(293.15,0.01);
  N2kMsg.AddVarStr("00022");
  N2kMsg.AddVarStr("Test");
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kProductInformation() {
  NMEA2000.SendProductInformation();
  N2kMsgSentCount++;
}

// *****************************************************************************
void SendN2kIsoAddressClaim() {
  // Note that sometime NMEA Reader gets grazy, when ISO Address claim will be sent periodically.
  // If that happens, reopen NMEA Reader.
  NMEA2000.SendIsoAddressClaim();
  N2kMsgSentCount++;
}

// *****************************************************************************
void SendN2kPressure() {
  tN2kMsg N2kMsg;
  SetN2kPressure(N2kMsg,0,2,N2kps_Atmospheric,mBarToPascal(periodicValue(1024,20,200)));
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kSetPressure() {
  tN2kMsg N2kMsg;
  SetN2kSetPressure(N2kMsg,0,2,N2kps_CompressedAir,1255);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kHumidity() {
  tN2kMsg N2kMsg;
  SetN2kHumidity(N2kMsg,0xff,1,N2khs_InsideHumidity,periodicValue(55,10,100)); //,43);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kTemperatureExt() {
  tN2kMsg N2kMsg;
  SetN2kTemperatureExt(N2kMsg, 1, 1, N2kts_MainCabinTemperature, ReadCabinTemp(),CToKelvin(21.6));
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kTemperature() {
  tN2kMsg N2kMsg;
  SetN2kTemperature(N2kMsg, 1, 1, N2kts_MainCabinTemperature, ReadCabinTemp(),CToKelvin(21.6));
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendEnvironmentalParameters() {
  tN2kMsg N2kMsg;
  SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_MainCabinTemperature, ReadCabinTemp(),N2khs_InsideHumidity, 55, mBarToPascal(1013.5));
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendOutsideEnvironmentalParameters() {
  tN2kMsg N2kMsg;
  SetN2kOutsideEnvironmentalParameters(N2kMsg, 1, ReadWaterTemp(), CToKelvin(25.3), mBarToPascal(1013.5));
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kMaretronTempHR() {
  tN2kMsg N2kMsg;
  SetN2kMaretronTempHR(N2kMsg,0xff,1,N2kts_MainCabinTemperature,CToKelvin(24.7));
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kChargerStatus() {
  tN2kMsg N2kMsg;
  SetN2kChargerStatus(N2kMsg, 1, 2, N2kCS_Absorption, N2kCM_Primary, N2kOnOff_On, N2kOnOff_On, 180);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kDCBatStatus1() {
  tN2kMsg N2kMsg;
  SetN2kDCBatStatus(N2kMsg, 1, 12.72);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kDCBatStatus2() {
  tN2kMsg N2kMsg;
  static unsigned char sid=0;
//    SetN2kDCBatStatus(N2kMsg, 0, 12.45, 5.08, CToKelvin(27.15));
  SetN2kPGN127508(N2kMsg, 0, 13.8, 0.95, N2kDoubleNA, sid++);
  if ( sid>252 ) sid=0;
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kBatConf() {
  tN2kMsg N2kMsg;
  SetN2kBatConf(N2kMsg,1,N2kDCbt_AGM,N2kDCES_Yes,N2kDCbnv_12v,N2kDCbc_LeadAcid,AhToCoulomb(410),95,1.26,97);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kDCStatus() {
  tN2kMsg N2kMsg;
  SetN2kDCStatus(N2kMsg,1,1,N2kDCt_Alternator,86,91,1420,0.215,N2kDoubleNA);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kTransmissionParameters() {
  tN2kMsg N2kMsg;
  SetN2kTransmissionParameters(N2kMsg,0,N2kTG_Forward,750000, CToKelvin(65.5),true,false,true);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kAISClassAPosition() {
  tN2kMsg N2kMsg;

  SetN2kAISClassAPosition(N2kMsg, 1, tN2kAISRepeat::N2kaisr_First, 123456789, 26.396, -80.075, 
              1, 1, 1, 20, 20, tN2kAISTransceiverInformation::N2kaischannel_A_VDL_reception, 30, 0, tN2kAISNavStatus::N2kaisns_At_Anchor);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kBinaryStatus1() {
  tN2kMsg N2kMsg;
  SetN2kBinaryStatus(N2kMsg,2,N2kOnOff_On,N2kOnOff_Unavailable,N2kOnOff_Off);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kBinaryStatus2() {
  tN2kMsg N2kMsg;
  tN2kBinaryStatus SwitchBoard;
  N2kResetBinaryStatus(SwitchBoard);
  N2kSetStatusBinaryOnStatus(SwitchBoard,N2kOnOff_On,7);
  SetN2kBinaryStatus(N2kMsg,3,SwitchBoard);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kEngineTripParameters() {
  tN2kMsg N2kMsg;
  SetN2kEngineTripParameters(N2kMsg,0,237,3.6,4.5,4.4);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kEngineDynamicParam() {
  tN2kMsg N2kMsg;
  tN2kEngineDiscreteStatus1 Status1;
  Status1.Bits.LowOilLevel=1;
  Status1.Bits.OverTemperature=1;
  SetN2kEngineDynamicParam(N2kMsg,0,656000,CToKelvin(86.3),CToKelvin(82.1),14.21,5.67,hToSeconds(2137.55),N2kDoubleNA,N2kDoubleNA,N2kInt8NA,N2kInt8NA,Status1);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kEngineRapid() {
  tN2kMsg N2kMsg;
  SetN2kEngineParamRapid(N2kMsg,0,4350); //,820000,48);
  SendN2kMsg(N2kMsg);
}

// *****************************************************************************
void SendN2kCOGSOGRapid() {
  tN2kMsg N2kMsg;
  SetN2kCOGSOGRapid(N2kMsg,1,N2khr_true,boat.cog,boat.sog);
 // SetN2kCOGSOGRapid(N2kMsg,1,N2khr_true,boat.cog  DegToRad(periodicValue(115.6,10,30)),periodicValue(6.1,3,100));
  SendN2kMsg(N2kMsg);
  boat.sog = periodicValue(6.1,3,100);
}

// *****************************************************************************
void SendN2kMagneticHeading() {
  tN2kMsg N2kMsg;
  SetN2kMagneticHeading(N2kMsg, 0, boat.cog, DegToRad(-3.0), DegToRad(5.5)); 
//  SetN2kMagneticHeading(N2kMsg, 0, boat.cogDegToRad(periodicValue(227.5,10,30)), DegToRad(-3.0), DegToRad(5.5)); 
  SendN2kMsg(N2kMsg);
}

void SendN2kWind() {
  tN2kMsg N2kMsg;
  SetN2kWindSpeed(N2kMsg, 0, KnotsToms(periodicValue(15,10,300)), DegToRad(periodicValue(180,160,3000)), N2kWind_Apparent); 
  SendN2kMsg(N2kMsg);
}

void SendN2kRateOfTurn() {
  tN2kMsg N2kMsg;
  SetN2kRateOfTurn(N2kMsg, 0, DegToRad(1.0)); 
  SendN2kMsg(N2kMsg);
}

void SendN2kMagneticVariation() {
  tN2kMsg N2kMsg;
  unsigned long now = millis();
  uint16_t daysSince1970 = 18973+(now/(24*3600000));
  SetN2kMagneticVariation(N2kMsg, 0, N2kmagvar_Chart, daysSince1970, DegToRad(-1.1)); 
  SendN2kMsg(N2kMsg);
}

void SendN2kFluidLevel() {
  tN2kMsg N2kMsg;
  SetN2kFluidLevel(N2kMsg, 0, N2kft_Fuel, 84, 60); 
  SendN2kMsg(N2kMsg);
  SetN2kFluidLevel(N2kMsg, 1, N2kft_Water, 81, 200); 
  SendN2kMsg(N2kMsg);
  SetN2kFluidLevel(N2kMsg, 2, N2kft_Water, 55, 200); 
  SendN2kMsg(N2kMsg);
}

void SendN2kSpeed() {
  tN2kMsg N2kMsg;
  SetN2kBoatSpeed(N2kMsg, 34,KnotsToms(periodicValue(12,4,100))); 
  SendN2kMsg(N2kMsg);
}
void SendN2kWaterDepth() {
  tN2kMsg N2kMsg;
  SetN2kWaterDepth(N2kMsg, 34, periodicValue(15,5,100),0.1); 
  SendN2kMsg(N2kMsg);
}

void SendN2kCrossTrackError() {
  tN2kMsg N2kMsg;
  SetN2kXTE(N2kMsg, 34, N2kxtem_Autonomous, false, 10.2); 
  SendN2kMsg(N2kMsg);
}


// The following are messages captured from a SeatalkNG bus

// Raymarine proprietary, backlight, probably wrong
void SendN2k126720() {
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(126720);
  N2kMsg.Priority=7;
  N2kMsg.AddByte(0x3B);
  N2kMsg.AddByte(0x9F);
  N2kMsg.AddByte(0xF0);
  N2kMsg.AddByte(0x81);
  N2kMsg.AddByte(0x84);
  N2kMsg.AddByte(0x6);
  N2kMsg.AddByte(0x6);
  N2kMsg.AddByte(0x18);
  N2kMsg.AddByte(0x40);
  N2kMsg.AddByte(0x0);
  N2kMsg.AddByte(0xB);
  N2kMsg.AddByte(0x2);
  N2kMsg.AddByte(0x8);
  SendN2kMsg(N2kMsg);
}
// Raymarine proprietary, headding track control, probably wrong
void SendN2k127237() {
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(127237);
  N2kMsg.Priority=2;
  N2kMsg.AddByte(0x3C);
  N2kMsg.AddByte(0xC0);
  N2kMsg.AddByte(0x1F);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0x7F);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0x7F);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0x7F);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0x7F);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  SendN2kMsg(N2kMsg);
}

// Raymarine proprietary, unknown, probably wrong
void SendN2k130916() {
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(130916);
  N2kMsg.Priority=7;
   N2kMsg.AddByte(0x3B);
  N2kMsg.AddByte(0x9F);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0x7F);
  N2kMsg.AddByte(0xFF);
  SendN2kMsg(N2kMsg);
}
// Raymarine proprietary, seatalk pilot heading, probably wrong
void SendN2k65359() {
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(65359);
  N2kMsg.Priority=7;
  N2kMsg.AddByte(0x3B);
  N2kMsg.AddByte(0x9F);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0x2F);
  N2kMsg.AddByte(0x8);
  N2kMsg.AddByte(0xFF);
  SendN2kMsg(N2kMsg);
}


// Raymarine proprietary, seatalk pilot heading, probably wrong
void SendN2k65379() {
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(65379);
  N2kMsg.Priority=7;
  N2kMsg.AddByte(0x3B);
  N2kMsg.AddByte(0x9F);
  N2kMsg.AddByte(0x0);
  N2kMsg.AddByte(0x0);
  N2kMsg.AddByte(0x0);
  N2kMsg.AddByte(0x0);
  N2kMsg.AddByte(0x2);
  N2kMsg.AddByte(0xFF);
  SendN2kMsg(N2kMsg);
}

// Raymarine proprietary, seatalk pilot heading, probably wrong
void SendN2k65384() {
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(65384);
  N2kMsg.Priority=7;
  N2kMsg.AddByte(0x3B);
  N2kMsg.AddByte(0x9F);
  N2kMsg.AddByte(0x0);
  N2kMsg.AddByte(0x0);
  N2kMsg.AddByte(0x0);
  N2kMsg.AddByte(0x0);
  N2kMsg.AddByte(0x0);
  N2kMsg.AddByte(0x0);
  SendN2kMsg(N2kMsg);
}



/*

PGN:126208 group function request, ignore
PGN:126720 Ray probably backlight setting, ignore  PGN:126720 Source:204 Dest:255 Len:13 Data:3B,9F,F0,81,84,6,6,18,40,0,B,2,8
PGN:126993 heartbeat, ignore
PGN:127237 Heading/track control, TODO 234496 : Pri:2 PGN:127237 Source:172 Dest:255 Len:21 Data:3C,C0,1F,FF,7F,FF,FF,FF,FF,FF,FF,FF,FF,FF,7F,FF,7F,FF,7F,FF,FF
PGN:127245 Rudder - sent
PGN:127250 Vessel Heading - sent
PGN:127251 Rate of Turn - senmt
PGN:127257 Attitude - sent
PGN:127258 Magnetic Variation - sent
PGN:127488 Engine Parameters, Rapid Update - sent
PGN:127489 Engine Parameters, Dynamic -sent
PGN:127505 Fluid Level - sent 
PGN:127508 Battery Status -sent
PGN:128259 Speed - sent 
PGN:128267  Water Depth - sent
PGN:128275  Distance Log - sent
PGN:129044 Datum - Ignore 
PGN:129283 Cross Track Error - sent 
PGN:130306 Wind Data - sent
PGN:130310 Environmental Parameters - sent
PGN:130312 Temperature - sent
PGN:130916 Ray Unknown   229408 : Pri:7 PGN:130916 Source:204 Dest:255 Len:7 Data:3B,9F,FF,FF,FF,7F,FF
PGN:59904 Iso Request ignore
PGN:60928 Iso Address Claim  ignore
PGN:65359 Seatalk Pilot Headding - ignore 238179 : Pri:7 PGN:65359 Source:204 Dest:255 Len:8 Data:3B,9F,FF,FF,FF,2F,8,FF
PGN:65379 Seatalk Pilot mode  229270 : Pri:7 PGN:65379 Source:204 Dest:255 Len:8 Data:3B,9F,0,0,0,0,2,FF
PGN:65384 Ray Unknown 229172 : Pri:7 PGN:65384 Source:0 Dest:255 Len:8 Data:3B,9F,0,0,0,0,0,0
*/

#define AddSendPGN(fn,NextSend,Period,Offset,Enabled) {fn,#fn,NextSend,Period,Offset,Enabled}

tN2kSendMessage N2kSendMessages[]={
   AddSendPGN(SendN2kIsoAddressClaim,0,5000,0,false) // 60928 Not periodic
  ,AddSendPGN(SendN2kSystemTime,0,1000,0,true) // 126992
  ,AddSendPGN(SendN2kProductInformation,0,5000,60,false) // 126996 (20) Not periodic
  ,AddSendPGN(SendN2kRudder,0,100,0,true) // 127245
  ,AddSendPGN(SendN2kMagneticHeading,0,100,1,true) // 127250
  ,AddSendPGN(SendN2kAttitude,0,1000,20,true) // 127257
  ,AddSendPGN(SendN2kEngineRapid,0,100,2,true) // 127488
  ,AddSendPGN(SendN2kEngineDynamicParam,0,500,21,true) // 127489 (4)
//  ,AddSendPGN(SendN2kTransmissionParameters,0,100,3,true) // 127493
//  ,AddSendPGN(SendN2kEngineTripParameters,0,1000,25,true) // 127497 (2)
//  ,AddSendPGN(SendN2kBinaryStatus1,0,2500,27,true) // 127501
//  ,AddSendPGN(SendN2kBinaryStatus2,0,2500,28,true) // 127501
//  ,AddSendPGN(SendN2kChargerStatus,0,1500,29,true) // 127507 
  ,AddSendPGN(SendN2kDCBatStatus1,0,1500,30,true) // 127508
  ,AddSendPGN(SendN2kDCBatStatus2,0,1500,31,true) // 127508
  ,AddSendPGN(SendN2kDCStatus,0,1500,32,true) // 127506 (2)
  ,AddSendPGN(SendN2kBatConf,0,5000,34,false) // 127513 Not periodic
  ,AddSendPGN(SendN2kCOGSOGRapid,0,250,0,true) // 129026
  ,AddSendPGN(SendN2kGNSS,0,1000,75,true) // 129029 (7)
  ,AddSendPGN(SendN2kDistanceLog,0,1000,23,true) // Distance log 
  ,AddSendPGN(SendN2kLocalOffset,0,5000,36,false) // 129033 Not periodic
  ,AddSendPGN(SendN2kAISClassAPosition,0,5000,80,false) // 129038 (4) Not periodic
  ,AddSendPGN(SendUserDatumData,0,5000,82,false) // 129045 (6) Not periodic
  ,AddSendPGN(SendN2kGNSSDOPData,0,1000,37,false) // 129539
  ,AddSendPGN(SendN2kGNSSSatsInView,0,1000,37,false) // 129540 (max. 32)
  ,AddSendPGN(SendOutsideEnvironmentalParameters,0,500,38,true) // 130310
  ,AddSendPGN(SendEnvironmentalParameters,0,500,39,true) // 130311
  ,AddSendPGN(SendN2kTemperature,0,2000,40,true) // 130312
  ,AddSendPGN(SendN2kHumidity,0,2000,41,true) // 130313
  ,AddSendPGN(SendN2kPressure,0,2000,42,true) // 130314
  ,AddSendPGN(SendN2kSetPressure,0,5000,43,true) // 130315 Not periodic
  ,AddSendPGN(SendN2kTemperatureExt,0,2500,44,true) // 130316 Not periodic
  ,AddSendPGN(SendTideStationData,0,1000,86,false) // 130320 (8)
//  ,AddSendPGN(SendSalinity,0,1000,90,false) // 130321 (8)
  ,AddSendPGN(SendN2kMaretronTempHR,0,2000,45,false) // 130823 (2)
  ,AddSendPGN(SendN2kWind,0,1000,45,true) // 130823 (2)
  ,AddSendPGN(SendN2kRateOfTurn,0,1000,48,true) // 127251 (2)
  ,AddSendPGN(SendN2kMagneticVariation,0,10000,49,true) // 127258 (2)
  ,AddSendPGN(SendN2kFluidLevel,0,10000,80,true) // 127505
  ,AddSendPGN(SendN2kSpeed,0,1000,81,true)// 128259
  ,AddSendPGN(SendN2kWaterDepth,0,1000,82,true)// 128267
  ,AddSendPGN(SendN2kCrossTrackError,0,5000,82,true)// 129283
  ,AddSendPGN(SendN2k126720,0,5000,82,true)// 126720
  ,AddSendPGN(SendN2k127237,0,5000,82,true)// 127237
  ,AddSendPGN(SendN2k130916,0,5000,82,true)// 130916
  ,AddSendPGN(SendN2k65359,0,5000,82,true)// 65359
  ,AddSendPGN(SendN2k65379,0,5000,82,true)// 65379
  ,AddSendPGN(SendN2k65384,0,5000,82,true)// 65384
};

size_t nN2kSendMessages=sizeof(N2kSendMessages)/sizeof(tN2kSendMessage);

// *****************************************************************************
// Command handling functions. These are environment functions for example not
// for how to use library.

// *****************************************************************************
void CheckLoopTime() {
#define LoopTimePeriod 1000
  static unsigned long NextCheck=millis()+LoopTimePeriod ;
  static unsigned long AvgCount=0;
  static float AvgSum=0;
  static unsigned long MaxLoopTime=0;
  static unsigned long StartTime=micros();

  unsigned long UsedTime=micros()-StartTime;
  if ( UsedTime>MaxLoopTime ) MaxLoopTime=UsedTime;
  AvgCount++;
  AvgSum+=UsedTime;

  if ( NextCheck<millis() ) {
    NextCheck=millis()+LoopTimePeriod;
    Serial.print("- Loop times max:"); 
    Serial.print(MaxLoopTime); 
    Serial.print(" us, avg:"); 
    Serial.print(AvgSum/AvgCount); 
    Serial.println(" us");
    MaxLoopTime=0;
    AvgSum=0;
    AvgCount=0;
  }

  StartTime=micros();
}

#ifndef LONG_MAX
#define LONG_MAX 0x7fffffff
#endif

// *****************************************************************************
bool IsTimeToUpdate(unsigned long NextUpdate) {
  return ( millis()-NextUpdate<LONG_MAX );
}

// *****************************************************************************
unsigned long RemainingToUpdate(unsigned long NextUpdate) {
  unsigned long Diff=NextUpdate-millis();
  return ( Diff<LONG_MAX?0:Diff);
}

// *****************************************************************************
unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset=0) {
  return millis()+Period+Offset;
}

// *****************************************************************************
void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period, unsigned long Offset) {
  unsigned long Now=millis();
  if ( Now>Offset ) {
    Now-=Offset;
    NextUpdate=Now+(Period-Now%Period)+Offset;
  } else { NextUpdate=Period+Offset; }
}

// *****************************************************************************
void tN2kSendMessage::SetNextUpdate()  {
  ::SetNextUpdate(NextSend,Period,Offset);
}

// *****************************************************************************
void tN2kSendMessage::Enable(bool state)  {
  if ( Enabled!=state ) {
    Enabled=state;
    if ( state ) SetNextUpdate();
  }
}

using tCommandFunction=void (*)(const char *Params);

struct tCommand {
  const char *const Command;
  const char *const Usage;
  const char *const Description;
  tCommandFunction CommandFunction;
};

// *****************************************************************************
void ToggleStats(const char *) {
  ShowStatistics=!ShowStatistics;
  if ( ShowStatistics ) SetNextUpdate(NextStatsTime,StatisticsPeriod);
}

// *****************************************************************************
void ToggleSending(const char *) {
  Sending=!Sending;
  Serial.print("Message sending ");
  Serial.println(Sending?"enabled":"disabled");
}

// *****************************************************************************
void ToggleShowSentMessages(const char *) {
  ShowSentMessages=!ShowSentMessages;
}

// *****************************************************************************
void ToggleForward(const char *) {
  EnableForward=!EnableForward;
  NMEA2000.EnableForward(EnableForward);

  Serial.print("Show bus messages ");
  Serial.println(EnableForward?"enabled":"disabled");
}

// *****************************************************************************
void ClearStatistics(const char *) {
  N2kMsgSentCount=0;
  N2kMsgFailCount=0;
  Serial.println("Message statistics cleared");
}

// *****************************************************************************
void ToggleMsgSending(const char *Params) {
  if ( Params==0 ) return;

  size_t MsgIndex=atoi(Params);
  if ( MsgIndex<nN2kSendMessages ) {
    N2kSendMessages[MsgIndex].Enable(!N2kSendMessages[MsgIndex].Enabled);
    Serial.print(N2kSendMessages[MsgIndex].Description);
    Serial.print("  ");
    Serial.println(N2kSendMessages[MsgIndex].Enabled?"enabled":"disabled");
  }
}

// *****************************************************************************
void EnableAllMessages(const char *) {
  for ( size_t i=0; i<nN2kSendMessages; i++ ) {
    N2kSendMessages[i].Enable(true);
  }
  Serial.println("All messages enabled");
}

// *****************************************************************************
void DisableAllMessages(const char *) {
  for ( size_t i=0; i<nN2kSendMessages; i++ ) {
    N2kSendMessages[i].Enable(false);
  }
  Serial.println("All messages disabled");
}

// *****************************************************************************
void ListMessages(const char *) {
  for ( size_t i=0; i<nN2kSendMessages; i++ ) {
    Serial.print("    "); 
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(N2kSendMessages[i].Enabled?"(enabled)  ":"(disabled) ");
    Serial.println(N2kSendMessages[i].Description);
  }
}

void ListCommands(const char *Command);

tCommand Commands[]={
  {"?",0,"List commands",ListCommands}
  ,{"p",0,"Toggle sending",ToggleSending}
  ,{"stat",0,"Toggle stat printing",ToggleStats}
  ,{"cl",0,"Clear statistics",ClearStatistics}
  ,{"e",0,"Enable all messages",EnableAllMessages}
  ,{"d",0,"Disable all messages",DisableAllMessages}
  ,{"s",0,"Toggle show sent messages",ToggleShowSentMessages}
  ,{"fwd",0,"Toggle show other bus messages",ToggleForward}
  ,{"msgs",0,"List messages",ListMessages}
  ,{"msg","msg x","Toggle message x sending",ToggleMsgSending}
};

size_t nCommands=sizeof(Commands)/sizeof(tCommand);

// *****************************************************************************
void ListCommands(const char *Command) {
  Serial.println();
  Serial.println("Commands:");
  char Info[200];
  for ( size_t i=0; i<nCommands; i++ ) {
    strcpy(Info,"  ");
    strcat(Info,Commands[i].Usage!=0?Commands[i].Usage:Commands[i].Command);
    size_t len=strlen(Info);
    if ( len<16 ) {
      memset(Info+len,' ',16-len);
      Info[16]=0;
    }
    strcat(Info,"- ");
    strcat(Info,Commands[i].Description);
    Serial.println(Info);
  }
  Serial.println();
}

// *****************************************************************************
void CheckCommand() {
  static char Command[20]={0};
  static size_t CommandPos=0;

  if ( Serial.available() ) {
    char ch=Serial.read();
    if ( ch=='\n') {
      char *Params=strchr(Command,' ');
      if ( Params!=0 ) {
        
        *Params=0;
        Params++;
      }
      for ( size_t i=0; i<nCommands; i++ ) {
        if ( strcmp(Command,Commands[i].Command)==0 ) {
          Commands[i].CommandFunction(Params);
          break;
        }
      }
      CommandPos=0;
      Command[CommandPos]=0;
    } else {
      if ( ch>=' ' && CommandPos+1<sizeof(Command) ) {
        Command[CommandPos]=ch;
        CommandPos++;
        Command[CommandPos]=0;
      }
    }
  }
}
