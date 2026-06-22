/*
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// NMEA2000 Man over board button
// Version 1.0, 19.04.2023, buhhe (https://github.com/buhhe)
// https://github.com/buhhe/NMEA2000-Man-Over-Board-Button
// Libraries needed:
//        - https://github.com/ttlappalainen/NMEA2000
//        - https://github.com/ttlappalainen/NMEA2000_esp32
//        - https://github.com/ttlappalainen/NMEA0183
//
//

#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5 
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4

#include <Arduino.h>
#include <Preferences.h>
#include <NMEA2000_CAN.h>  
#include <N2kMessages.h>

// to be printed out to USB-serial
const char Description[] = "MOB-Alarm Button. Comes without any warranty or reliability. Only for test purposes!";
// Vor dem Aufruf von SendN2kMOBAlarm() hinzufügen:
const char* mobText = "MOB ALARM";  // oder dein gewünschter Text

#define ALARM_BUTTON    13          // GPIO pin to be connected to GND when the alarm button is pressed
#define MYMMSI          211311370   // set your MMSI in here
#define ACTIVATION_TIME  5          // seconds the button has to be pressed to activate the alarm
#define AIS_MOB_MMSI    972311370   // 972 + last 7 digits of MYMMSI (123456789 → 2345678 → 972234567... adjust to your MMSI)

int NodeAddress;            // To store last Node Address
Preferences preferences;    // Nonvolatile storage on ESP32 - To store LastDeviceAddress

// structure which contains all data needed to set the mob alarm
struct PGN127233
{
  unsigned char                     SID;                      // sequence id
  uint32_t                          EmitterID;                // Identifier for each MOB emitter, unique to the vessel
  tN2kMOBStatus                     MOBStatus;                // MOBStatus: MOBEmitterActivated=0, ManualOnBoardMOBButtonActivation=1, TestMode=2, MOBNotActive=3
  double                            ActivationTime;           // Time of day (UTC) when MOB was activated
  tN2kMOBPositionSource             PositionSource;           // Position Source: PositionEstimatedByVessel=0,PositionReportedByMOBEmitter=1 
  uint16_t                          DateOfMobPosition;        // Date of MOB position
  double                            TimeOfMobPosition;        // Time of day of MOB position (UTC)
  double                            LatitudeOfMob;            // Latitude in degrees
  double                            LongitudeOfMob;           // Longitude in degrees
  tN2kHeadingReference              COGReference;             // True or Magnetic: N2khr_true=0, N2khr_magnetic=1, N2khr_error=2, N2khr_Unavailable=3
  double                            COG;                      // Course Over Ground in radians
  double                            SOG;                      // Speed Over Ground in m/s
  uint32_t                          MMSI;                     // MMSI
  tN2kMOBEmitterBatteryStatus       MOBEmitterBatteryStatus;  // Battery status: Good=0, Low=1
} ;


PGN127233 PGNOut;

// Set the information for other devices on the bus which messages we support
const unsigned long  ReceiveMessages[] PROGMEM = { 129029L, 129026L, 0};  // get navigational data
const unsigned long TransmitMessages[] PROGMEM = {127233L, 129038L, 129802L, 0};           // send man over board alarm

// forward declarations
void          SayHello(void);           
void          SendN2kMOBAlarm(void);
void          CheckSourceAddressChange(void);
void          MyParsePGN129029(const tN2kMsg);
void          MyParsePGN129026(const tN2kMsg);
void          MyHandleNMEA2000Msg(const tN2kMsg &);
void          DispMessage();
void          handleSerialInput();
void          HandleNMEA2000Msg(const tN2kMsg &N2kMsg);
void          SendRaymarineMOB(bool); 

//*****************************************************************************
void setup()
{
  uint8_t chipid[6];
  uint32_t id = 0;
  int i = 0, j = 0; 

 // DeviceAddress tempDeviceAddress;

  // Init USB serial port
  Serial.begin(115200);
  delay(400);


  SayHello();  // print some useful information to USB-serial

  /*   NMEA2000 initialisation section */
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(150);
  NMEA2000.SetN2kCANSendFrameBufSize(150);

  // Generate unique number from chip id
  esp_efuse_mac_get_default(chipid);
  for (i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

// set some static information 
  PGNOut.SID          = 0xff;
  PGNOut.EmitterID    = 1;
  PGNOut.MOBStatus    = ManualOnBoardMOBButtonActivation;   // MOBEmitterActivated=0,ManualOnBoardMOBButtonActivation=1, TestMode=2, MOBNotActive=3
  PGNOut.MMSI         = MYMMSI;
  PGNOut.MOBEmitterBatteryStatus = Good;



  // Set product information
  NMEA2000.SetProductInformation("1", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "MOB-Alarm Button",  // Manufacturer's Model ID
                                 "SW-Vers:  1.0 (2023-04-19)",  // Manufacturer's Software version code
                                 "Mod-Vers: 1.0 (buhhe)" // Manufacturer's Model version
                                );
  // Set device information
  NMEA2000.SetDeviceInformation(id, // Unique number. Use e.g. Serial number.
                                135, // Device function= Man Overboard detection/reporting. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                20,  // Device class=Safety Systems. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
  NodeAddress = preferences.getInt("LastNodeAddress", 36);  // Read stored last NodeAddress, default 34
  preferences.end();
  Serial.printf("NodeAddress=%d\n", NodeAddress);

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.SetMsgHandler(MyHandleNMEA2000Msg);
  NMEA2000.Open();
  
  pinMode(ALARM_BUTTON, INPUT_PULLUP);  
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
}


//*****************************************************************************
void loop() 
{
  static unsigned long myTime;

  NMEA2000.ParseMessages();
  CheckSourceAddressChange();

  if (digitalRead(ALARM_BUTTON)==LOW)
  {
    myTime = millis();
    while ( digitalRead(ALARM_BUTTON) == LOW && (millis() < myTime + ACTIVATION_TIME*1000))
    {}
    if (digitalRead(ALARM_BUTTON)==LOW)
    {
      SendN2kMOBAlarm();
    }
  }

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if ( Serial.available() ) 
  {
    handleSerialInput();
    //Serial.read();
  }
}

//*****************************************************************************
void MyHandleNMEA2000Msg(const tN2kMsg &N2kMsg)   // callback function for incoming N2K messages
{
  switch (N2kMsg.PGN)
  {
    case 129029L: MyParsePGN129029(N2kMsg); 
                  break;
    case 129026L: MyParsePGN129026(N2kMsg); 
    default:      break;
  }
}


//********* retrieve some navigational data, PGN129029 *************************************
void MyParsePGN129029(const tN2kMsg N2kMsg) 
{
  unsigned char   SID;
  double          Altitude;
  tN2kGNSStype    GNSStype;
  tN2kGNSSmethod  GNSSmethod;
  unsigned char   nSatellites;
  double          HDOP;
  double          PDOP;
  double          GeoidalSeparation;
  unsigned char   nReferenceStations;
  tN2kGNSStype    ReferenceStationType;
  uint16_t        ReferenceSationID;
  double          AgeOfCorrection;

  ParseN2kPGN129029(N2kMsg, SID, PGNOut.DateOfMobPosition, PGNOut.TimeOfMobPosition, PGNOut.LatitudeOfMob, PGNOut.LongitudeOfMob, Altitude, GNSStype, GNSSmethod,
                     nSatellites, HDOP, PDOP, GeoidalSeparation, nReferenceStations, ReferenceStationType, ReferenceSationID, AgeOfCorrection);

  PGNOut.ActivationTime = PGNOut.TimeOfMobPosition;                 // Time of day (UTC) when MOB was activated
  PGNOut.PositionSource = (tN2kMOBPositionSource)0;                 // Position Source
  PGNOut.MOBEmitterBatteryStatus = (tN2kMOBEmitterBatteryStatus)0;
}

//********* retrieve some navigational data, PGN129026 *************************************
void MyParsePGN129026(const tN2kMsg N2kMsg)
{
  unsigned char SID;
  ParseN2kPGN129026(N2kMsg, SID, PGNOut.COGReference, PGNOut.COG, PGNOut.SOG);
}

//********* send the man overt board alarm to the N2K bus, PGN127233  **********************

void SendN2kMOBAlarm(void)
{
  tN2kMsg N2kMsg;
  int i;
    Serial.println("!!!MOB Alarm!!!");
    
    SetN2kPGN127233(N2kMsg, 
                          PGNOut.SID,                       // Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages to link this PGN to other related PGNs. When no linkage exists, the value of the SID shall be set to 25   
                          PGNOut.EmitterID,                 // Identifier for each MOB emitter, unique to the vessel
                          PGNOut.MOBStatus,                 // MOBEmitterActivated=0,ManualOnBoardMOBButtonActivation=1, TestMode=2, MOBNotActive=3
                          PGNOut.ActivationTime,            // Time of day (UTC) when MOB was activated
                          PGNOut.PositionSource,            // Position Source
                          PGNOut.DateOfMobPosition,         // Date of MOB position
                          PGNOut.TimeOfMobPosition,         // time of day UTC, get from N2K  
                          PGNOut.LatitudeOfMob,             // Latitude in degrees
                          PGNOut.LongitudeOfMob,            // Longitude in degrees
                          PGNOut.COGReference,
                          PGNOut.COG,
                          PGNOut.SOG,
                          PGNOut.MMSI,
                          PGNOut.MOBEmitterBatteryStatus);
                                          
    NMEA2000.SendMsg(N2kMsg);

  Serial.println("  -> PGN 127233 sent");
 
  // --- PGN 129038: AIS Class A Position Report ---
  // Library signature:
  //   SetN2kAISClassAPosition(N2kMsg, MessageID, Repeat, UserID,
  //     Latitude, Longitude, Accuracy, RAIM, Seconds,
  //     COG, SOG, AISTransceiverInformation, Heading, ROT, NavStatus)
  // NavStatus N2kaisns_AIS_SART_is_active (=14) is what triggers MOB on Raymarine Axiom
  SetN2kAISClassAPosition(N2kMsg,
                          1,                              // MessageID: AIS message type 1
                          N2kaisr_First,                  // Repeat: not repeated
                          AIS_MOB_MMSI,                   // UserID: 972311370
                          PGNOut.LatitudeOfMob,
                          PGNOut.LongitudeOfMob,
                          true,                           // Accuracy: high
                          false,                          // RAIM: not active
                          60,                             // Seconds: 60 = not available
                          PGNOut.COG,
                          PGNOut.SOG,
                          N2kaischannel_A_VDL_reception,  // AISTransceiverInformation
                          N2kDoubleNA,                    // Heading: not available
                          N2kDoubleNA,                    // ROT: not available
                          N2kaisns_AIS_SART);   // NavStatus 14 = MOB/SART active
  NMEA2000.SendMsg(N2kMsg);
  Serial.println("  -> PGN 129038 (AIS NavStatus=14) sent");
 
  // --- PGN 129802: AIS Safety Related Broadcast Message ---
  // Library signature:
  //   SetN2kAISSafetyRelatedBroadcastMsg(N2kMsg, MessageID, Repeat,
  //     SourceID, AISTransceiverInformation, SafetyRelatedText)
  SetN2kAISSafetyRelatedBroadcastMsg(N2kMsg,
                                     14,                             // MessageID: AIS msg type 14
                                     N2kaisr_First,                  // Repeat: not repeated
                                     AIS_MOB_MMSI,                   // SourceID
                                     N2kaischannel_A_VDL_reception,  // AISTransceiverInformation
                                     mobText);                       // SafetyRelatedText
  NMEA2000.SendMsg(N2kMsg);
  Serial.println("  -> PGN 129802 (AIS Safety 'MOB ALARM') sent");
 
  delay(1000);
  Serial.println("MOB-Alarm sent!");
    
}

//*****************************************************************************
// Function to check if SourceAddress has changed (due to address conflict on bus)
void CheckSourceAddressChange()
{
  int SourceAddress = NMEA2000.GetN2kSource();

  if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory
    NodeAddress = SourceAddress;      // Set new Node Address (to save only once)
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }
}

//*****************************************************************************
void SayHello()
{
  char Sketch[80], buf[256], Version[80];
  int i;

  // Get source code filename
  strcpy(buf, __FILE__);
  i = strlen(buf);

  // remove path and suffix
  while (buf[i] != '.' && i >= 0)
    i--;
  buf[i] = '\0';
  while ( buf[i] != '\\' && i >= 0)
    i--;
  i++;
  strcpy(Sketch, buf + i);

  // Sketch date/time of compliation
  sprintf(buf, "\nSketch: \"%s\", compiled %s, %s\n", Sketch, __DATE__, __TIME__);
  Serial.println(buf);
  Serial.println(Description);

  Serial.println("SGS MOB System bereit.");
  Serial.println("Befehle: -h (Hilfe), -a (MOB Alarm ausloesen)");
}

String serialBuffer = "";

// Neue Funktion für Serial-CLI:
void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      serialBuffer.trim();
      
      if (serialBuffer == "-h") {
        Serial.println("=== SGS MOB CLI ===");
        Serial.println("-h   Hilfe anzeigen");
        Serial.println("-a   MOB Alarm ausloesen");
      } 
      else if (serialBuffer == "-a") {
        Serial.println(">>> MOB ALARM wird ausgeloest!");
        SendN2kMOBAlarm();  // deine bestehende Funktion
      }
      else if (serialBuffer == "-r") {
        SendRaymarineMOB(true);
      }
      else if (serialBuffer == "-d") {
        SendRaymarineMOB(false);
        Serial.println("Befehle: -h (Hilfe), -a (MOB SART an), -r (MOB Ray an) -d (MOB aus)");
      }
      else if (serialBuffer.length() > 0) {
        Serial.println("Unbekannter Befehl. '-h' fuer Hilfe.");
      }
      
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}


void DispMessage()
{
  Serial.printf("\n\nSID:%d \nEmitterID:%d \nMOBStatus:%d \nActivationTime:%f \nPositionSource:%d \nDateOfMobPosition:%ld \nTimeOfMobPosition: %f \nLatitudeOfMob:%f \nLongitudeOfMob%f \nCOGReference:%d \nCOG: %.2f \nSOG:%.2f \nMMSI:%d \nMOBEmitterBatteryStatus%d\n",
                            PGNOut.SID,                       // Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages to link this PGN to other related PGNs. When no linkage exists, the value of the SID shall be set to 25   
                            PGNOut.EmitterID,                 // Identifier for each MOB emitter, unique to the vessel
                            PGNOut.MOBStatus,                 // MOBEmitterActivated=0,ManualOnBoardMOBButtonActivation=1, TestMode=2, MOBNotActive=3
                            PGNOut.ActivationTime,            // Time of day (UTC) when MOB was activated
                            PGNOut.PositionSource,            // Position Source
                            PGNOut.DateOfMobPosition,         // Date of MOB position
                            PGNOut.TimeOfMobPosition,         // time of day UTC, get from N2K  
                            PGNOut.LatitudeOfMob,             // Latitude in degrees
                            PGNOut.LongitudeOfMob,            // Longitude in degrees
                            PGNOut.COGReference,
                            PGNOut.COG,
                            PGNOut.SOG,
                            PGNOut.MMSI,
                            PGNOut.MOBEmitterBatteryStatus);
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  
  return; 
const unsigned long ignoredPGNs[] = {
    // Bereits gefiltert
    129025,  127250,  127251,  127257,  127245,
    130306,  126208,  126720,  65359,   128259,
    130311,  128267,  128275,  127258,  65379,
    65384,   129026,  127237,  129038,  129039,
    129033,  129029,  129540,  129542,  126992,
    129284,  129285,  129283,  130577,  130848,
    130918,  130916,  65520,   129291,  61184,
    65396,

    // NEU hinzufügen
    129794,  // AIS Class A Static & Voyage (Dauerstrom)
    129809,  // AIS Class B CS Static Data Part A
    129810,  // AIS Class B CS Static Data Part B
    129793,  // AIS UTC/Date Report
    130934,  // Proprietär AIS
    130935,  // Proprietär AIS
    129044,  // Datum
    65311,   // Proprietär
    65535,   // Proprietär

    0        // Ende der Liste - IMMER als letztes lassen!
};
  // Prüfen ob PGN gefiltert werden soll
  for (int i = 0; ignoredPGNs[i] != 0; i++) {
    if (N2kMsg.PGN == ignoredPGNs[i]) return;
  }

  // Ausgabe nur für nicht gefilterte PGNs
  Serial.print("PGN: ");
  Serial.print(N2kMsg.PGN);
  Serial.print("  Src: ");
  Serial.print(N2kMsg.Source);
  Serial.print("  Len: ");
  Serial.print(N2kMsg.DataLen);
  Serial.print("  Data: ");
  for (int i = 0; i < N2kMsg.DataLen; i++) {
    if (N2kMsg.Data[i] < 0x10) Serial.print("0");
    Serial.print(N2kMsg.Data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}


void SendRaymarineMOB(bool activate) {
  tN2kMsg N2kMsg;

  // Nachricht 1
  N2kMsg.Init(6, 65288, 36, 255);
  N2kMsg.AddByte(0x3B); N2kMsg.AddByte(0x9F);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(activate ? 0x01 : 0x00);
  N2kMsg.AddByte(0x26); N2kMsg.AddByte(0x03);
  N2kMsg.AddByte(0x00); N2kMsg.AddByte(0xC0);
  NMEA2000.SendMsg(N2kMsg);

  // Nachricht 2
  N2kMsg.Init(6, 65288, 36, 255);
  N2kMsg.AddByte(0x3B); N2kMsg.AddByte(0x9F);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(activate ? 0x01 : 0x00);
  N2kMsg.AddByte(0x72); N2kMsg.AddByte(0x03);
  N2kMsg.AddByte(0x00); N2kMsg.AddByte(0xC0);
  NMEA2000.SendMsg(N2kMsg);

  // Nachricht 3 - nur beim Aktivieren
  if (activate) {  
    N2kMsg.Init(6, 65361, 36, 255);
    N2kMsg.AddByte(0x3B); N2kMsg.AddByte(0x9F);
    N2kMsg.AddByte(0x26); N2kMsg.AddByte(0x03);
    N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);
    N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);
    NMEA2000.SendMsg(N2kMsg);
  }

  Serial.println(activate ? ">>> MOB ALARM AKTIVIERT!" : ">>> MOB ALARM DEAKTIVIERT!");
}