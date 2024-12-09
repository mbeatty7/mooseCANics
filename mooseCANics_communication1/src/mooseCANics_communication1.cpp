/* 
 * Project mooseCANics communication
 * Author: Mamie-Jo Beatty
 * Date: 12/2/24
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */


#include "Particle.h"
#include "mcp_can.h"
#include "SPI.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"

//SYSTEM_THREAD(ENABLED);
//SerialLogHandler logHandler;

const int OLED_RESET = -1;

unsigned long rxId;
byte dlc;
byte rxBuf[8];

unsigned char len = 0;
char msgString[128];                        // Array to store serial string



#define CAN0_INT A1                              // Set INT to pin A1
#define CAN1_INT A1

#define PAD 0x00

#define standard 1

#if standard == 1
  #define REPLY_ID 0x7E8
  #define LISTEN_ID 0x7E0
  #define FUNCTIONAL_ID 0x7DF  
#else
  #define REPLY_ID 0x98DAF101
  #define LISTEN_ID 0x98DA01F1
  #define FUNCTIONAL_ID 0x98DB33F1
#endif

MCP_CAN CAN1(A0);                                // Set CS to pin A0
//MCP_CAN CAN0(A2);                               // Set CS to pin A2

// ADD IN //
const pin_t RPM_PIN = A0;
const int MAX_RPM = 8000;
const int IDLE_RPM = 1500;

void obdReq(byte *data);
void unsupported(byte mode, byte pid);
void negAck(byte mode, byte reason);
void unsupportedPrint(byte mode, byte pid);
void iso_tp(byte mode, byte pid, int len, byte *data);


unsigned long lastRequest = 0;
Adafruit_SSD1306 display(OLED_RESET);

const uint8_t SERVICE_CURRENT_DATA = 0X01;

const uint32_t OBD_CAN_REQUEST_ID =  0x7DF;
const uint32_t OBD_CAN_REPLY_ID = 0x7E8;

const uint8_t PID_ENGINE_RPM = 0x0C;
const uint8_t PID_VEHICLE_SPEED = 0x0D;


//byte data[8] = {0x02, SERVICE_CURRENT_DATA, PID_ENGINE_RPM, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc};

void setup()
{
  Serial.begin(115200);

  // Wait 10 seconds for USB debug serial to be connected (plus 1 more)
  waitFor(Serial.isConnected, 10000);
  delay(1000);

  // OLED DISPLAY CODE //

  display.begin();
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1,0);
  display.setRotation(2);
  display.setCursor(0,0);
  display.display();

  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  //***** CAN 0 *****//
  // if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK){
  //   Serial.printf("MCP2515 0 Initialized Successfully!");
  //   display.printf("MCP2515 0 Initialized Successfully!");
  //   display.display();
  // }
  // else
  //   Serial.printf("Error Initializing MCP2515 0...");
  //   display.printf("Error Initializing MCP2515 0...");
  //   display.display();
  
  // CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  // pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  
  // Serial.println("MCP2515 0 Library Receive Example...");


//***** CAN 1 *****//
   if(CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK){
    Serial.printf("MCP2515 1 Initialized Successfully!");
    display.printf("MCP2515 1 Initialized Successfully!");
    display.display();
   }
  else
    Serial.printf("Error Initializing MCP2515 1...");
    display.printf("Error Initializing MCP2515 1...");
    display.display();
  
  CAN1.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN1_INT, INPUT);                            // Configuring pin for /INT input
  
  Serial.println("MCP2515 1 Library Receive Example...");
  display.printf("MCP2515 1 Library Receive Example...");
  display.display();
  delay(1000);
}

// byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
byte data[8] = {0x02, SERVICE_CURRENT_DATA, PID_ENGINE_RPM, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc};

void loop()
{
   ///// SEND CODE FOR CAN 0 /////

      //***** CAN 0 *****//
     // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN1.sendMsgBuf(0x100, 0, 8, data);
  if(sndStat == CAN_OK){
    Serial.println("Message Sent Successfully!");
    display.printf("Message Sent Successfully!");
    display.display();
  } else {
    Serial.printf("Error Sending Message %d", sndStat);
    display.printf("Error Sending Message %d", sndStat);
    display.display();
  }
  delay(100);   // send data per 100ms


    /////// RECEIVE CODE FOR CAN 1 ///////

    //***** CAN 1 *****//

   if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN1.readMsgBuf(&rxId, &dlc, rxBuf);             // Get CAN data
    
    // First request from most adapters...
    if(rxId == FUNCTIONAL_ID){
      Log.info("got request id=0x%x", rxId);
      obdReq(rxBuf);
    }       
  }

  if(!digitalRead(CAN1_INT))                         // If CAN1_INT pin is low, read receive buffer
  {
    CAN1.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    
    if((rxId & 0x80000000) == 0x80000000) {    // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
      display.printf(msgString, "Extended ID: 0x%.8lX DLC: %1d Data:", (rxId & 0x1FFFFFFF), len);
      display.display();
    }
    else 
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
      display.printf(msgString, "Standard ID: 0x%.3lX     DLC: %1d  Data:", rxId, len);
      display.display();
    Serial.print(msgString);
  
    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      char *cp = msgString;
      cp += sprintf(cp, "Data: ");
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
        
    Serial.println();
  }

  if (millis()- lastRequest >= 1000) {
    lastRequest = millis();

    byte sndStat = CAN1.sendMsgBuf(OBD_CAN_REQUEST_ID, 0, 8, data);
    if(sndStat == CAN_OK){
      Serial.printf("Message Sent Successfully!");
    }
      else{
        Serial.printf("Error Sending Message %d", sndStat);
      }
    }
  }

void obdReq(byte *data){
  byte numofBytes = data[0];
  byte mode = data[1] & 0x0F;
  byte pid = data[2];
  bool tx = false;
  byte txData[] = {0x00,(0x40 | mode),pid,PAD,PAD,PAD,PAD,PAD};



if(mode == 0x01){
    if(pid == 0x00){        // Supported PIDs 01-20
      txData[0] = 0x06;
      
      txData[3] = 0x80;
      txData[4] = 0x38;
      txData[5] = 0x00;
      txData[6] = 0x01;
      tx = true;
    }
    else if(pid == 0x01){    // Monitor status since DTs cleared.
      bool MIL = true;
      byte DTC = 5;
      txData[0] = 0x06;
      
      txData[3] = (MIL << 7) | (DTC & 0x7F);
      txData[4] = 0x07;
      txData[5] = 0xFF;
      txData[6] = 0x00;
      tx = true;
    }
else if(pid == 0x03){    // Fuel system status
      txData[0] = 0x03;
      
      txData[3] = 0xFA;
      tx = true;
    }
//    else if(pid == 0x04){    // Calculated engine load
//    }
    else if(pid == 0x05){    // Engine coolant temperature
      txData[0] = 0x03;
      
      txData[3] = 0xFA;
      tx = true;
    }
else if(pid == 0x0B){    // Intake manifold absolute pressure
      txData[0] = 0x03;
      
      txData[3] = 0x64;
      tx = true;
    }
    else if(pid == 0x0C){    // Engine RPM
      txData[0] = 0x04;

      int value = analogRead(RPM_PIN);

      // Value is now 0-4095 (12-bit ADC)
      
      // Map to 0-MAX_RPM
      int rpm = value * MAX_RPM / 4095;

      //
      if (value < 50) {
          rpm = 0;
      }
      else
      if (rpm < IDLE_RPM) {
          rpm = IDLE_RPM;
      }
      rpm *= 4;

      txData[3] = (byte)(rpm >> 8);
      txData[4] = (byte)rpm;
      tx = true;
    }
    else if(pid == 0x0D){    // Vehicle speed
      txData[0] = 0x03;
      
      txData[3] = 0xFA;
      tx = true;
    }
//    else if(pid == 0x0E){    // Timing advance
//    }
    else if(pid == 0x0F){    // Intake air temperature
      txData[0] = 0x03;
      
      txData[3] = 0xFA;
      tx = true;
    }
    else if(pid == 0x11){    // Throttle position
      txData[0] = 0x03;
      
      txData[3] = 0xFA;
      tx = true;
    }

    else if(pid == 0x20){    // Supported PIDs 21-40
      txData[0] = 0x06;
      
      txData[3] = 0x80;
      txData[4] = 0x00;
      txData[5] = 0x00;
      txData[6] = 0x01;
      tx = true;
    }
    else if(pid == 0x21){    // Distance traveled with MIL on
      txData[0] = 0x04;
      
      txData[3] = 0x00;
      txData[4] = 0x23;
      tx = true;
    }

    else if(pid == 0x40){    // Supported PIDs 41-60
      txData[0] = 0x06;
      
      txData[3] = 0x00;
      txData[4] = 0x08;
      txData[5] = 0x00;
      txData[6] = 0x0D;
      tx = true;
    }
     else if(pid == 0x4D){    // Time run with MIL on
      txData[0] = 0x04;
      
      txData[3] = 0x00;
      txData[4] = 0x3C;
      tx = true;
    }

    else if(pid == 0x5C){    // Engine oil Temperature
      txData[0] = 0x03;
      
      txData[3] = 0x1E;
      tx = true;
    }
    else if(pid == 0x5D){    // Fuel injection timing
      txData[0] = 0x04;
      
      txData[3] = 0x61;
      txData[4] = 0x80;
      tx = true;
    }
    else if(pid == 0x5E){    // Engine fuel rate
      txData[0] = 0x04;
      
      txData[3] = 0x07;
      txData[4] = 0xD0;
      tx = true;
    }
//    else if(pid == 0x5F){    // Emissions requirements to which vehicle is designed
//    }
    else if(pid == 0x60){    // Supported PIDs 61-80
      txData[0] = 0x06;
      
      txData[3] = 0x00;
      txData[4] = 0x00;
      txData[5] = 0x00;
      txData[6] = 0x01;
      tx = true;
    }
    else if(pid == 0x80){    // Supported PIDs 81-A0
      txData[0] = 0x06;
      
      txData[3] = 0x00;
      txData[4] = 0x00;
      txData[5] = 0x00;
      txData[6] = 0x01;
      tx = true;
    }

    else if(pid == 0xA0){    // Supported PIDs A1-C0
      txData[0] = 0x06;
      
      txData[3] = 0x00;
      txData[4] = 0x00;
      txData[5] = 0x00;
      txData[6] = 0x01;
      tx = true;
    }
    else if(pid == 0xC0){    // Supported PIDs C1-E0
      txData[0] = 0x06;
      
      txData[3] = 0x00;
      txData[4] = 0x00;
      txData[5] = 0x00;
      txData[6] = 0x01;
      tx = true;
    }
    else if(pid == 0xE0){    // Supported PIDs E1-FF?
      txData[0] = 0x06;
      
      txData[3] = 0x00;
      txData[4] = 0x00;
      txData[5] = 0x00;
      txData[6] = 0x00;
      tx = true;
    }
    else{
      unsupported(mode, pid);
    }
    if(tx) {
    CAN1.sendMsgBuf(REPLY_ID, 8, txData);
    Log.info("sending reply %02x %02x %02x %02x %02x %02x %02x %02x", 
      txData[0], txData[1], txData[2], txData[3], txData[4], txData[5], txData[6], txData[7]);
    }

}
}

// Generic debug serial output
void unsupported(byte mode, byte pid){
  negAck(mode, 0x12);
  unsupportedPrint(mode, pid);  
}


// Generic debug serial output
void negAck(byte mode, byte reason){
  byte txData[] = {0x03,0x7F,mode,reason,PAD,PAD,PAD,PAD};
  CAN1.sendMsgBuf(REPLY_ID, 8, txData);
}


// Generic debug serial output
void unsupportedPrint(byte mode, byte pid){
  Log.error("Mode $%02X: Unsupported PID $%02X requested!", mode, pid);
}


// Blocking example of ISO transport
void iso_tp(byte mode, byte pid, int len, byte *data){
  byte tpData[8];
  int offset = 0;
  byte index = 0;
//  byte packetcnt = ((len & 0x0FFF) - 6) / 7;
//  if((((len & 0x0FFF) - 6) % 7) > 0)
//    packetcnt++;

  // First frame
  tpData[0] = 0x10 | ((len >> 8) & 0x0F);
  tpData[1] = 0x00FF & len;
  for(byte i=2; i<8; i++){
    tpData[i] = data[offset++];
  }
  CAN1.sendMsgBuf(REPLY_ID, 8, tpData);
  index++; // We sent a packet so increase our index.
  
  bool not_done = true;
  unsigned long sepPrev = millis();
  byte sepInvl = 0;
  byte frames = 0;
  bool lockout = false;
  while(not_done){
    // Need to wait for flow frame
    if(!digitalRead(CAN0_INT)){
      CAN1.readMsgBuf(&rxId, &dlc, rxBuf);
    
      if((rxId == LISTEN_ID) && ((rxBuf[0] & 0xF0) == 0x30)){
        if((rxBuf[0] & 0x0F) == 0x00){
          // Continue
          frames = rxBuf[1];
          sepInvl = rxBuf[2];
          lockout = true;
        } else if((rxBuf[0] & 0x0F) == 0x01){
          // Wait
          lockout = false;
          delay(rxBuf[2]);
        } else if((rxBuf[0] & 0x0F) == 0x03){
          // Abort
          not_done = false;
          return;
        }
      }
    }

    if(((millis() - sepPrev) >= sepInvl) && lockout){
      sepPrev = millis();

      tpData[0] = 0x20 | index++;
      for(byte i=1; i<8; i++){
        if(offset != len)
          tpData[i] = data[offset++];
        else
          tpData[i] = 0x00;
      }
      
      // Do consecutive frames as instructed via flow frame
      CAN1.sendMsgBuf(REPLY_ID, 8, tpData);
      
      if(frames-- == 1)
        lockout = false;
        
    }

    if(offset == len)
      not_done = false;
    else{
      Log.info("Offset: 0x%04X  Len: 0x%04X", offset, len);
    }


    // Timeout
    if((millis() - sepPrev) >= 1000)
      not_done = false;
  }
}