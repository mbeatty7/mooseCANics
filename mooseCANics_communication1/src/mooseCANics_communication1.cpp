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
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"



TCPClient TheClient;

const int OLED_RESET = -1;

int status;
long unsigned int rxId;
long unsigned int wheelPosition;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string



#define CAN0_INT A1                              // Set INT to pin A1
#define CAN1_INT A1

MCP_CAN CAN1(A0);                                // Set CS to pin A0
MCP_CAN CAN0(A2);                               // Set CS to pin A2

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

// feeds 
Adafruit_MQTT_Publish brakepedalFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/brakepedal");
Adafruit_MQTT_Publish SIDFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/0x224");
Adafruit_MQTT_Publish steeringwheelFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/steeringwheel");


unsigned int last, lastTime, wheelTime;

void MQTT_connect();
bool MQTT_ping();

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);

SYSTEM_MODE(AUTOMATIC);


void setup()
{
  Serial.begin(115200);

  // Wait 10 seconds for USB debug serial to be connected (plus 1 more)
  waitFor(Serial.isConnected, 10000);
  delay(1000);

  // Connect to Internet but not Particle Cloud //
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  Serial.printf("\n\n");


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
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK){
    Serial.println("MCP2515 0 Initialized Successfully!");
    display.printf("MCP2515 0 Initialized Successfully!");
    display.display();
  }
  else
    Serial.println("Error Initializing MCP2515 0...");
    display.printf("Error Initializing MCP2515 0...");
    display.display();
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  
  Serial.println("MCP2515 0 Library Receive Example...");


//***** CAN 1 *****//
   if(CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK){
    Serial.println("MCP2515 1 Initialized Successfully!");
    display.printf("MCP2515 1 Initialized Successfully!");
    display.display();
   }
  else
    Serial.println("Error Initializing MCP2515 1...");
    display.printf("Error Initializing MCP2515 1...");
    display.display();
  
  CAN1.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN1_INT, INPUT);                            // Configuring pin for /INT input
  
  Serial.println("MCP2515 1 Library Receive Example...");
  display.printf("MCP2515 1 Library Receive Example...");
  display.display();
  delay(1000);
}

byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

void loop()
{

  MQTT_connect();
  MQTT_ping();

   ///// SEND CODE FOR CAN 0 /////

      //***** CAN 0 *****//
     // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
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
  if(!digitalRead(CAN1_INT))                         // If CAN1_INT pin is low, read receive buffer
  {
    CAN1.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    
    if((rxId & 0x80000000) == 0x80000000) {    // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
      display.printf(msgString, "Extended ID: 0x%.8lX DLC: %1d Data:", (rxId & 0x1FFFFFFF), len);
      display.display();
    }
    else{ 
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
      display.printf(msgString, "Standard ID: 0x%.3lX     DLC: %1d  Data:", rxId, len);
      display.display();
      if(rxId == 0x025) {                         // if the ID is 0x025, then it is the wheelposition
        wheelPosition = rxBuf[0] <<8 | rxBuf[1];
        Serial.printf("Wheel position %i \n", wheelPosition);
        wheelTime = millis();
      }

      Serial.print(msgString);
    }

    // publish feeds on dashboard // 
    if((millis()-lastTime > 6000)) {
      if(mqtt.Update()) {
        brakepedalFeed.publish(rxId);
        Serial.printf("Publishing %.3lx \n", rxId);
        SIDFeed.publish(rxId);
        Serial.printf("Publishing %.3lx \n", rxId);
        if(wheelTime - lastTime < 6000) {                    // if the wheel position is read under 60 seconds, publish to feed
          steeringwheelFeed.publish(wheelPosition);
          Serial.printf("Publishing %i \n", wheelPosition);
        }      
      }
      lastTime = millis();
    }

    
    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
        
    Serial.println();
  }

}

// create/define functions and call them within the loop
void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT...");

  while ((ret = mqtt.connect()) != 0) {
    Serial.printf("Error Code %s\n", mqtt.connectErrorString(ret));
    Serial.printf("Retrying MQTT connection in 5 seconds...\n");
    mqtt.disconnect();
    delay(5000);
  }
  Serial.printf("MQTT Connected!\n");
}


bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if((millis()-last)>120000) {
    Serial.printf("Pinging MQTT \n");
    pingStatus = mqtt.ping();
    if(!pingStatus) {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
    last = millis();
  }
  return pingStatus;
}