// This #include statement was automatically added by the Particle IDE.
#include "clickButton/clickButton.h"

// WARNING Initial hack !!!!!

// **** TODO ****
// Add individual node web reset particle function
// Add function to add nodes at startup
// Add function to indicate missing data after say 5 minutes
// Improve sequencing of data transmissions, ACKs and checks for connectivity
//

#include "application.h"
#include "RFM69.h"
#include "RFM69registers.h"
//#include "clickButton/clickButton.h"

//SYSTEM_MODE(AUTOMATIC);
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

PRODUCT_VERSION(2);
PRODUCT_ID(162);

//#define RANDOMDATA
//#define DEBUG_ON        //DEBUG is defined so cannot use)
#define DEBUG_MIN
#define DEBUG_ADDNODE


#define sDesc "Falco Manager"
#define sVersion "v0.5.8.2"
#define hVersion "v0.5.0"
#define tStars "***************************"

#define  OTHER_ADDRESS 0x04 

#define led_delay 500

unsigned int lastPublish = 0;

#define NODEID          31   
#define NETWORKID       100  
#define NETWORKID_SU    1
#define FREQUENCY       RF69_868MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY      "setup1EncryptKey" 
#define IS_RFM69HW      //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME        450 // max # of ms to wait for an ack default 30
#define SERIAL_BAUD     9600 // 57600
#define SW1             D5 //was D4
#define buttonPin       D5 // was D4 FIX 
#define LED             D7 
#define ledPin          D7 //FIX

String EK_SU = ENCRYPTKEY;
char EK1[17] = {'s', 'a', 'm', 'p', 'l', 'e', 'E', 'n', 'c', 'r', 'y', 'p', 't', 'K', 'e', 'y', '\0'};
uint8_t theNodeID = 99; 
  
int ledState = HIGH; 
int lastButtonState = LOW;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

byte ackCount=0;

byte SW1Counter = 0;
byte SW1State = 0;

//byte resetTemp = 0;

typedef struct {
  uint8_t           node;
  uint8_t           tran;
  int16_t           batt;   // battery voltage in millivolts
  int16_t           value1; // Current temperature in dec * 100
  int16_t           value2; // Minimum temperature in dec * 100
  int16_t           value3; // Maximum temperature in dec * 100
  int16_t           value4; 
  #ifdef DEBUG_ADDNODE
  int16_t           value5; // Maximum temperature in dec * 100
  int16_t           value6; // Maximum temperature in dec * 100
  int16_t           value7; // Maximum temperature in dec * 100
  int16_t           value8; // Maximum temperature in dec * 100
  #endif
} Payload;
Payload theData;

bool promiscuousMode = false;

int mem1 = 0;
int newSeed;

RFM69 radio;

int reboots = 10;
int wifiOff = 11;
uint8_t addr1 = 1;
uint8_t addr2 = 5;
uint8_t addr3 = 9;

String deviceID = Particle.deviceID();
String deviceCode = "1";

int cloudReset(String command);

IPAddress remoteIP(181,224,135,60);
int numberOfReceivedPackage = 0;

uint8_t addNodeStatus = 0;
uint8_t resetTempStatus = 0;

boolean nodePresent[9]; // used
int UNID = 99;
unsigned long lastData[9];
boolean nodeDataMissing[9];
boolean tResetRequested[9];
boolean tResetCompleted[9];
uint8_t numNodes = 9;

#define MINUTES_1 (1 * 60 * 1000)
#define MINUTES_2 (2 * 60 * 1000)
#define MINUTES_5 (5 * 60 * 1000)
unsigned long lastDataCheck = millis();

unsigned long delayStartTime = 0;
unsigned long delayMS = 0;

// the Button
const int buttonPin1 = buttonPin;
ClickButton button1(buttonPin1, HIGH, CLICKBTN_PULLUP);
int8_t function = 0; // was int
int button1State = 0;


/*
char* dataArray[] = {"test1|5000|xxxxxxxxxxxxxxxxxx", 
"test01|2000|xxxxxxxxxxxxxxxxxx",
"test02|2000|xxxxxxxxxxxxxxxxxx",
"test03|2000|xxxxxxxxxxxxxxxxxx",
};
*/


//**** START SETUP ****
//**** START SETUP ****
//**** START SETUP ****
void setup() {

  if (Particle.connected())
  {
    Particle.process();
  } 
  
  Serial.begin(SERIAL_BAUD);

  // was in functio
  #ifdef DEBUG_ON  
  Serial.println(sVersion);
  Serial.println(serialTitle);
  #endif

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  pinMode(buttonPin1, INPUT_PULLDOWN);
  //pinMode(buttonPin1, INPUT_PULLUP); 
  // Setup button timers (all in milliseconds / ms)
  // (These are default if not set, but changeable for convenience)
  button1.debounceTime   = 20;   // Debounce timer in ms
  button1.multiclickTime = 250;  // Time limit for multi clicks
  button1.longClickTime  = 1000; // time until "held-down clicks" register
  
  delay(50);
  digitalWrite(LED, LOW);
  delay(50);

  #ifdef DEBUG_MIN
  Time.zone(0);
  Serial.println(Time.timeStr());
  #endif
  Wire.begin();
 
  //#ifdef DEBUG_MIN      
  Serial.println("");
  Serial.println("");
  Serial.print("Decription: ");
  Serial.println(sDesc);
  Serial.print("Software version: ");
  Serial.println(sVersion);
  Serial.print("Hardware version: ");
  Serial.println(hVersion);
  Serial.println(tStars);
  //#endif
  // Currently the application supports the creation of up to 4 different cloud functions
  // 12 characters long with name
  // String parameter of 63 characters.
  Particle.function("resetWifi", cloudResetWifi);
  Particle.function("resetReboots", cloudResetReboots);  
  Particle.function("manageNodes", manageNodes);  
  
  
  Particle.variable("sVersion", sVersion, STRING);
  Particle.variable("hVersion", hVersion, STRING);
  Particle.variable("sDesc", sDesc, STRING);
  
  Particle.variable("button1State", &button1State, INT);
  Particle.variable("reboots", &reboots, INT);
  Particle.variable("wifiOff", &wifiOff, INT);
  
#ifdef DEBUG_ON
  Serial.print("deviceID = ");
  Serial.println(deviceCode);
  Particle.variable("deviceID", deviceID);
#endif
  
  Particle.subscribe("hook-response/temperature1", tempResponse, MY_DEVICES);

  initializeRadio(NETWORKID, EK1);
  
  randomSeed(newSeed);
  
  Blink(LED,1000);

  reboots = EEPROM.read(addr1);
  reboots++;
  EEPROM.update(addr1, reboots);
#ifdef DEBUG_ON
  Serial.print("Reboots = ");
  Serial.println(reboots);
#endif
  
  wifiOff = EEPROM.read(addr2); // DO NOT INCREMENT
  #ifdef DEBUG_ON  
  Serial.print("WifiOff = ");
  Serial.println(wifiOff);
  #endif

  wireTrans("te|0|0|0||0|0");  
  delay(5000);
  wireTrans("te|0|0|0|" + String(sDesc) + "|0|1");
  delay(500);
  wireTrans("te|0|0|0||0|0");
  delay(500);
  wireTrans("te|0|0|0|SW: " + String(sVersion) + "|0|1");
  delay(500);
  wireTrans("te|0|0|0||0|0");
  delay(500);
  wireTrans("te|0|0|0|HW: " + String(hVersion) + "|0|1");
  delay(500);
  wireTrans("te|0|0|0|Waiting for data|0|0");
  delay(500);

 //Mark all nodes as present - FUTURE allow for only specified devices to be added
  for (int i = 0; i < numNodes; i++) {
    nodePresent[i] = 0; //FIX ?? breaks reset, see fix below
    #ifdef DEBUG_MIN       
    Serial.print("nodePresent [node ");
    Serial.print(i + 1);
    Serial.print("] = ");
    Serial.println(nodePresent[i]);
    #endif
  }
 
  
  //FIX for node 4
  EEPROM.update(103, 1);
  
  Serial.println(); 
  UNID = unusedNodeID();
  Serial.print("\nUNID = "); 
  Serial.println(UNID);
  
  mem1 = System.freeMemory();
  #ifdef DEBUG_MIN  
  Serial.print("\nFree memory:"); 
  Serial.println(mem1);
  #endif
  
  #ifdef DEBUG_MIN 
  Serial.println("\nStartup completed ... "); 
  Serial.println(tStars);
  #endif
 
}
// End setup
// End setup
// End setup



// Start loop
// Start loop
// Start loop
void loop() {

  if (!Particle.connected() && ((wifiOff % 4) == 0))
        Particle.connect();

  if (Particle.connected())
  {
    Particle.process();
  }  
  
  
  if (millis() - lastDataCheck > MINUTES_2) 
  { //ADD FIX for millis turning over to 0 
    lastDataCheck = millis();
    #ifdef DEBUG_ON
    Serial.println("millis() check ...");
    #endif
    
    for (int i = 0; i < numNodes; i++) 
    {
      if ((lastData[i] != 0) && (lastDataCheck - lastData[i] > MINUTES_2) && (nodeDataMissing[i] == 0))
      {
        nodeDataMissing[i] = 1;
        int nv = i + 1;
        wireTrans("md|" + String(nv) + "|0|0|0|0|0");
        #ifdef DEBUG_MIN      
        Serial.print("Node ");
        Serial.print(i + 1);
        Serial.print(" data out of date. Sent [");
        Serial.print("md|" + String(nv) + "|0|0|0|0|0");
        Serial.println("]");
        #endif
      }
    }
  }
  
  
  if ((addNodeStatus ==2) && (millis() - delayStartTime > delayMS))
  {
    initializeRadio(NETWORKID, EK1);
    addNodeStatus = 0;
    Serial.println("Node add failed");
    wireTrans("te|0|0|0|Add timeout|0|1");
    delay(1000);
    wireTrans("te|0|0|0||0|0");
  }
  
  
  button1.Update();
  function = button1.clicks;
    
  if (function < 0)  // Long click, check if node to be added
  {
    wireTrans("te|0|0|0|Click to add node|0|1");
    addNodeStatus = 1;
    delay(500);
    delayStartTime = millis();
    delayMS = 15000;  //delayMS = 4000;
    while ((millis() - delayStartTime < delayMS))
    {
      //wireTrans("te|0|0|0|.|0|0");      
      delay(5);
      button1.Update();
      function = button1.clicks;
      if (function > 0)  // ie a short button click
      {
        addNodeStatus = 2;  
        wireTrans("te|0|0|0|Waiting ...|0|1");
        initializeRadio(NETWORKID_SU, EK_SU);
        break;
      }
      //delay(10);
    }
    if (addNodeStatus == 1)
    {
      wireTrans("te|0|0|0|Add failed|0|1");
      addNodeStatus = 0;
      delay(1000);
      wireTrans("te|0|0|0||0|0");
    }
  }

  
  if (function > 0 && addNodeStatus == 0)
  {
    function = 0; // reset variable for button1.clicks
    wireTrans("rt|999|0|0|0|0|0");
    for (int i = 0; i < numNodes; i++) {
      //if (nodePresent[i] == 1) //FIX *************************
      //{
        tResetRequested[i] = 1;
        tResetCompleted[i] = 1;
        printNodeStatus();
      //}
    }
  }
      
      
  if (radio.receiveDone())
  {
    #ifdef DEBUG_MIN 
    Serial.println("");
    Serial.print("At ");
    Serial.println(Time.timeStr());
    Serial.print('[');
    Serial.print(radio.SENDERID, DEC);
    Serial.print("] ");
    Serial.print(" [RX_RSSI:");
    Serial.print(radio.readRSSI());
    Serial.print("]");
    #endif
    if (promiscuousMode)
    {
      #ifdef DEBUG_ON
      Serial.print(" to [");
      Serial.print(radio.TARGETID, DEC);
      Serial.print("] ");
      #endif
    }

    if (radio.DATALEN != sizeof(Payload))
    {
      #ifdef DEBUG_MIN
      Serial.print("[Payload = ");
      Serial.print(sizeof(Payload));
      Serial.print(" / radio.DATALEN = ");
      Serial.print(radio.DATALEN);
      Serial.println("] Invalid payload received, not matching Payload struct!");
      #endif
    }
    else
    {
    theData = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
    #ifdef DEBUG_MIN
    Serial.print("Node=");
    Serial.println(theData.node);
    #endif
    
    theNodeID = theData.node;
    // FIX FIX must check if node is not on network 1
    //lastData[theNodeID - 1] = Time.now();   
    lastData[theNodeID - 1] = millis();  // time may not be reliable if device starts up disconnected from the web   
    nodeDataMissing[theNodeID - 1] = 0; //since data is retrived do not need to send 
    
    #ifdef RANDOMDATA
    uint8_t nodeRandomizer;
    nodeRandomizer = random(0,2);
    node = node + nodeRandomizer;
    #endif
    
    #ifdef DEBUG_MIN 
    Serial.print("tran=");
    Serial.println(theData.tran);
    #endif 
     
    int batt1 = rndInt(theData.batt)/100;
    int batt2 = rndInt(theData.batt)/10;
    int temp1 = rndInt(theData.value1)/10;
    int temp2 = rndInt(theData.value1);
    int tMin = rndInt(theData.value2)/10;
    int tMax = rndInt(theData.value3)/10;
    
    int tFlag = 0;
    if ((temp1 > 200) || (temp1 < 40)) tFlag = 1;
      
    #ifdef DEBUG_ON
    Serial.println("");
    Serial.print("Wire: ");
    Serial.println("[tran|node|batt|value1|value2|value3|tFlag] = ");
    #endif
      
    #ifdef DEBUG_MIN      
    Serial.print(Time.timeStr(lastData[theNodeID - 1]));
    Serial.print(" [dt|");
    Serial.print(theNodeID);
    Serial.print("|");
    Serial.print(batt1);
    Serial.print("|");
    Serial.print(temp1);
    Serial.print("|");
    Serial.print(tMin);
    Serial.print("|");
    Serial.print(tMax);
    Serial.print("|");
    Serial.print(tFlag);
    Serial.println("]");
    #endif
      
    if (tResetCompleted[theNodeID - 1] == 1) 
    {
      tMin = temp1;
      tMax = temp1;
      tResetCompleted[theNodeID - 1] = 0;
      #ifdef DEBUG_MIN
      Serial.print(temp1);
      Serial.print("|");
      Serial.print(tMin);
      Serial.print("|");
      Serial.print(tMax);
      Serial.print("|");
      #endif
    }
      
    Serial.println();
      
    // displayThermometer(int layout, int tval, int tmin, int tmax, int tx, int ty, int flag)
    String value1 = "{ \"node\": \"" + String(theData.node) + "\", \"volts\": \"" + String(batt2) + "\", \"temp1\": \"" + String(temp2) + "\", \"sVersion\": \"" + String(SW1Counter) + "\"  }";
    String value2 = "dt|" + String(theNodeID) + "|" + String(batt1) + "|" + String(temp1) + "|" + String(tMin) + "|" + String(tMax)  + "|" + String(tFlag);
    
    /*
    Codes:
    te = text
    rt = reset temp
    dt = display tem
    String value2 = "10|" + String(theNodeID) + "|" + String(batt1) + "|" + String(temp1) + "|" + String(tMin) + "|" + String(tMax)  + "|" + String(tFlag);
    */
    
    #ifdef DEBUG_MIN
    Serial.println("Pinging started...");
    #endif
    numberOfReceivedPackage = 0;
    numberOfReceivedPackage = WiFi.ping(remoteIP);
    #ifdef DEBUG_MIN
    Serial.println("Pinging ended ->");
    Serial.println(numberOfReceivedPackage);
    #endif
      
    bool success;
    success = Particle.publish("temperature1", value1, 60, PRIVATE);
      
      #ifdef DEBUG_MIN  
      if (success) {
        Serial.println("'temperature1' Particle.publish - success");
      } else {
        Serial.println("'temperature1' Particle.publish - failed");
      }
      #endif
      
      if (!success) {
        wifiOff++;   
        EEPROM.update(addr2, wifiOff);
        #ifdef DEBUG_MIN 
        Serial.println("wifiOff updated");
        #endif
        }
      
      #ifdef DEBUG_ON  
      Serial.print("wifi disconnects = ");
      Serial.println(wifiOff);
      #endif

    if (theData.tran == 2)
    {
      wireTrans(value2);
    }
      #ifdef DEBUG_ON  
      reboots = EEPROM.read(addr1);
      Serial.print("Reboots = ");
      Serial.println(reboots);
      #endif
  }
  
  theNodeID = radio.SENDERID;

  if (radio.ACK_REQUESTED)
  {
    radio.sendACK();
    //#ifdef DEBUG_MIN
    Serial.println(" - ACK sent.");
    //#endif
  } else {
    //#ifdef DEBUG_MIN
    Serial.println(" - NO ACK sent.");
    //#endif
  }
  
  #ifdef DEBUG_MIN
  Serial.print("tResetRequested = ");
  Serial.println(tResetRequested[theNodeID - 1]);
  #endif
  
  #ifdef DEBUG_MIN
  Serial.print("theNodeID = ");
  Serial.println(theNodeID);
  #endif
  
  int nID = 0;
  nID = theNodeID - 1;
  String n1 = "";
  bool sendRF = 0;
        
  if (tResetRequested[nID] == 0) 
  {
    sendRF = 1;
    n1 = ""; 
    theData.node = theNodeID;
    theData.tran = 3;
    theData.batt = NETWORKID;
    theData.value1 = 0;
    theData.value2 = 0;
    theData.value3 = 0;
    theData.value4 = 0;
    theData.value5 = 0;
    theData.value6 = 0;
    theData.value7 = 0;
    theData.value8 = 0;
    #ifdef DEBUG_MIN
    Serial.print(" ..Node = ");
    Serial.print(theData.node);
    Serial.print(", Tran = ");
    Serial.println(theData.tran);
    #endif
  }
  
  if (tResetRequested[nID] == 1)
  {
    sendRF = 1;
    n1 = "temp reset"; 
    theData.node = theNodeID;
    theData.tran = 99;
    theData.batt = NETWORKID;
    theData.value1 = 0;
    theData.value2 = 0;
    theData.value3 = 0;
    theData.value4 = 0;
    theData.value5 = 0;
    theData.value6 = 0;
    theData.value7 = 0;
    theData.value8 = 0;
    #ifdef DEBUG_MIN
    Serial.print(" ..Node = ");
    Serial.print(theData.node);
    Serial.print(", Tran = ");
    Serial.println(theData.tran);
    #endif
  }   
  
  if (addNodeStatus == 2)
  {
    sendRF = 1;
    n1 = "register node"; 
    theNodeID = 1;
    //theData.node = 5; //FIX - add ability to lookup nodeID
    UNID = unusedNodeID();
    if (UNID == 99) UNID = 1;
    theData.node = UNID;
    theData.tran = 90;
    theData.batt = NETWORKID;
    theData.value1 = EK1[0] | uint16_t(EK1[1]) << 8;
    theData.value2 = EK1[2] | uint16_t(EK1[3]) << 8;
    theData.value3 = EK1[4] | uint16_t(EK1[5]) << 8;
    theData.value4 = EK1[6] | uint16_t(EK1[7]) << 8;
    theData.value5 = EK1[8] | uint16_t(EK1[9]) << 8;
    theData.value6 = EK1[10] | uint16_t(EK1[11]) << 8;
    theData.value7 = EK1[12] | uint16_t(EK1[13]) << 8;
    theData.value8 = EK1[14] | uint16_t(EK1[15]) << 8;
    
    #ifdef DEBUG_MIN
    Serial.print(" ..Node = ");
    Serial.print(theData.node);
    Serial.print(", Tran = ");
    Serial.println(theData.tran);
    Serial.print(", v1 = ");
    Serial.println(theData.value1);
    Serial.print(", v2 = ");
    Serial.println(theData.value2);
    Serial.print(", v3 = ");
    Serial.println(theData.value3);
    Serial.print(", v4 = ");
    Serial.println(theData.value4);
    Serial.print(", v5 = ");
    Serial.println(theData.value5);
    Serial.print(", v6 = ");
    Serial.println(theData.value6);
    Serial.print(", v7 = ");
    Serial.println(theData.value7);
    Serial.print(", v8 = ");
    Serial.println(theData.value8);
    #endif
  }
  
  #ifdef DEBUG_MIN
  Serial.print("theNodeID = ");
  Serial.print(theNodeID);
  Serial.print(", Node = ");
  Serial.print(theData.node);
  Serial.print(", Tran = ");
  Serial.println(theData.tran);
  #endif
  
  if (sendRF == 1)
  {
    if (radio.sendWithRetry(theNodeID, (const void*)(&theData), sizeof(theData)), 1) // node must be a byte
    { 
      
      Serial.println(Time.timeStr() + ": OK ... " + n1 + " message sent to and received by Node " + theNodeID + " [Tran=" + theData.tran + "]");
      
      if (tResetRequested[nID] == 1) tResetRequested[nID] = 0;
      
      if (addNodeStatus == 2)
      { 
        addNodeStatus = 0;
        uint8_t UNIDUpdate = UNID + 100 - 1;
        Serial.print("************ UNIDUpdate = ");
        Serial.println(UNIDUpdate);
        EEPROM.update(UNIDUpdate, 1);
        wireTrans("te|0|0|0|Node " + String(UNID) + " added.|0|1");
        delay(500);
        wireTrans("te|0|0|0||0|0");
        initializeRadio(NETWORKID, EK1);
      }
    } 
    else 
    {
      //#ifdef DEBUG_MIN
      Serial.println(Time.timeStr() + ": Fail ... " + n1 + " message NOT sent to and received by Node " + theNodeID + " [Tran=" + theData.tran + "]");
      //#endif
      
      if (tResetRequested[nID] == 1) tResetRequested[nID] = 0;
      
      if (addNodeStatus == 2)
      { 
        wireTrans("te|0|0|0|Node " + String(UNID) + " FAIL.|0|1");
        delay(500);
        wireTrans("te|0|0|0|.|0|0");
        //initializeRadio(NETWORKID, EK1);
      }
    }
    sendRF = 0;
  }    

  printNodeStatus();
  displayFreeMem(); 
  } // END receiveDone()

}



// **** END LOOP ****
// **** END LOOP ****
// **** END LOOP ****



// **** START FUNCTIONS ****
// **** START FUNCTIONS ****
// **** START FUNCTIONS ****



void Blink(byte PIN, int DELAY_MS)
{
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}



void tempResponse(const char *name, const char *data) {
    String responseStr = String(data);
    responseStr.replace("\"", "");
    #ifdef DEBUG_ON
    Serial.println("Web response: [" + responseStr + "]");
    #endif
}



bool readSW(int SW)
{
  bool SWState = 0;
  if(digitalRead(SW)) 
  {
    delay(50); // was 50
    if(digitalRead(SW))
      return SWState = 1;
  } else 
      return SWState = 0;
}



void switchCount(){
  if (SW1State == 1) {
    ++SW1Counter;
    //int addr = 1;
    //EEPROM.update(addr3, SW1Counter);

    digitalWrite(LED, HIGH);
    #ifdef DEBUG_ON
    Serial.print("Switch counter = ");
    Serial.println(SW1Counter);
    #endif
    //delay(250);
    delay(10);
    digitalWrite(LED, LOW);
  }
}



int rndUpDn(int in1, int integerDecPlaces, int roundedDecPlaces) {
  int divFactor = integerDecPlaces/roundedDecPlaces;
  int roundingFactor = divFactor/10;
  if (divFactor == 1) roundingFactor = 0;
  
  if (in1 != 9999) {
    int round1 = in1 / divFactor;
    int round2;
    int round3;
    if (in1 < 0) {
      round2 = (in1 - (5 * roundingFactor)) / divFactor;
      round3 = (in1 / divFactor) - 1;
    } else {
      round2 = (in1 + (5 * roundingFactor)) / divFactor;
      round3 = (in1 / divFactor) + 1;    
    }
    if (round1 == round2) {
      #ifdef DEBUG_ON
      Serial.print("Result = ");
      Serial.println(round1);
      #endif
      return round1;
    }
    else {     
      #ifdef DEBUG_ON
      Serial.print("Result = ");
      Serial.println(round3);
      #endif
      return round3;
    }
    
  } else {
    return in1; // ie 9999
  }
}



int cloudResetReboots(String command)
{
  // look for the matching argument "coffee" <-- max of 64 characters long
  if(command == "1")
  {
    reboots = 0;
    EEPROM.update(addr1, reboots);
    #ifdef DEBUG_ON
    Serial.print("Reboots = ");
    Serial.println(reboots);
    #endif
    return 200;
  }
  else return -1;
}



int manageNodes(String command)  // FUTURE - use to repotely set values
{
    
    Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
    Wire.write("te|0|0|0|[" + command + "]|0|0");
    Wire.endTransmission(true);
  /*
  // look for the matching argument "coffee" <-- max of 64 characters long
  if(command == "1")
  {
    reboots = 0;
    EEPROM.update(addr1, reboots);
    #ifdef DEBUG_ON
    Serial.print("Reboots = ");
    Serial.println(reboots);
    #endif
    return 200;
  }
  else return -1;
  */
}



int cloudResetWifi(String command)
{
  // look for the matching argument "coffee" <-- max of 64 characters long
  if(command == "1")
  {
    wifiOff = 0;
    EEPROM.update(addr2, wifiOff);
    #ifdef DEBUG_ON
    Serial.print("WifiOff = ");
    Serial.println(wifiOff);
    #endif
    return 200;
  }
  else return -1;
}

int rndInt(int intToRnd)
{
  int modAdj = 0;
  if (intToRnd < 0)
  {
    modAdj = -intToRnd % 10;
    if (modAdj >= 5) intToRnd = intToRnd - 5;
  }
  if (intToRnd > 0)
  {
    modAdj = intToRnd % 10;
    if (modAdj >= 5) intToRnd = intToRnd + 5;
  }
  return intToRnd;
}



void connect() 
{
  if (Particle.connected() == false) 
  {
    Particle.connect();
  }
}



void initializeRadio(uint8_t rfNetworkID, String rfEncryptkey)
{
  radio.initialize(FREQUENCY, NODEID, rfNetworkID);
  #ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
  int highPower = 1;
  #endif
  radio.encrypt(rfEncryptkey);
  radio.promiscuous(promiscuousMode);
  delay(100);
  #ifdef DEBUG_ON
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  Serial.print("Promiscuous mode = ");
  Serial.println(promiscuousMode);
  Serial.print("Power level = ");
  Serial.println(highPower);
  Serial.print("Encryption key = [");
  Serial.print(rfEncryptkey);
  Serial.println("]");
  #endif
}



int unusedNodeID()
{
  Serial.print("\n-->Checking for unused Node IDs");
  Serial.print("\n-->>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  int unusedNodeID = 99;
  for (uint8_t i = 0; i < numNodes; i++)
  {
    Serial.print("\nNode: ");
    uint8_t n = i + 1;
    Serial.print(n);
    uint8_t addr = i + 100;
    Serial.print(" Address: ");
    Serial.print(addr);
    uint8_t val = EEPROM.read(addr);
    Serial.print(" Value: ");
    Serial.print(val);
    if (val == 0)
    {
      Serial.print(" = Unused Node ID ");
      Serial.println(n);
      unusedNodeID = n;
      return unusedNodeID;
    } 
  }
  if (unusedNodeID == 99)
  {
    for (uint8_t i = 0; i < numNodes; i++)
    {
      uint8_t addr = i + 100;
      EEPROM.update(addr, 0);
    }
  }
  return unusedNodeID;  // return node number or error message
}



void wireTrans(String wt)
{
  Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
  Wire.write(wt);
  Wire.endTransmission(true);    // stop transmitting
} 

  
  
void printNodeStatus()
{
  #ifdef DEBUG_ON         
  Serial.print("Node = [");
  Serial.print(i + 1);
  Serial.print("] tResetRequested = [");
  Serial.println(tResetRequested[i]);
  Serial.print("]\tlastData = [ ");
  Serial.print(lastData[i]);
  Serial.print("]");
  #endif
}



void displayFreeMem()
{
  #ifdef DEBUG_MIN  
  mem1 = System.freeMemory();
  Serial.print("Free memory=");
  Serial.println(mem1);
  #endif
}
