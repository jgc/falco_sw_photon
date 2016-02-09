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

SYSTEM_MODE(SEMI_AUTOMATIC);
//SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

PRODUCT_VERSION(2);
PRODUCT_ID(162);

//#define RANDOMDATA
//#define DEBUG_ON        //DEBUG is defined so cannot use)
//#define DEBUG_MIN

#define sDesc "Falco Manager"
#define sVersion "v0.5.4"
#define hVersion "v0.5.0"
#define tStars "***************************"

#define  OTHER_ADDRESS 0x04 

#define led_delay 500

unsigned int lastPublish = 0;

#define NODEID          31   
#define NETWORKID       100  
#define FREQUENCY       RF69_868MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY      "sampleEncryptKey" 
#define IS_RFM69HW      //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME        450 // max # of ms to wait for an ack default 30
#define SERIAL_BAUD     9600 // 57600
#define SW1             D4
#define buttonPin       D4 //FIX 
#define LED             D7 
#define ledPin          D7 //FIX

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

boolean nodePresent[9];
unsigned long lastData[9];
boolean nodeDataMissing[9];
boolean tResetRequested[9];
boolean tResetCompleted[9];
uint8_t numNodes = 9;

#define MINUTES_1 (1 * 60 * 1000)
#define MINUTES_2 (2 * 60 * 1000)
#define MINUTES_5 (5 * 60 * 1000)
unsigned long lastDataCheck = millis();

/*
// the Button
const int buttonPin1 = 4;
ClickButton button1(buttonPin1, LOW, CLICKBTN_PULLUP);
// Button results 
uint8_t function = 0; // was int
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
  
  pinMode(SW1, INPUT_PULLDOWN);
 
  /*
  //pinMode(D4, INPUT_PULLUP);
  // Setup button timers (all in milliseconds / ms)
  // (These are default if not set, but changeable for convenience)
  button1.debounceTime   = 20;   // Debounce timer in ms
  button1.multiclickTime = 250;  // Time limit for multi clicks
  button1.longClickTime  = 1000; // time until "held-down clicks" register
  */
  
  delay(2000);
  digitalWrite(LED, LOW);
  delay(2000);

  #ifdef DEBUG_MIN
  Time.zone(0);
  Serial.println(Time.timeStr());
  #endif
  Wire.begin();
  // end was in functio
  

  //#ifdef DEBUG_MIN      
  Serial.println("");
  Serial.println("");
  Serial.println(tStars);
  Serial.print("Decription: ");
  Serial.println(sDesc);
  Serial.print("Software version: ");
  Serial.println(sVersion);
  Serial.print("Hardware version: ");
  Serial.println(hVersion);
  Serial.println("");
  //#endif
  
  Particle.function("resetWifi", cloudResetWifi);
  Particle.function("resetReboots", cloudResetReboots);  
  Particle.variable("sVersion", sVersion, STRING);
  Particle.variable("hVersion", hVersion, STRING);
  Particle.variable("sDesc", sDesc, STRING);
  //Particle.variable("SW1State", &SW1State, INT);
  //Particle.variable("SW1Counter", &SW1Counter, INT);
  Particle.variable("reboots", &reboots, INT);
  Particle.variable("wifiOff", &wifiOff, INT);
  
#ifdef DEBUG_ON
  Serial.print("deviceID = ");
  Serial.println(deviceCode);
  Particle.variable("deviceID", deviceID);
#endif
  
  Particle.subscribe("hook-response/temperature1", tempResponse, MY_DEVICES);
  pinMode(LED, OUTPUT);
  
  //Serial.begin(SERIAL_BAUD);
  //delay(10);
 
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
  int highPower = 1;
#endif
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(promiscuousMode);

#ifdef DEBUG_ON
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  Serial.print("Promiscuous mode = ");
  Serial.println(promiscuousMode);
  Serial.print("Power level = ");
  Serial.println(highPower);
#endif

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
  
  String valueSU;
 
  Wire.beginTransmission(OTHER_ADDRESS);
  Wire.write("te|0|1|3|Startup completed|0|1");
  //delay(100);  // try to fix i2c issue
  Wire.endTransmission(true);    // stop transmitting
  delay(1000);
  
  Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
  Wire.write("te|0|0|0|" + String(sDesc) + "|0|1");
  //delay(100);  // try to fix i2c issue
  Wire.endTransmission(true);    // stop transmitting
  delay(1000);
  
  Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
  Wire.write("te|0|0|0|SW: " + String(sVersion) + "|0|1");
  //delay(100);  // try to fix i2c issue
  Wire.endTransmission(true);    // stop transmitting
  delay(1000);
  
  Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
  Wire.write("te|0|0|0|HW: " + String(hVersion) + "|0|1");
  //delay(100);  // try to fix i2c issue
  Wire.endTransmission(true);    // stop transmitting
  delay(1000);  // try to fix i2c issue
  
  Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
  Wire.write("te|0|0|0|Press button|0|1");
  //delay(100);  // try to fix i2c issue
  Wire.endTransmission(true);
  delay(1000);  // try to fix i2c issue

  //checkButton();
  if (readSW(SW1) == 1)
  {
    Wire.beginTransmission(OTHER_ADDRESS);
    Wire.write("te|0|0|0|Button pressed|0|1");
    //delay(100);  // try to fix i2c issue
    Wire.endTransmission(true);
    //delay(2000);  // try to fix i2c issue
  }
  
  mem1 = System.freeMemory();
  #ifdef DEBUG_MIN  
  Serial.print("Free memory:"); 
  Serial.println(mem1);
  #endif
  
  //Mark all nodes as present - FUTURE allow for only specified devices to be added
  for (int i = 0; i < numNodes; i++) {
    nodePresent[i] = 1;
    #ifdef DEBUG_ON          
    Serial.print("nodePresent [node ");
    Serial.print(i + 1);
    Serial.print("] = ");
    Serial.println(nodePresent[i]);
    #endif
  }
  
  #ifdef DEBUG_MIN 
  Serial.println(""); 
  Serial.println("Startup completed ... "); 
  Serial.println(tStars);
  #endif
  
  delay (10000);
 
}
// End setup
// End setup
// End setup



void loop() {

  if (!Particle.connected() && ((wifiOff % 4) == 0))
        Particle.connect();

  if (Particle.connected())
  {
    Particle.process();
  }  
  
  
  if (millis() - lastDataCheck > MINUTES_2) { //ADD FIX for millis turning over to 0 
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
        String wireMD = "md|" + String(nv) + "|0|0|0|0|0";
        //[tran|node|batt|value1|value2|value3|tFlag]
        Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
        Wire.write(wireMD);
        Wire.endTransmission(true);    // stop transmitting
        #ifdef DEBUG_MIN      
        Serial.print("Node ");
        Serial.print(i + 1);
        Serial.print(" data out of date. Sent [");
        Serial.print(wireMD);
        Serial.println("]");
        #endif
      }
    }
  }
  
  
  SW1State = readSW(SW1);
  switchCount();

  if (SW1Counter >= 1){
    
    // **** FUTURE **** Add web based reset on a per node basis
    
    // Reset all temperatures on the display
    String value3 = "rt|999|0|0|0|0|0";
    Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
    Wire.write(value3);
    delay(100);  // try to fix i2c issue
    Wire.endTransmission(true);    // stop transmitting
    delay(100);  // try to fix i2c issue - does not work 

    #ifdef DEBUG_ON  
    Serial.print(Time.timeStr());
    Serial.print(" [");
    Serial.print(value3);
    Serial.println("]");
    #endif
    
    for (int i = 0; i < numNodes; i++) {
      if (nodePresent[i] == 1)
      {
        tResetRequested[i] = 1;
        tResetCompleted[i] = 1;
        #ifdef DEBUG_ON         
        Serial.print("tResetRequested [node ");
        Serial.print(i + 1);
        Serial.print("] = ");
        Serial.println(tResetRequested[i]);
        #endif
      }
    }
    
    SW1Counter = 0;

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
      
      if (tResetCompleted[theNodeID - 1] == 1) {
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

      Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
      Wire.write(value2);
      delay(100);  // try to fix i2c issue
      Wire.endTransmission(true);    // stop transmitting
      delay(100);  // try to fix i2c issue

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
        #ifdef DEBUG_MIN
        Serial.println(" - ACK sent.");
        #endif
    } else {
        #ifdef DEBUG_MIN
        Serial.println(" - NO ACK sent.");
        #endif
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

  if (tResetRequested[nID] == 1) {
    theData.node = theNodeID;
    theData.tran = 99;
    theData.batt = 0;
    theData.value1 = 0;
    theData.value2 = 0;
    theData.value3 = 0;
    theData.value4 = 0;
    #ifdef DEBUG_MIN
    Serial.print(" ..Node = ");
    Serial.print(theData.node);
    Serial.print(", Tran = ");
    Serial.println(theData.tran);
    #endif
  } else {
    theData.node = theNodeID;
    theData.tran = 3;
    theData.batt = 0;
    theData.value1 = 0;
    theData.value2 = 0;
    theData.value3 = 0;
    theData.value4 = 0;
    #ifdef DEBUG_MIN
    Serial.print(" ..Node = ");
    Serial.print(theData.node);
    Serial.print(", Tran = ");
    Serial.println(theData.tran);
    #endif
  }   
   
  if (radio.sendWithRetry(theNodeID, (const void*)(&theData), sizeof(theData)), 1) { // node must be a byte
    Serial.println("OK ... temp reset message sent ok");
    tResetRequested[nID] = 0;
    } else {
      #ifdef DEBUG_MIN
      Serial.println("Fail ... temp reset message NOT sent");
      #endif
      //tResetRequested[theNodeID - 1] = 1;
      tResetRequested[nID] = 1;
    }
  //**}

  #ifdef DEBUG_ON          
  for (int i = 0; i < numNodes; i++) {
    Serial.print("tResetRequested [node ");
    Serial.print(i + 1);
    Serial.print("] = ");
    Serial.println(tResetRequested[i]);
  }
   #endif
    
    
  #ifdef DEBUG_ON          
  for (int i = 0; i < numNodes; i++) {
    Serial.print("lastData [node ");
    Serial.print(i + 1);
    Serial.print("] = ");
    Serial.println(lastData[i]);
  }
  #endif
   
   
  #ifdef DEBUG_MIN  
  mem1 = System.freeMemory();
  Serial.print("Free memory=");
  Serial.println(mem1);
  #endif

  #ifdef DEBUG_ON
  Serial.println();
  #endif    

  }
}
// **** END LOOPP ****



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



void connect() {
  if (Particle.connected() == false) {
    Particle.connect();
  }
}


/*
void checkButton()
{
  // Update button state
  button1.Update();

  // Save click codes in LEDfunction, as click codes are reset at next Update()
  if (button1.clicks != 0) function = button1.clicks;
  
  if(button1.clicks == 1) Serial.println("SINGLE click");

  if(function == 2) Serial.println("DOUBLE click");

  if(function == 3) Serial.println("TRIPLE click");

  if(function == -1) Serial.println("SINGLE LONG click");

  if(function == -2) Serial.println("DOUBLE LONG click");

  if(function == -3) Serial.println("TRIPLE LONG click");
  SW1State = function;
  function = 0;
  delay(5);
}
*/
