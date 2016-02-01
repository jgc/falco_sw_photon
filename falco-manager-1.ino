// WARNING Initial hack !!!!!

// **** TODO ****
// Fix rounding of WIRE data
// Add individual node web reset particle function
// Add function to add nodes at startup
// Add function to indicate missing data after say 5 minutes
// Fix counter block to sending  data


#include "application.h"
#include "RFM69.h"
#include "RFM69registers.h"
#define  OTHER_ADDRESS 0x04 // 

PRODUCT_VERSION(2);
PRODUCT_ID(162);

//#define RANDOMDATA
//#define DEBUG_ON        //DEBUG is defined so cannot use)
#define DEBUG_MIN

#define serialTitle "\n\nswitch-led-test_1\n*****************\n"
#define sVersion "20150724_03"
#define hVersion "20150724_01"
#define hDesc "FalcoA Monitor - Prototype"

#define led_delay 500

unsigned int lastPublish = 0;

#define NODEID          31   
#define NETWORKID       100  
#define FREQUENCY       RF69_868MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY      "sampleEncryptKey" 
#define IS_RFM69HW      //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME        30 // max # of ms to wait for an ack
#define SERIAL_BAUD     57600
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



//**** START SETUP ****
void setup() {
  Particle.function("resetWifi", cloudResetWifi);
  Particle.function("resetReboots", cloudResetReboots);  
  //Particle.variable("sVersion", sVersion, STRING);
  //Particle.variable("hVersion", hVersion, STRING);
  //Particle.variable("hDesc", hDesc, STRING);
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
  
  Serial.begin(SERIAL_BAUD);
  delay(10);
 
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
  
  SW_startup();
  
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
  valueSU = "te|1|3|Startup at " + String(Time.timeStr());
     
  Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
  Wire.write(valueSU);
  delay(100);  // try to fix i2c issue
  Wire.endTransmission(true);    // stop transmitting
  delay(100);  // try to fix i2c issue
  
  mem1 = System.freeMemory();
  Serial.println(mem1);
  
  if (radio.sendWithRetry(theNodeID, "Hi", 2)) //target node Id, message as string or byte array, message length
      Serial.println("Hi received");
      
  // REMOVE ONCE FIX ADDED
  for (int i = 0; i < numNodes; i++) {
    nodePresent[i] = 1;
    #ifdef DEBUG_MIN          
    Serial.print("tResetRequested [node ");
    Serial.print(i + 1);
    Serial.print("] = ");
    Serial.println(tResetRequested[i]);
    #endif
  }
  
  nodeDataMissing[4] = true;
  
  
  
}
// End setup



void loop() {

  /*  Add timer else too slow
  for (int i = 0; i < numNodes; i++) {
    if (nodeDataMissing[i])
    {
      String valueX = "md|" + String(i) + "|0|0|0|0|0";
      //[tran|node|batt|value1|value2|value3|tFlag]
      Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
      Wire.write(valueX);
      Wire.endTransmission(true);    // stop transmitting
      #ifdef DEBUG_MIN  
      Serial.print(Time.timeStr());
      Serial.print(" [");
      Serial.print(valueX);
      Serial.println("]");
      #endif
    }
  }
  */
  
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

    #ifdef DEBUG_MIN  
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
        #ifdef DEBUG_MIN          
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
    Serial.print("\nAt ");
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
      Serial.print("] Invalid payload received, not matching Payload struct!");
      #endif
    }
    else
    {
    theData = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
    #ifdef DEBUG_ON
    Serial.print("\nNode=");
    Serial.println(theData.node);
    #endif
    
    theNodeID = theData.node;
    lastData[theNodeID - 1] = Time.now();   
    
    #ifdef RANDOMDATA
    uint8_t nodeRandomizer;
    nodeRandomizer = random(0,2);
    node = node + nodeRandomizer;
    #endif
    
    #ifdef DEBUG_ON  
    Serial.print("tran=");
    Serial.println(theData.tran);
    #endif 
     
    // ***************** 
    // ADD: Roundup and down logic
    //rndUpDn(n10, integerDecPlaces1, roundedDecPlaces1);
    // FIX FIX FIX - add loop ete etc
      
    //float battF = theData.batt;
    //battF = battF/100;
    //float battF2 = theData.batt;
    //battF2 = battF2/10;
    //int batt1 = battF;
    //int batt2 = battF2;

    int batt1 = rndInt(theData.batt)/100;
    int batt2 = rndInt(theData.batt)/10;
    int temp1 = rndInt(theData.value1)/10;
    int temp2 = rndInt(theData.value1);
    int tMin = rndInt(theData.value2)/10;
    int tMax = rndInt(theData.value3)/10;
    
    int tFlag = 0;
    if ((temp1 > 200) || (temp1 < 40)) tFlag = 1;
      
    #ifdef DEBUG_ON  
    Serial.print("\nWire: ");
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
        Serial.print(temp1);
        Serial.print("|");
        Serial.print(tMin);
        Serial.print("|");
        Serial.print(tMax);
        Serial.print("|");
      }
      
      Serial.println();
      
      // displayThermometer(int layout, int tval, int tmin, int tmax, int tx, int ty, int flag)
      String value1 = "{ \"node\": \"" + String(theData.node) + "\", \"volts\": \"" + String(batt2) + "\", \"temp1\": \"" + String(temp2) + "\", \"sVersion\": \"" + String(SW1Counter) + "\"  }";
      String value2 = "dt|" + String(theNodeID) + "|" + String(batt1) + "|" + String(temp1) + "|" + String(tMin) + "|" + String(tMax)  + "|" + String(tFlag);
      
      #ifdef DEBUG_ON
      Serial.println("Pinging started...");
      #endif
      numberOfReceivedPackage = 0;
      numberOfReceivedPackage = WiFi.ping(remoteIP);
      #ifdef DEBUG_ON
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
        Serial.println(" - ACK sent.");
    } else {
        Serial.println(" - NO ACK sent.");
    }
  
  #ifdef DEBUG_ON
  Serial.print("tResetRequested = ");
  Serial.println(tResetRequested[theNodeID - 1]);
  #endif
  
  radio.receiveDone(); //put radio in RX mode
  //Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
  
  #ifdef DEBUG_ON 
  Serial.print("theNodeID = ");
  Serial.println(theNodeID);
  #endif
  
  //delay(10);
  delay(2); // seems to give best results with reset
  //resetTemp = 1;
  
  if (tResetRequested[theNodeID - 1] == 1) {
    theData.node = theNodeID;
    theData.tran = 99;
    theData.batt = 0;
    theData.value1 = 0;
    theData.value2 = 0;
    theData.value3 = 0;
    theData.value4 = 0;
   
  if (radio.sendWithRetry(theNodeID, (const void*)(&theData), sizeof(theData), 2)) { // node must be a byte
    Serial.println("OK ... temp reset message sent ok");
    tResetRequested[theNodeID - 1] = 0;
    } else {
      Serial.println("Fail ... temp reset message NOT sent");
      tResetRequested[theNodeID - 1] = 1;
    }
  }

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



void SW_startup() {
  Serial.begin(57600);

#ifdef DEBUG_ON  
  Serial.println(sVersion);
  Serial.println(serialTitle);
#endif

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  pinMode(SW1, INPUT_PULLDOWN);
  
  delay(2000);
  digitalWrite(LED, LOW);
  delay(2000);

#ifdef DEBUG_MIN
  Time.zone(0);
  Serial.println(Time.timeStr());
  Serial.println("Switch LED test - startup completed\n");
#endif
  Wire.begin();
}



bool readSW(int SW) {
  bool SWState = 0;
  if(digitalRead(SW)) {
    delay(50);
    if(digitalRead(SW))
      return SWState = 1;
  } else 
      return SWState = 0;

}



int rndUpDn(int in1, int integerDecPlaces, int roundedDecPlaces) {
  // assumes 2 decimal places
  //Serial.println("\n------");
  int divFactor = integerDecPlaces/roundedDecPlaces;
  //Serial.print("divFactor = ");
  //Serial.println(divFactor);
  int roundingFactor = divFactor/10;
  if (divFactor == 1) roundingFactor = 0;
  //Serial.print("roundingFactor = ");
  //Serial.println(roundingFactor);
  
  if (in1 != 9999) {
    int round1 = in1 / divFactor;
    //Serial.print("round1 = ");
    //Serial.println(round1);
    int round2;
    int round3;
    if (in1 < 0) {
      round2 = (in1 - (5 * roundingFactor)) / divFactor;
      //Serial.print("round2 = ");
      //Serial.println(round2);
      round3 = (in1 / divFactor) - 1;
      //Serial.print("round3 = ");
      //Serial.println(round3);
    } else {
      round2 = (in1 + (5 * roundingFactor)) / divFactor;
      //Serial.print("round2 = ");
      //Serial.println(round2);
      round3 = (in1 / divFactor) + 1;    
      //Serial.print("round3 = ");
      //Serial.println(round3);
    }
     //Serial.println("------\n"); 
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
