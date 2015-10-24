#include "application.h"
#include "RFM69.h"
#include "RFM69registers.h"
#define  OTHER_ADDRESS 0x04 // 

PRODUCT_VERSION(2);
PRODUCT_ID(162);

//#define RANDOMDATA

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

int ledState = HIGH; 
int lastButtonState = LOW;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

byte ackCount=0;

int SW1Counter = 0;
int SW1State = 0;
uint8_t val = 0;

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



void setup() {
  Particle.variable("sVersion", sVersion, STRING);
  Particle.variable("hVersion", hVersion, STRING);
  Particle.variable("hDesc", hDesc, STRING);
  Particle.variable("SW1State", &SW1State, INT);
  Particle.variable("SW1Counter", &SW1Counter, INT);
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

  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  Serial.print("Promiscuous mode = ");
  Serial.println(promiscuousMode);
  Serial.print("Power level = ");
  Serial.println(highPower);

  randomSeed(newSeed);
  
  SW_startup();
  
  Blink(LED,1000);

  val =  EEPROM.read(1);
  Serial.println(val);
  //if (val != 0) EEPROM.write(1, 0);
  //val =  EEPROM.read(1);
  //Serial.println(val);
  mem1 = System.freeMemory();
  Serial.println(mem1);
  
  Time.zone(0);
  Serial.println(Time.timeStr());
  
  String valueSU;
  valueSU = "te|1|3|Startup at " + String(Time.timeStr());
     
  Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
  Wire.write(valueSU);
  Wire.endTransmission(true);    // stop transmitting
  
}



void loop() {

  SW1State = readSW(SW1);
  switchCount();

  if (SW1Counter >= 10){
    Serial.println("\n\n");
    String value3 = "rt|999|0|0|0|0|0";
    Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
    Wire.write(value3);
    Wire.endTransmission(true);    // stop transmitting
    SW1Counter = 0;
    Serial.println(value3);
  }
      
  if (radio.receiveDone())
  {
    Serial.println("\n\n");
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    Serial.print(" [RX_RSSI:");Serial.print(radio.readRSSI());Serial.print("]");
    if (promiscuousMode)
    {
      Serial.print(" to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
    }

    if (radio.DATALEN != sizeof(Payload))
    {
      Serial.print("[Payload = ");
      Serial.print(sizeof(Payload));
      Serial.print(" / radio.DATALEN = ");
      Serial.print(radio.DATALEN);
      Serial.print("] Invalid payload received, not matching Payload struct!");
    }
    else
    {
    theData = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
      
    Serial.print("\nNode=");
    Serial.println(theData.node);
    uint8_t node = theData.node;
    
    #ifdef RANDOMDATA
    uint8_t nodeRandomizer;
    nodeRandomizer = random(0,2);
    node = node + nodeRandomizer;
    #endif
      
    Serial.print("tran=");
    Serial.println(theData.tran);
     
     
    // ***************** 
    // ADD: Roundup and down logic
    //rndUpDn(n10, integerDecPlaces1, roundedDecPlaces1);
    // FIX FIX FIX - add loop ete etc
      
    float battF = theData.batt;
    battF = battF/100;
    float battF2 = theData.batt;
    battF2 = battF2/10;
    int batt1 = battF;
    int batt2 = battF2;
    
      
    float tempF = theData.value1;
    tempF = tempF/10;
    float tempF2 = theData.value1;
    tempF2 = tempF2/10;
    int temp1 = tempF;
    int temp2 = tempF2;
    
    float tminF2 = theData.value2;
      tminF2 = tminF2/10;
      int tMin = tminF2;
      //int tmin2 = tminF2;
      
      float tmaxF2 = theData.value3;
      tmaxF2 = tmaxF2/10;
      int tMax = tmaxF2;
      //int tmax2 = tmaxF2;
      
      int tFlag = 0;
      if ((temp1 > 200) || (temp1 < 40)) tFlag = 1;
      
      Serial.print("\Wire: ");
      Serial.println("[tran|node|batt|value1|value2|value3|tFlag] = ");
      Serial.print("[dt|");
      Serial.print(node);
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

      // displayThermometer(int layout, int tval, int tmin, int tmax, int tx, int ty, int flag)
      String value1 = "{ \"node\": \"" + String(node) + "\", \"volts\": \"" + String(batt2) + "\", \"temp1\": \"" + String(temp2) + "\", \"sVersion\": \"" + String(SW1Counter) + "\"  }";
      String value2 = "dt|" + String(node) + "|" + String(batt1) + "|" + String(temp1) + "|" + String(tMin) + "|" + String(tMax)  + "|" + String(tFlag);
      Particle.publish("temperature1", value1, 60, PRIVATE);
      //Serial.println("Temperature published: [" + value + "]");
      Serial.println("Temperature published");
      
      Wire.beginTransmission(OTHER_ADDRESS); // transmit to slave device #4
      Wire.write(value2);
      Wire.endTransmission(true);    // stop transmitting

      mem1 = System.freeMemory();
      Serial.print("Free memory=");
      Serial.println(mem1);
      
  }
    
   if (theData.tran == 10 && SW1Counter > 2)
   {
      uint8_t theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - trans response sent.");
      SW1Counter = 0;
   }
  
   if (radio.ACK_REQUESTED)
   {
      int16_t theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++%3==0)
      {
        Serial.print(" Pinging node ");
        Serial.print(theNodeID);
        Serial.print(" - ACK...");
        delay(3); //need this when sending right after reception .. ?
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
          Serial.print("ok!");
        else Serial.print("nothing");
      }
    }
    Serial.println();
    //Blink(LED,3);
  }
}




void Blink(byte PIN, int DELAY_MS)
{
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}



void tempResponse(const char *name, const char *data) {
    String responseStr = String(data);
    responseStr.replace("\"", "");
    Serial.println("Web response: [" + responseStr + "]");
}



void switchCount(){
  if (SW1State == 1) {
    ++SW1Counter;
    //int addr = 1;
    val = SW1Counter;
    EEPROM.write(1, val);

    digitalWrite(LED, HIGH);
    Serial.print("Switch counter = ");
    Serial.println(SW1Counter);
    delay(250);
    digitalWrite(LED, LOW);
  }
}



void SW_startup() {
  Serial.begin(57600);
  Serial.println(sVersion);
  Serial.println(serialTitle);
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  pinMode(SW1, INPUT_PULLDOWN);
  
  delay(2000);
  digitalWrite(LED, LOW);
  delay(2000);

  Serial.println("Switch LED test - startup completed\n");
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
      #ifdef DEBUG
      Serial.print("Result = ");
      Serial.println(round1);
      #endif
      return round1;
    }
    else {     
      #ifdef DEBUG
      Serial.print("Result = ");
      Serial.println(round3);
      #endif
      return round3;
    }
    
  } else {
    return in1; // ie 9999
  }
}
