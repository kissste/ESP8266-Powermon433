/*
 ESP8266-Powermon433
 Forked from scruss/Powermon433

 Monitoring 433MHz power monitor products (both the same internally)
 -- Black and Decker EM100B 
 -- BlueLine PowerCost Monitor


 Edited by kissste as follows:
 Ported to ESP8266
 Added UDP messaging
 Added Temperature and Humidity
 Using CC1101

 Use Arduino IDE
*/

#ifdef WIFIMANAGER
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#endif

#include <SPI.h>
#include "ESP8266-Powermon433.h"
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <WiFiClient.h>
#include <DNSServer.h>

#ifdef WEBSERVER
#include <ESP8266WebServer.h>
#endif

//#include <ESP8266mDNS.h>
#include <WifiUDP.h>

#ifdef WIFIMANAGER
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#endif

//define your default values here, if there are different values in config.json, they are overwritten.
char udp_server[40] = "yourserver.com";
char udp_port[6] = "5005";
char udp_token[34] = "";

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

struct observation {
  uint16_t WattHours;
  int8_t Temperature;
}

observations[10000];

#ifdef WIFIMANAGER
#else
const char* ssid = "yourSSID";
const char* password = "yourpassword";
#endif
uint16_t counter = 0;

#ifdef WEBSERVER
ESP8266WebServer server(80);
#endif

// the pin connected to the receiver output
#define DPIN_OOK_RX 5 

// pir
#define DPIN_PIR 6 

// am2303
#include "DHT.h"
#define DHTPIN 2 //0,4
#define DHTTYPE DHT22 

DHT dht(DHTPIN, DHTTYPE, 30);

#define RADIOCC110
#include "temp_lerp.h"

/*
 The default ID of the transmitter to decode from
 - it is likely that yours is different.
 see README.md on how to check yours and set it here.
 */
/*#define DEFAULT_TX_ID 0xfff8*/
#define DEFAULT_TX_ID 0x582c //your code
/*#define DEFAULT_TX_ID 0x7996*/
/*#define DEFAULT_TX_ID 0xD7D6*/

/*
 TX_ID_LOCK - 
 Uncomment this #define if you want to lock the decoder ID.
 This will prevent nearby stations hijacking your power log
 */
//#define TX_ID_LOCK

/*
 TEMPERATURE_F - 
 Uncomment this #define if you believe in the power of
 D. G. Farenheit's armpit, and want your temperature in °F.
 Otherwise, you get °C, like you should.
 [I'd include an option for K, but it doesn't fit into a byte.]
 */
//#define TEMPERATURE_F

static uint16_t g_TxId;
static uint16_t g_TxIdNew;

static struct tagDecoder {
  uint8_t state;
  uint8_t pos;
  uint8_t bit;
  uint8_t data[4];
} 
decoder;

static int8_t g_RxTemperature = -100;
static uint8_t g_RxFlags;
static uint16_t g_RxWatts;
static uint16_t g_RxWattHours;

// Watt-hour counter rolls over every 65.536 kWh, 
// or roughly 2½ days of my use.
// This should be good for > 4 GWh; enough for everyone ...
static unsigned long g_TotalRxWattHours;
// need this too
static uint16_t g_PrevRxWattHours;

// better stats on time between reports
// delta is long as packets are appx ½ range of uint16_t
// so we roll if we miss >2
static unsigned long g_PrintTime_ms;
static unsigned long g_PrevPrintTime_ms;
static unsigned long g_PrintTimeDelta_ms;

static bool g_RxDirty;
static uint32_t g_RxLast;
static uint8_t g_RxRssi;

volatile uint16_t pulse_433;
volatile uint16_t pulse_PIR;

void pinChangeOOK_RX(void) {
  static uint16_t last_433;

  uint16_t now = micros();
  uint16_t cnt = now - last_433;
  if (cnt > 10)
    pulse_433 = cnt;

  last_433 = now;
}

void pinChangePIR(void) {
  pulse_PIR = micros();
}

void setupPinChangeInterrupt() {
  // OOK_RX
  pinMode(DPIN_OOK_RX, INPUT);
  attachInterrupt(digitalPinToInterrupt(DPIN_OOK_RX), pinChangeOOK_RX, CHANGE);
  // PIR
  pinMode(DPIN_PIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(DPIN_PIR), pinChangePIR, RISING);  
}

// Short burst in uSec
#define OOK_TX_SHORT 500
// Long burst in uSec
#define OOK_TX_LONG  1000
// Inter-packet delay in msec
#define OOK_TX_DELAY 65
// Inter-ID-packet delay in msec
#define OOK_ID_DELAY 225

#define OOK_PACKET_INSTANT 1
#define OOK_PACKET_TEMP    2
#define OOK_PACKET_TOTAL   3

#define PIN_SS 15
uint8_t _slaveSelectPin = PIN_SS;
uint8_t _interruptPin = 3;

typedef enum {
  RHModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
  RHModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
  RHModeIdle,             ///< Transport is idle.
  RHModeTx,               ///< Transport is in the process of transmitting a message.
  RHModeRx                ///< Transport is in the process of receiving a message.
} RHMode;

RHMode _mode;

// Low level commands for interfacing with the device
uint8_t spiCommand(uint8_t command) {
    uint8_t status;
    //ATOMIC_BLOCK_START;
    digitalWrite(_slaveSelectPin, LOW);
    status = SPI.transfer(command);
    digitalWrite(_slaveSelectPin, HIGH);
    //ATOMIC_BLOCK_END;
    return status;
}

uint8_t spiRead(uint8_t reg) {
    uint8_t val;
    //ATOMIC_BLOCK_START;
    digitalWrite(_slaveSelectPin, LOW);
    SPI.transfer(reg); // Send the address, discard the status
    val = SPI.transfer(0); // The written value is ignored, reg value is read
    digitalWrite(_slaveSelectPin, HIGH);
    //ATOMIC_BLOCK_END;
    return val;
}

uint8_t spiWrite(uint8_t reg, uint8_t val) {
    uint8_t status = 0;
    //ATOMIC_BLOCK_START;
    digitalWrite(_slaveSelectPin, LOW);
    status = SPI.transfer(reg); // Send the address
    SPI.transfer(val); // New value follows
    #if (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(__arm__) && defined(CORE_TEENSY)
    // Sigh: some devices, such as MRF89XA dont work properly on Teensy 3.1:
    // At 1MHz, the clock returns low _after_ slave select goes high, which prevents SPI
    // write working. This delay gixes time for the clock to return low.
      delayMicroseconds(5);
    #endif
    digitalWrite(_slaveSelectPin, HIGH);
    //ATOMIC_BLOCK_END;
    return status;
}

uint8_t spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len) {
    uint8_t status = 0;
    //ATOMIC_BLOCK_START;
    digitalWrite(_slaveSelectPin, LOW);
    status = SPI.transfer(reg); // Send the start address
    while (len--)
      *dest++ = SPI.transfer(0);
    digitalWrite(_slaveSelectPin, HIGH);
    //ATOMIC_BLOCK_END;
    return status;
}

uint8_t spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len) {
    uint8_t status = 0;
    //ATOMIC_BLOCK_START;
    digitalWrite(_slaveSelectPin, LOW);
    status = SPI.transfer(reg); // Send the start address
    while (len--)
      SPI.transfer(*src++);
    digitalWrite(_slaveSelectPin, HIGH);
    //ATOMIC_BLOCK_END;
    return status;
}

void setSlaveSelectPin(uint8_t slaveSelectPin) {
    _slaveSelectPin = slaveSelectPin;
}

uint8_t spiReadRegister(uint8_t reg) {
    return spiRead((reg & 0x3f) | RH_CC110_SPI_READ_MASK);
}

uint8_t spiBurstReadRegister(uint8_t reg) {
    return spiRead((reg & 0x3f) | RH_CC110_SPI_READ_MASK | RH_CC110_SPI_BURST_MASK);
}

uint8_t spiWriteRegister(uint8_t reg, uint8_t val) {
    return spiWrite((reg & 0x3f), val);
}

uint8_t spiBurstWriteRegister(uint8_t reg, const uint8_t* src, uint8_t len) {
    return spiBurstWrite((reg & 0x3f) | RH_CC110_SPI_BURST_MASK, src, len);
}

bool printRegisters() {
    uint8_t i;
    for (i = 0; i <= 0x2f; i++) {
      Serial.print(i, HEX);
      Serial.print(F(": "));
      Serial.print(spiReadRegister(i), HEX);
      Serial.print(F("; "));
    }
    Serial.println("");
    // Burst registers
    for (i = 0x30; i <= 0x3e; i++) {
      Serial.print(i, HEX);
      Serial.print(F(": "));
      Serial.print(spiBurstReadRegister(i), HEX);
      Serial.print(F("; "));
    }
    Serial.println("");
    return true;
}

int16_t getLastRssi() {
  uint8_t rssi_dec;
  int16_t rssi_dBm;
  uint8_t rssi_offset = 74;
  
  rssi_dec = spiBurstReadRegister(RH_CC110_REG_34_RSSI);
  if (rssi_dec >= 128)
    rssi_dBm = (int16_t)((int16_t)( rssi_dec - 256) / 2) - rssi_offset;
  else
    rssi_dBm = (rssi_dec / 2) - rssi_offset; 
  return rssi_dBm;
}

void setModeRx() {
    if (_mode != RHModeRx){
      // Radio is configured to stay in RX mode
      // only receipt of a CRC_OK wil cause us to return it to IDLE
      spiCommand(RH_CC110_STROBE_34_SRX);
      _mode = RHModeRx;
    }
}

bool cc110_init() {
    // Reset the chip
    // Strobe the reset
    uint8_t val = spiCommand(RH_CC110_STROBE_30_SRES); // Reset
    //Serial.println(val);
    delay(100);
    val = spiCommand(RH_CC110_STROBE_36_SIDLE); // IDLE
    //Serial.println(val);
    if (val != 0x0f)
      return false; // No chip there or reset failed.

    // Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
    // ARM M4 requires the below. else pin interrupt doesn't work properly.
    // On all other platforms, its innocuous, belt and braces
    pinMode(_interruptPin, INPUT); 

    spiWriteRegister(RH_CC110_REG_02_IOCFG0, RH_CC110_GDO_CFG_CRC_OK_AUTORESET);  // gdo0 interrupt on CRC_OK
    spiWriteRegister(RH_CC110_REG_06_PKTLEN, RH_CC110_MAX_PAYLOAD_LEN); // max packet length
    spiWriteRegister(RH_CC110_REG_07_PKTCTRL1, RH_CC110_CRC_AUTOFLUSH); // no append status, crc autoflush, no addr check
    spiWriteRegister(RH_CC110_REG_08_PKTCTRL0, RH_CC110_PKT_FORMAT_NORMAL | RH_CC110_CRC_EN | RH_CC110_LENGTH_CONFIG_VARIABLE);
    spiWriteRegister(RH_CC110_REG_13_MDMCFG1, RH_CC110_NUM_PREAMBLE_4); // 4 preamble bytes, chan spacing not used
    spiWriteRegister(RH_CC110_REG_17_MCSM1, RH_CC110_CCA_MODE_RSSI_PACKET | RH_CC110_RXOFF_MODE_RX | RH_CC110_TXOFF_MODE_IDLE);
    spiWriteRegister(RH_CC110_REG_18_MCSM0, RH_CC110_FS_AUTOCAL_FROM_IDLE | RH_CC110_PO_TIMEOUT_64); // cal when going to tx or rx
    spiWriteRegister(RH_CC110_REG_20_WORCTRL, 0xfb); // from smartrf
    spiWriteRegister(RH_CC110_REG_29_FSTEST, 0x59); // from smartrf
    spiWriteRegister(RH_CC110_REG_2A_PTEST, 0x7f); // from smartrf
    spiWriteRegister(RH_CC110_REG_2B_AGCTEST, 0x3f); // from smartrf

    // Set some reasonable default values
    //uint8_t syncWords[] = { 0xd3, 0x91 };
    //setSyncWords(syncWords, sizeof(syncWords));
    //setTxPower(TransmitPower5dBm);
    //setFrequency(433.85);
    //setModemConfig(GFSK_Rb1_2Fd5_2);
    return true;  
}

#ifdef WEBSERVER
void handleRoot() {
  //digitalWrite(led, 1);
  server.send(200, F("text/plain"), String(F("hello from esp8266!\nFree Heap: ")) + String(ESP.getFreeHeap()));
  //digitalWrite(led, 0);
}

void handleNotFound(){
  //digitalWrite(led, 1);
  String message = F("File Not Found\n\n");
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  //digitalWrite(led, 0);
}
#endif

bool str_to_uint16(const char *str) {
    char *end;
    int errno = 0;
    long val = strtol(str, &end, 10);
    if (errno || end == str || *end != '\0' || val < 0 || val >= 0x10000) {
        return false;
    }
    return (uint16_t)val;
}

void UDPSend(String msg) {
  counter++;
  WiFiUDP Udp;
  int out = 0;
  out = Udp.beginPacket(udp_server, str_to_uint16(udp_port));
  Udp.print(ESP.getChipId(),HEX);
  Udp.write(",");
  Udp.print(WiFi.localIP());
  Udp.write(",");
  Udp.print(counter);
  Udp.write(",");  
  Udp.println(msg);
  out = Udp.endPacket();  
}

#ifdef WIFIMANAGER
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println(F("Entered config mode"));
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}
#endif

void setup() {
  Serial.begin(115200);

  Serial.print(F("\n#,Powermon433SK built "));
  Serial.print(F(__DATE__));
  Serial.print(F(" "));
  Serial.println(F(__TIME__));

#ifdef WIFIMANAGER
    Serial.println(F("#,mounting FS..."));
  
    if (SPIFFS.begin()) {
      Serial.println(F("#,mounted file system"));
      if (SPIFFS.exists("/config.json")) {
        //file exists, reading and loading
        Serial.println(F("#,reading config file"));
        File configFile = SPIFFS.open("/config.json", "r");
        if (configFile) {
          Serial.println(F("#,opened config file"));
          size_t size = configFile.size();
          // Allocate a buffer to store contents of the file.
          std::unique_ptr<char[]> buf(new char[size]);
  
          configFile.readBytes(buf.get(), size);
          DynamicJsonBuffer jsonBuffer;
          JsonObject& json = jsonBuffer.parseObject(buf.get());
          Serial.print(F("#,"));
          json.printTo(Serial);
          if (json.success()) {
            Serial.println(F("\n#,parsed json"));
  
            strcpy(udp_server, json["udp_server"]);
            strcpy(udp_port, json["udp_port"]);
            strcpy(udp_token, json["udp_token"]);
  
          } else {
            Serial.println(F("#,failed to load json config"));
          }
        }
      }
    } else {
      Serial.println(F("#,failed to mount FS"));
    }  
  
    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length
    WiFiManagerParameter custom_udp_server("server", "UDP server", udp_server, 40);
    WiFiManagerParameter custom_udp_port("port", "UDP port", udp_port, 5);
    WiFiManagerParameter custom_udp_token("token", "UDP token", udp_token, 32);
    
    WiFiManager wifiManager;
    //reset settings - for testing
    //wifiManager.resetSettings();
      
    //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);
  
    //add all your parameters here
    wifiManager.addParameter(&custom_udp_server);
    wifiManager.addParameter(&custom_udp_port);
    wifiManager.addParameter(&custom_udp_token);
  
    if(!wifiManager.autoConnect("S101", "k0k1tk2")) {
      Serial.println(F("#,failed to connect and hit timeout"));
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    } 
  
    //if you get here you have connected to the WiFi
    Serial.println(F("#,connected...yeey :)"));
  
    //read updated parameters
    strcpy(udp_server, custom_udp_server.getValue());
    strcpy(udp_port, custom_udp_port.getValue());
    strcpy(udp_token, custom_udp_token.getValue());
  
    //save the custom parameters to FS
    if (shouldSaveConfig) {
      Serial.println(F("saving config"));
      DynamicJsonBuffer jsonBuffer;
      JsonObject& json = jsonBuffer.createObject();
      json["udp_server"] = udp_server;
      json["udp_port"] = udp_port;
      json["udp_token"] = udp_token;
  
      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println(F("#,failed to open config file for writing"));
      }
  
      json.printTo(Serial);
      json.printTo(configFile);
      configFile.close();
    }
  
#else
  // Connecting to Wifi without the connection manager
    WiFi.begin(ssid, password);
    Serial.println("");
  
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print(F("#,Connected to "));
    Serial.println(ssid);
    Serial.print(F("#,IP address: "));
    Serial.println(WiFi.localIP());
  
  //  if (MDNS.begin("esp8266")) {
  //    Serial.println(F("MDNS responder started"));
  //  }
    
#endif

#ifdef WEBSERVER
    server.on("/", handleRoot);
  
    server.on("/inline", [](){
      server.send(200, "text/plain", "this works as well");
    });
  
    server.onNotFound(handleNotFound);
  
    server.begin();
    Serial.println(F("#,HTTP server started"));
#endif

  dht.begin();  
  
  Serial.print(F("#,Listening for Sensor ID: 0x"));
  Serial.println(DEFAULT_TX_ID, HEX);
  
  yield();

  UDPSend("Start");
  
  #ifdef RADIOCC110
  if(0) {
    Serial.println(F("#,C110x init try"));
    // SPI uint8_t miso = 12, uint8_t mosi = 11, uint8_t sck = 13, ss= 10
    pinMode(PIN_SS, OUTPUT);  
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    SPI.endTransaction();
    uint8_t i = 0;
    while (!cc110_init()) {
      i++;
      Serial.print(F("#,C110x init failed, "));
      Serial.println(i);
      delay(1000);
    }
    Serial.println(F("#,C110x init OK"));
    spiWriteRegister(RH_CC110_REG_02_IOCFG0, RH_CC110_GDO_CFG_SDO );  // 0x0D
    spiWriteRegister(RH_CC110_REG_08_PKTCTRL0, RH_CC110_PKT_FORMAT_ASYNC_SERIAL | RH_CC110_LENGTH_CONFIG_INFINITE); //0x32
    
    spiWriteRegister(RH_CC110_REG_0B_FSCTRL1, 0x06); //Frequency Synthesizer Control
    spiWriteRegister(RH_CC110_REG_0C_FSCTRL0, 0x00);
    
    spiWriteRegister(RH_CC110_REG_0D_FREQ2, 0x10);   //Frequency Control Word, High Byte
    spiWriteRegister(RH_CC110_REG_0E_FREQ1, 0xB0);   //Frequency Control Word, Middle Byte
    spiWriteRegister(RH_CC110_REG_0F_FREQ0, 0x3F);   //Frequency Control Word, Low Byte   
    
    spiWriteRegister(RH_CC110_REG_10_MDMCFG4, 0xF6); //Modem Configuration
    spiWriteRegister(RH_CC110_REG_11_MDMCFG3, 0x43); //Modem Configuration
    spiWriteRegister(RH_CC110_REG_12_MDMCFG2, 0x37); //Modem Configuration
    spiWriteRegister(RH_CC110_REG_13_MDMCFG1, 0x00); //Modem Configuration
    spiWriteRegister(RH_CC110_REG_14_MDMCFG0, 0x00); 
    spiWriteRegister(RH_CC110_REG_15_DEVIATN, 0x40); //Modem Deviation Setting
    spiWriteRegister(RH_CC110_REG_19_FOCCFG, 0x16);  //Frequency Offset Compensation Configuration
    spiWriteRegister(RH_CC110_REG_1A_BSCFG, 0x6C);
    spiWriteRegister(RH_CC110_REG_1B_AGCCTRL2, 0x43);//AGC Control
    spiWriteRegister(RH_CC110_REG_1C_AGCCTRL1, 0x40);
    spiWriteRegister(RH_CC110_REG_1D_AGCCTRL0, 0x91);
    spiWriteRegister(RH_CC110_REG_21_FREND1, 0x56);
    spiWriteRegister(RH_CC110_REG_22_FREND0, 0x11);  //Front End TX Configuration
    spiWriteRegister(RH_CC110_REG_23_FSCAL3, 0xE9);  //Frequency Synthesizer Calibration
    spiWriteRegister(RH_CC110_REG_24_FSCAL2, 0x2A);  //Frequency Synthesizer Calibration
    spiWriteRegister(RH_CC110_REG_25_FSCAL1, 0x00);  //Frequency Synthesizer Calibration
    spiWriteRegister(RH_CC110_REG_26_FSCAL0, 0x1F);  //Frequency Synthesizer Calibration
    spiWriteRegister(RH_CC110_REG_2C_TEST2, 0x81);   //Various Test Settings
    spiWriteRegister(RH_CC110_REG_2D_TEST1, 0x35);   //Various Test Settings
    spiWriteRegister(RH_CC110_REG_2E_TEST0, 0x09);   //Various Test Settings
    printRegisters(); 
    //Serial.println("a");
    setModeRx();
    //Serial.println("b");  
    #endif
    
    // RX
    setupPinChangeInterrupt();
    Serial.print(F("#,Setup completed"));
  }

  yield();
  
  if(1) {
    Serial.print(F("#,dht measure"));
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F(", "));
    Serial.print(F("Temperature: "));
    Serial.print(t);
    Serial.print(F(" *C "));
  }
}

/* crc8 from chromimum project */
__attribute__((noinline)) uint8_t crc8(uint8_t const *data, uint8_t len) {
  uint16_t crc = 0;
  for (uint8_t j=0; j<len; ++j)
  {
    crc ^= (data[j] << 8);
    for (uint8_t i=8; i>0; --i)
    {
      if (crc & 0x8000)
        crc ^= (0x1070 << 3);
      crc <<= 1;
    }
  }
  return crc >> 8;
}

void resetDecoder(void) {
  decoder.pos = 0;
  decoder.bit = 0;
  decoder.state = 0;
}

void decoderAddBit(uint8_t bit) {
  decoder.data[decoder.pos] = (decoder.data[decoder.pos] << 1) | bit;
  if (++decoder.bit > 7) {
    decoder.bit = 0;
    if (++decoder.pos >= sizeof(decoder.data))
      resetDecoder();
  }
}

bool decodeRxPulse(uint16_t width) {
  // 500,1000,1500 usec pulses with 25% tolerance
  if (width > 375 && width < 1875) {
    // The only "extra long" long signals the end of the preamble
    if (width > 1200) {
      //rf69ook_startRssi();
      resetDecoder();
      return false;
    }

    bool isShort = width < 750;
    if (decoder.state == 0) {
      // expecting a short to start a bit
      if (isShort) {
        decoder.state = 1;
        return false;
      }
    }
    else if (decoder.state == 1) {
      decoder.state = 0;
      if (isShort)
        decoderAddBit(1);
      else
        decoderAddBit(0);

      // If we have all 3 bytes, we're done
      if (decoder.pos > 2)
        return true;
      return false;
    }
  }  // if proper width

  resetDecoder();
  return false;
}

void decodePowermon(uint16_t val16) {
  switch (decoder.data[0] & 3) {
  case OOK_PACKET_INSTANT:
    // val16 is the number of milliseconds between blinks
    // Each blink is one watt hour consumed
    g_RxWatts = 3600000UL / val16;
    break;

  case OOK_PACKET_TEMP:
    #if defined(TEMPERATURE_F)
    g_RxTemperature = (int8_t)(temp_lerp(decoder.data[1]));
    #else
    g_RxTemperature = (int8_t)(fudged_f_to_c(temp_lerp(decoder.data[1])));
    #endif /* TEMPERATURE_F */
    g_RxFlags = decoder.data[0];
    break;

  case OOK_PACKET_TOTAL:
    g_PrevRxWattHours = g_RxWattHours;
    g_RxWattHours = val16;
    // prevent rollover through the power of unsigned arithmetic
    g_TotalRxWattHours += (g_RxWattHours - g_PrevRxWattHours);
    break;
  }
}

void decodeRxPacket(void) {
  uint16_t val16 = *(uint16_t *)decoder.data;
  #ifndef TX_ID_LOCK
  if (crc8(decoder.data, 3) == 0) {
    g_TxIdNew = decoder.data[1] << 8 | decoder.data[0];
    g_PrintTimeDelta_ms = millis() - g_PrevPrintTime_ms;
    String buf;
    buf = String(F("New,")) + String(g_PrintTimeDelta_ms,DEC) + ',' 
          + String(getLastRssi(),DEC) + ',' 
          + String(val16,HEX);
    UDPSend(buf);    
    /*Serial.print(F("#,"));
    Serial.print(g_PrintTimeDelta_ms, DEC);
    printRSSI();
    Serial.print(F(",New ID,0x"));
    Serial.println(val16, HEX);
    */
    return;
  }
  #endif /* ifndef TX_ID_LOCK */

  val16 -= g_TxId;
  decoder.data[0] = val16 & 0xff;
  decoder.data[1] = val16 >> 8;
  if (crc8(decoder.data, 3) == 0) {
    decodePowermon(val16 & 0xfffc);
    g_RxDirty = true;
    g_RxLast = millis();
  } else {
    g_PrintTimeDelta_ms = millis() - g_PrevPrintTime_ms;
    String buf;
    buf = String(F("Err,")) + String(g_PrintTimeDelta_ms,DEC) + ',' + String(getLastRssi(),DEC);
    UDPSend(buf);    
    /*
    Serial.print(F("#,"));
    Serial.print(g_PrintTimeDelta_ms, DEC);
    Serial.println(F(",CRC ERR"));
    */
  }
}

void printResults(unsigned long g_PrintTimeDelta_ms, uint16_t g_RxWattHours, uint16_t g_TotalRxWattHours, uint16_t g_RxWatts, int8_t g_RxTemperature) {
    String temperatureS;
    if(g_RxTemperature == -100) 
      temperatureS = String('U');
    else
      temperatureS = String(g_RxTemperature,DEC);
      
    String buf;
    buf = String(F("Read,")) + String(g_PrintTimeDelta_ms, DEC) + ',' 
          + String(g_RxWattHours,DEC) + ',' 
          + String(g_TotalRxWattHours,DEC) + ',' 
          + String(g_RxWatts,DEC) + ',' 
          + temperatureS + ',' 
          + String(getLastRssi(),DEC);
    UDPSend(buf);
    /*  
    Serial.print(F(",")); 
    Serial.print(g_RxWattHours, DEC);  
    Serial.print(F(",")); 
    Serial.print(g_TotalRxWattHours, DEC);
    Serial.print(F(",")); 
    Serial.print(g_RxWatts, DEC);
    Serial.print(F(","));
    Serial.print(g_RxTemperature, DEC);
    */
    g_RxDirty = false;
    /*
    printRSSI();
        
    float humidity = 0;// dht.readHumidity();
    // Read temperature as Celsius
    float temperature = 0; //dht.readTemperature();
    Serial.print(F(",")); 
    Serial.print(humidity);
    Serial.print(F(",")); 
    Serial.println(temperature);  
    */
}

void ookRx(void) {
  uint16_t v;
  //ATOMIC_BLOCK(ATOMIC_FORCEON)
  //{
  yield();
  v = pulse_433;
  pulse_433 = 0;
  yield();
  //}
  //Serial.print(v);
  //Serial.print(", DS: ");
  //Serial.print(spiRead(RH_RF22_REG_02_DEVICE_STATUS), HEX);
  //Serial.print(", RSSI: "); 
  //Serial.println((int8_t)(-120 + ((rf22.spiRead(RH_RF22_REG_26_RSSI) / 2))), DEC); 
  if (v != 0) {
    if (decodeRxPulse(v) == 1) {
      #ifdef RADIOCC110
        //g_RxRssi = getLastRssi(); //rf69ook_Rssi();
      #endif
      #ifdef RADIORF22
        g_RxRssi = (int8_t)(-120 + ((rf22.spiRead(RH_RF22_REG_26_RSSI) / 2)));
      #endif
      decodeRxPacket();
      resetDecoder();
    }
  } else if (g_RxDirty && (millis() - g_RxLast) > 250U) { // If it has been more than 250ms since the last receive, dump the data
    /*
     track duration since last report
     
     If > ~31.8 s (318nn ms), we have missed a packet, 
     and the instantaneous Power reading 
     isn't continuous. 
     */
    g_PrevPrintTime_ms = g_PrintTime_ms;
    g_PrintTime_ms = millis();
    g_PrintTimeDelta_ms = g_PrintTime_ms - g_PrevPrintTime_ms;

    printResults(g_PrintTimeDelta_ms, g_RxWattHours, g_TotalRxWattHours, g_RxWatts, g_RxTemperature);

    g_RxDirty = false;
  } else if (g_RxLast != 0 && (millis() - g_RxLast) > 32000U) { 
    g_PrintTimeDelta_ms = millis() - g_PrevPrintTime_ms;
    String buf;
    buf = String(F("Mis,")) + String(g_PrintTimeDelta_ms,DEC) + ',' + String(getLastRssi(),DEC);
    UDPSend(buf);
    /*
    Serial.print(F("#,"));
    Serial.print(g_PrintTimeDelta_ms, DEC);
    Serial.print(F(",RSSI=")); 
    #ifdef RADIOCC110
      Serial.print(getLastRssi(),DEC);
    #endif
    #ifdef RADIORF22
      Serial.print((int8_t)(-120 + ((rf22.spiRead(RH_RF22_REG_26_RSSI) / 2))),DEC); 
    #endif
    printRSSI();
    Serial.print(F(",")); 
    Serial.println(F("Missed Packet"));  
    */
      
    g_RxLast = millis();
  }
}

void loop() { 
  if(1) 
    ookRx();
  
//  server.handleClient();
}

