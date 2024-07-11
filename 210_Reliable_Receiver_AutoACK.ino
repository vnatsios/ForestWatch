/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 13/09/21

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a basic demonstration of the transmission and acknowledgement of a 'Reliable'
  packet.

  A reliable packet has 4 bytes automatically appended to the end of the buffer\array that is the data
  payload. The first two bytes appended are a 16bit 'NetworkID'. The receiver needs to have the same
  NetworkID as configured for the transmitter since the receiver program uses the NetworkID to check that
  the received packet is from a known source.  The third and fourth bytes appended are a 16 bit CRC of
  the payload. The receiver will carry out its own CRC check on the received payload and can then verify
  this against the CRC appended in the packet. The receiver is thus able to check if the payload is valid.

  For a packet to be accepted by the receiver, the networkID and payload CRC appended to the packet by the
  transmitter need to match those from the receiver which gives a high level of assurance that the packet
  is valid.

  If the received packet is valid then the networkID and payload CRC are returned in a 4 byte packet as an
  acknowledgement that the transmitter listens for. If the transmitter does not receive the acknowledgement
  of the networkID and payloadCRC within the ACKtimeout period, the original packet is re-transmitted until
  a valid acknowledgement is received. This program should be used with the matching transmitter program,
  209_Reliable_Transmitter_AutoACK.

  Serial monitor baud rate should be set at 115200.
*******************************************************************************************************/

#define FW_APP_USE_SCREEN
#define FW_APP_USE_MQTT

#include <SPI.h>                                //the LoRa device is SPI based so load the SPI library
#include <SX127XLT.h>                           //include the appropriate library   

#include <ForestWatch.h>

// I2C-Monitor-SSD1306
#ifdef FW_APP_USE_SCREEN
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

// MQTT Client
#ifdef FW_APP_USE_MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#endif

SX127XLT LT;                                    //create a library class instance called LT

#define NSS 4                                  //select pin on LoRa device
#define NRESET 5                                //reset pin on LoRa device
#define DIO0 16                                  //DIO0 pin on LoRa device, used for RX and TX done 
#define LED1 2                                  //LED used to indicate receive
#define LORA_DEVICE DEVICE_SX1278               //we need to define the device we are using

#define ACKdelay 100                            //delay in mS before sending acknowledge                    
#define RXtimeout 300000                        //receive timeout in millisec (5 min) - matches transmitter sleep time
#define TXpower 2                               //dBm power to use for ACK   

// I2C-Monitor-SSD1306
#ifdef FW_APP_USE_SCREEN
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

// MQTT Client
#ifdef FW_APP_USE_MQTT
WiFiClient espClient; // Initialize the WiFi client
PubSubClient mqttclient(FW_MQTT_SERVER, FW_MQTT_PORT, espClient); // Initialize the MQTT client
#endif

// Other
#define DELAY 1000
#define MSG_SIZE 17
#define MAX_MSG_SIZE 128

const uint8_t RXBUFFER_SIZE = 251;              //RX buffer size, set to max payload length of 251, or maximum expected length
uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into

uint8_t RXPacketL;                              //stores length of packet received
uint8_t RXPayloadL;                             //stores length of payload received
uint8_t PacketOK;                               //set to > 0 if packetOK
int16_t PacketRSSI;                             //stores RSSI of received packet
int8_t PacketSNR;                               //stores SNR of received packet
uint16_t LocalPayloadCRC;                       //locally calculated CRC of payload
uint16_t RXPayloadCRC;                          //CRC of payload received in packet
uint16_t TransmitterNetworkID;                  //the NetworkID from the transmitted and received packet

const uint16_t NetworkID = 0x3210;              //NetworkID identifies this connection, needs to match value in transmitter


void loop()
{
  float temperature, humidity;
  uint16_t smoke;
  uint8_t attempt;
  uint64_t chip_id;
  struct tm timeinfo;
 
  PacketOK = LT.receiveReliableAutoACK(RXBUFFER, RXBUFFER_SIZE, NetworkID, ACKdelay, TXpower, RXtimeout, WAIT_RX); //wait for a packet to arrive with RXtimeout timeout

  RXPacketL = LT.readRXPacketL();               //get the received packet length
  RXPayloadL = RXPacketL - 4;                   //payload length is always 4 bytes less than packet length
  PacketRSSI = LT.readPacketRSSI();             //read the received packets RSSI value
  PacketSNR = LT.readPacketSNR();               //read the received packets SNR value

  if (PacketOK > 0)
  {
    char json[256];
    char datetime[15];
    if (getLocalTime(&timeinfo)) {
      strftime(datetime, 15, "%d/%m %H:%M:%S", &timeinfo);
    }

    //if the LT.receiveReliable() returns a value > 0 for PacketOK then packet was received OK
    packet_is_OK();
    dump_data(RXBUFFER);
    unpack_data(RXBUFFER, &chip_id, &temperature, &humidity, &smoke, &attempt);
    sprintf(json, "{\"cid\": %lld, \"t\": %.2f, \"h\": %.2f, \"g\": %d, \"a\": %d, \"rssi\": %d, \"snr\": %d}", chip_id, temperature, humidity, smoke, attempt, PacketRSSI, PacketSNR);
    Serial.printf("Chip ID: %lld Temp: %.2f Hum: %.2f Smoke: %d Att: %d\n", chip_id, temperature, humidity, smoke, attempt);
    // Serial.println(json);
    Serial.printf("Packet Length: %u Payload Length: %u\n", RXPacketL, RXPayloadL);
    Serial.printf("Date/Time: %s\n", datetime);
    #ifdef FW_APP_USE_SCREEN
    // 5th line of message doesn't appear on 128x32 SSD1306 display
    printmsg(0, 0, true, "SID: %lld\nT: %d H: %d S: %u\nRSSI: %d SNR: %d\nD/T: %s\nPacket Size: %u", chip_id, (int) temperature, (int) humidity, smoke, PacketRSSI, PacketSNR, datetime, RXPacketL);
    #endif
    #ifdef FW_APP_USE_MQTT
    if (!WiFi.isConnected()) {
      wifi_connect();
    }
    if (!mqttclient.connected()) {
      mqtt_connect();
    }
    if (!mqttclient.publish(FW_MQTT_TOPIC, json)) {
      Serial.println("Failed to publish to MQTT");
    }
    #endif
  }
  else
  {
    //if the LT.receiveReliable() function detects an error PacketOK is 0
    packet_is_Error();
  }

  Serial.println();
}


void packet_is_OK()
{
  Serial.print(F("Payload received OK > "));
  LT.printASCIIPacket(RXBUFFER, RXPayloadL);
  Serial.println();
  printPacketDetails();
  Serial.println();
}


void packet_is_Error()
{
  uint16_t IRQStatus;

  IRQStatus = LT.readIrqStatus();                  //read the LoRa device IRQ status register
  Serial.print(F("Error "));

  if (IRQStatus & IRQ_RX_TIMEOUT)                  //check for an RX timeout
  {
    Serial.print(F(" RXTimeout "));
  }
  else
  {
    printPacketDetails();
  }
}


void printPacketDetails()
{
  LocalPayloadCRC = LT.CRCCCITT(RXBUFFER, RXPayloadL, 0xFFFF);  //calculate payload crc from the received RXBUFFER
  TransmitterNetworkID = LT.getRXNetworkID(RXPacketL);
  RXPayloadCRC = LT.getRXPayloadCRC(RXPacketL);

  Serial.print(F("LocalNetworkID,0x"));
  Serial.print(NetworkID, HEX);
  Serial.print(F(",TransmitterNetworkID,0x"));
  Serial.print(TransmitterNetworkID, HEX);
  Serial.print(F(",LocalPayloadCRC,0x"));
  Serial.print(LocalPayloadCRC, HEX);
  Serial.print(F(",RXPayloadCRC,0x"));
  Serial.print(RXPayloadCRC, HEX);
  LT.printReliableStatus();
}

#ifdef FW_APP_USE_SCREEN
void printmsg(int16_t curx, int16_t cury, boolean clear, char *formatstr, ...) {
  char output[MAX_MSG_SIZE];
  va_list args;
  va_start(args, formatstr);
  vsprintf(output, formatstr, args);
  va_end(args);  

  if (clear) {
    display.clearDisplay();
  }
  // display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(curx, cury);
  display.print(output);
  display.display();
}
#endif

void unpack_data(uint8_t buff[], uint64_t *chip_id, float *temperature, float *humidity, uint16_t *smoke, uint8_t *attempt) {
  // Same as unpack_data2 but implementing memcpy by itself!
  uint8_t *bytes;
  uint8_t i, pos = 0;
  // Copy the bytes of buffer into the variables
  bytes = (uint8_t *) smoke;
  for (i = 0; i < sizeof(*smoke); i++) {
    bytes[i] = buff[pos++];
  }
  bytes = (uint8_t *) humidity;
  for (i = 0; i < sizeof(*humidity); i++) {
    bytes[i] = buff[pos++];
  }
  bytes = (uint8_t *) temperature;
  for (i = 0; i < sizeof(*temperature); i++) {
    bytes[i] = buff[pos++];
  }
  bytes = (uint8_t *) chip_id;
  // Only 6 of 8 bytes of chip_id will be used but we have to pad most significant bytes with zero's
  for (i = 0; i < 6; i++) {
    bytes[i] = buff[pos++];
  }
  bytes[6] = bytes[7] = 0;
  bytes = (uint8_t *) attempt;
  for (i = 0; i < sizeof(*attempt); i++) {
    bytes[i] = buff[pos++];
  }
}

void unpack_data2(uint8_t buff[], uint64_t *chip_id, float *temperature, float *humidity, uint16_t *smoke, uint8_t *attempt) {
  // Copy the bytes of buffer into the variables
  memcpy(smoke, buff, sizeof(*smoke));
  buff += sizeof(*smoke);
  memcpy(humidity, buff, sizeof(*humidity));
  buff += sizeof(*humidity);
  memcpy(temperature, buff, sizeof(*temperature));
  buff += sizeof(*temperature);
  // Only 6 of 8 bytes of chip_id will be used
  memcpy(chip_id, buff, sizeof(*chip_id));
  buff += sizeof(*chip_id);
  memcpy(attempt, buff, sizeof(*attempt));
  buff += sizeof(*attempt);
}

void dump_data(uint8_t buff[]) {
  // Change to 'i < MSG_SIZE + ...' to dump more bytes for padding problem
  for (int i = 0; i < MSG_SIZE; i++) {
    Serial.printf("%d ", buff[i]);
  }
  Serial.println();
}

#ifdef FW_APP_USE_MQTT
boolean wifi_connect() {
  Serial.println("Trying primary WiFi...");
  WiFi.begin(FW_WIFI_SSID, FW_WIFI_PASS);
  for (int cnt = 0; cnt < 30; cnt++) {
    if (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Trying fallback WiFi...");
    WiFi.begin(FW_WIFI_FLBK_SSID, FW_WIFI_FLBK_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }    
  // Print local IP address
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  configTime(FW_GMT_OFFSET, FW_DAYLIGHT_OFFSET, FW_NTP_SERVER);
  return (WiFi.isConnected());
}

boolean mqtt_connect() {
  int i = 0;
  while (!mqttclient.connect("arduinoClient", FW_MQTT_USERNAME, FW_MQTT_PASSWORD) && (i < 5)) {
    Serial.print("MQTT connection failed, rc=");
    Serial.print(mqttclient.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    delay(5000);
    i++;
  }
  return (mqttclient.connected());
}
#endif


void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("210_Reliable_Receiver_AutoACK Starting"));

  SPI.begin();

  #ifdef FW_APP_USE_SCREEN
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  #endif

  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    #ifdef FW_APP_USE_SCREEN
    printmsg(0, 0, true, "LoRa Device found");
    #endif
    delay(1000);
  }
  else
  {
    Serial.println(F("No LoRa device responding"));
    while (1);
  }

  LT.setupLoRa(FW_LORA_FREQ, 0, FW_LORA_SF, FW_LORA_BW, FW_LORA_CR, LDRO_AUTO);   //configure frequency and LoRa settings

  Serial.println();
  LT.printModemSettings();                               //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();
  LT.printRegisters(0x00, 0x4F);                         //print contents of device registers, normally 0x00 to 0x4F
  Serial.println();
  Serial.println(F("Receiver ready"));
  #ifdef FW_APP_USE_SCREEN
  printmsg(0, 10, false, "Receiver ready");
  #endif
  Serial.println();

  #ifdef FW_APP_USE_MQTT
  if (wifi_connect()) {
    #ifdef FW_APP_USE_SCREEN
    printmsg(0, 0, true, "WiFi connected");
    #endif
  }
  if (mqtt_connect()) {
    Serial.println("Connected to MQTT server");
    #ifdef FW_APP_USE_SCREEN
    printmsg(0, 10, false, "MQTT connected");
    #endif
  }
  #endif
}
