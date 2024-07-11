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
  within the ACKtimeout period, the original packet is re-transmitted until a valid acknowledgement is
  received. This program should be used with the matching receiver program, 210_Reliable_Receiver_AutoACK.

  The program will attempt to transmit the packet and have it acknowledged by the receiver a number of times
  as defined by constant TXattempts. If there is no acknowledge withing this time it will be reported.

  It is possible to use the 'NetworkID' to direct the packet to specific receivers.

  Serial monitor baud rate should be set at 115200.

  *******************************************************************************************************/

#include <SPI.h>                                //the LoRa device is SPI based so load the SPI library                                         
#include <SX127XLT.h>                           //include the appropriate library  

#include <ForestWatch.h>

// Sensor-DHT11
#include <DHT.h>

#include <Preferences.h>

// LoRa params
#define LORA_DEVICE DEVICE_SX1278               //we need to define the device we are using
#define TXpower 2                               //LoRa transmit power in dBm
#define ACKtimeout 1500                         //Acknowledge timeout in mS                      
#define TXtimeout 1000                          //transmit timeout in mS. If 0 return from transmit function after send.  
#define TXattempts 5                            //number of times to attempt to TX and get an Ack before failing

#define DELAY_RETRY_TX 500

#define MSG_SIZE 17

#define CONF_NAMESPACE "config"
#define CONF_CHIP_ID "chip_id"

#define POWER_PIN 33

#if defined(ARDUINO_ESP32_DEV) // TTGO
// LoRa
#define NSS 4
#define NRESET 2
#define DIO0 15
// LED and sensors
#define LED1 5
#define DHT11PIN 12
#define MEMSPIN 13
#else // NodeMCU
// LoRa
#define NSS 4
#define NRESET 5
#define DIO0 16
// LED and sensors
#define LED1 2
#define DHT11PIN 17
#define MEMSPIN 13
#endif

SX127XLT LT;                                    //create a library class instance called LT
DHT dht(DHT11PIN, DHT11);

uint16_t PayloadCRC;
uint8_t TXPayloadL;                                            //this is the payload length sent
uint8_t TXPacketL;

const uint16_t NetworkID = 0x3210;              //NetworkID identifies this connection, needs to match value in receiver

uint64_t chip_id;

void loop() {
  //This is not going to be called
}

// loop() in non-deepsleep version
void maintask() {
  uint8_t buff[MSG_SIZE];
  float temperature, humidity;
  uint16_t smoke;

  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  // From https://www.arduino.cc/reference/en/language/functions/analog-io/analogread/
  // If the analog input pin is not connected to anything, the value returned by analogRead() will fluctuate 
  // based on a number of factors (e.g. the values of the other analog inputs, how close your hand is to the board, etc.).
  smoke = analogRead(MEMSPIN);

  Serial.printf("Chip ID: %lld Temp: %.2f Hum: %.2f Smoke: %d\n", chip_id, temperature, humidity, smoke);

  //keep transmitting the packet until an ACK is received
  uint8_t attempt = 1;

  do
  {
    pack_data2(buff, chip_id, temperature, humidity, smoke, attempt);
    dump_data(buff);

    Serial.print(F("Transmit Payload > "));
    TXPayloadL = sizeof(buff);
    LT.printASCIIArray(buff, TXPayloadL);     //print the payload buffer as ASCII
    Serial.println();
    Serial.flush();

    Serial.print(F("Send attempt "));
    Serial.println(attempt);

    digitalWrite(LED1, HIGH);                                            //LED on to indicate transmit
    TXPacketL = LT.transmitReliableAutoACK(buff, TXPayloadL, NetworkID, ACKtimeout, TXtimeout, TXpower, WAIT_TX);

    if (TXPacketL > 0)
    {
      //if transmitReliable() returns > 0 then transmit and ack was OK
      PayloadCRC = LT.getTXPayloadCRC(TXPacketL);                        //read the actual transmitted CRC from the LoRa device buffer
      packet_is_OK();
      Serial.println();
    }
    else
    {
      //if transmitReliableAutoACK() returns 0 there was an error, timeout etc
      packet_is_Error();
      Serial.println();
    }
    delay(DELAY_RETRY_TX);                                        //small delay between tranmission attampts
  }
  while ((TXPacketL == 0) && (++attempt <= TXattempts));

  if (TXPacketL > 0)
  {
    Serial.println(F("Packet acknowledged"));
  }

  if (attempt > TXattempts)
  {
    Serial.print(F("No acknowledge after "));
    Serial.print(TXattempts);
    Serial.print(F(" attempts"));
  }

  digitalWrite(LED1, LOW);
  Serial.println();
}


void packet_is_OK()
{
  Serial.print(F("LocalNetworkID,0x"));
  Serial.print(NetworkID, HEX);
  Serial.print(F(",TransmittedPayloadCRC,0x"));        //print CRC of transmitted packet
  Serial.print(PayloadCRC, HEX);
}


void packet_is_Error()
{
  Serial.print(F("No Packet acknowledge"));
  LT.printIrqStatus();                                 //prints the text of which IRQs set
  LT.printReliableStatus();                            //print the reliable status
}

void pack_data(uint8_t buff[], uint64_t chip_id, float temperature, float humidity, uint16_t smoke, uint8_t attempt) {
  uint8_t *bytes;
  uint8_t i, pos = 0;
  // Copy the bytes of each input variable into buff  
  bytes = (uint8_t *) &smoke;
  for (i = 0; i < sizeof(smoke); i++) {
    buff[pos++] = bytes[i];
  }
  bytes = (uint8_t *) &humidity;
  for (i = 0; i < sizeof(humidity); i++) {
    buff[pos++] = bytes[i];
  }
  bytes = (uint8_t *) &temperature;
  for (i = 0; i < sizeof(temperature); i++) {
    buff[pos++] = bytes[i];
  }
  bytes = (uint8_t *) &chip_id;
  // Only 6 of 8 bytes of chip_id will be used
  for (i = 0; i < 6; i++) {
    buff[pos++] = bytes[i];
  }
  bytes = (uint8_t *) &attempt;
  for (i = 0; i < sizeof(attempt); i++) {
    buff[pos++] = bytes[i];
  }
}

void pack_data2(uint8_t buff[], uint64_t chip_id, float temperature, float humidity, uint16_t smoke, uint8_t attempt) {
  // Copy the bytes of each input variable into buff 
  memcpy(buff, &smoke, sizeof(smoke));
  buff += sizeof(smoke);
  memcpy(buff, &humidity, sizeof(humidity));
  buff += sizeof(humidity);
  memcpy(buff, &temperature, sizeof(temperature));
  buff += sizeof(temperature);
  // Only 6 of 8 bytes of chip_id will be used
  memcpy(buff, &chip_id, 6);
  buff += 6;
  memcpy(buff, &attempt, sizeof(attempt));
  buff += sizeof(attempt);
}

void dump_data(uint8_t buff[]) {
  for (int i = 0; i < MSG_SIZE; i++) {
    Serial.printf("%d ", buff[i]);
  }
  Serial.println();
}

void setup()
{
  Preferences preferences;

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("209_Reliable_Transmitter_AutoACK Starting"));

  pinMode(LED1, OUTPUT);
  // Alternatively we can power LoRa and/or sensors by POWER_PIN instead of 3V3
  // and turn it off when falling to sleep
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

  esp_sleep_enable_timer_wakeup(FW_TIME_TO_SLEEP * FW_uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(FW_TIME_TO_SLEEP) + " Seconds");

  // Set analog resolution to 10 bits, so that smoke sensors returns values between 0-1023
  analogReadResolution(10);

  SPI.begin();

  preferences.begin(CONF_NAMESPACE, false);
  chip_id = preferences.getULong64(CONF_CHIP_ID);

  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE)) {
    Serial.println(F("LoRa Device found"));
    delay(1000);
  }
  else {
    Serial.println(F("No LoRa device responding"));
    while (1);
  }

  LT.setupLoRa(FW_LORA_FREQ, 0, FW_LORA_SF, FW_LORA_BW, FW_LORA_CR, LDRO_AUTO); //configure frequency and LoRa settings

  /*
  Serial.println();
  LT.printModemSettings();                               //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();
  LT.printRegisters(0x00, 0x4F);                         //print contents of device registers, normally 0x00 to 0x4F
  */
  Serial.println();
  Serial.println(F("Transmitter ready"));
  Serial.println();

  dht.begin();

  maintask(); // do the main task

  LT.setSleep(CONFIGURATION_RETENTION);

  Serial.println("Going to sleep now");
  Serial.flush();
  digitalWrite(POWER_PIN, LOW);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}
