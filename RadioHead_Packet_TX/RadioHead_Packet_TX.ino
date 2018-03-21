// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include "Seeed_BME280.h"
#include <Wire.h>
#include <stdlib.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Instance of Weather Monitor
BME280 bme280;

int16_t packetnum = 0;  // packet counter, we increment per xmission
double temp = 0.0;
double pressure = 0.0; //pressure in Pa
double p = 0.0;       //pressure in hPa
double altitude = 0.0;
double humidity = 0.0;

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }

  if(!bme280.init()){
    Serial.println("Device error!");
  }
  
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}



void loop() {
  delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!
  readData();
  //serialPrintValues();

  char radiopacket[30];     //"Hello World #";
  char tempstr[10];
  dtostrf (temp, 4, 2, tempstr);
  Serial.print("The tempstr is: ");
  Serial.println(tempstr);
  char pressurestr[10];
  dtostrf (p, 5, 2, pressurestr);
  Serial.println(pressurestr);
  char humiditystr[10];
  dtostrf (temp, 4, 2, humiditystr);
  Serial.println(humiditystr);
  //char* dtostre (pressure, 10, 
  
  int i=0;
  int z=0;    
  do{
    radiopacket[i] = tempstr[z];
    ++i;
    ++z;
    }while(tempstr[z] != '\0');
  z = 0;
  do{
    radiopacket[i] = tempstr[z];
    ++i;
    ++z;
    }while(tempstr[z] != '\0');
  

  //itoa(packetnum++, tempstr+4, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  
  // Send a message!
  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf69.waitPacketSent();

  // Now wait for a reply
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf69.waitAvailableTimeout(500))  { 
    // Should be a reply message for us now   
    if (rf69.recv(buf, &len)) {
      Serial.print("Got a reply: ");
      Serial.println((char*)buf);
      Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is another RFM69 listening?");
  }
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
void readData()
{
  //Serial.println("Reading Temp");
  temp = ((bme280.getTemperature() * 9.0) / 5.0 + 32);
  //Serial.println("Reading Pressure");
  pressure = bme280.getPressure(); // pressure in Pa
  p = pressure/100.0 ; // pressure in hPa
  altitude = bme280.calcAltitude(pressure)/0.3048;
  //Serial.println("Reading Humidity");
  humidity = bme280.getHumidity();
}


void serialPrintValues()
{
 
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.println("F");//The unit for  Celsius because original arduino don't support speical symbols

  Serial.print("Pressure: ");
  Serial.print(p);
  Serial.println("hPa");

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println("ft");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
}
