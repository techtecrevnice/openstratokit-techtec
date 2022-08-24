/*
  openSTRATOkit Basic Tracker

  @DevOK1RAJ Team, code by OK1CDJ 3/2021, later edits Q1/2022

  low ("space") frequency:     434.69 MHz
  frequency shift:             610 Hz
  baud rate:                   300 baud
  encoding:                    ASCII (7-bit)
  stop bits:                   2

  NOTE: RTTY frequency shift will be rounded
        to the nearest multiple of frequency step size.
        The exact value depends on the module:
        RF69 - 61 Hz steps

  MegaCoreX documentation: https://github.com/MCUdude/MegaCoreX

  Compile and upload instructions:
  - if you're using Windows, you can skip all but the last two steps
  and open this file in a portable version of Arduino IDE with everything
  preinstalled: https://files.dotknisevesmiru.cz/openSTRATOkit_Arduino-IDE-1.8.19-windows.zip
  - In your Arduino IDE, go to File > Preferences and enter
  https://mcudude.github.io/MegaCoreX/package_MCUdude_MegaCoreX_index.json
  into the “Additional Board Manager URLs” field
  - Go to Tools > Board > Boards Manager…, search for MegaCoreX and install
    MegaCoreX by MCUdude
  - In the Tools > Board menu select MegaCoreX > ATmega4809
  - In the Tools menu, make sure you select:
    Clock: "Internal 16MHz"
    BOD: "BOD 2.6V"
    Pinout: "48 pin standard"
    Reset pin: "Reset"
    Bootloader: "Optiboot (UART3 alternative pins)"
    Programmer: "JTAG2UPDI"
  - Download the following libraries from the Arduino Library Manager
  (Project > Add library > Manage libraries...):
    Adafruit AHT10 by Adafruit (select "Install All" when prompted)
    RadioLib by Jan Gromes -> make sure to select version 4.3.0
  - Download and import TinyGPSPlus library from here:9
  https://github.com/mikalhart/TinyGPSPlus/archive/refs/heads/master.zip
  (Project > Add library > Add .ZIP Library)
  - Connect your openSTRATOkit and select it in Tools > Port
  (make sure you use a short, quality cable, like the one provided with the kit)
  - Before uploading, make sure to change the default call to your own, in the variable below
  ---
    NOTE: in case you are having problems connecting to the board or even getting it
    recognized in Windows, please install this driver:
    https://cdn.sparkfun.com/assets/learn_tutorials/8/4/4/CH341SER.EXE
*/

// libraries
#include <Adafruit_AHT10.h>
#include <Adafruit_BMP280.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <util/crc16.h>
#include <SPI.h>
#include <SD.h>

// Radio Settings
#define FREQ 434.690
#define SHIFT 610
#define BAUD 300
#define ENC ASCII
#define STOPB 2

//get from https://www.chmi.cz/aktualni-situace/aktualni-stav-pocasi/ceska-republika/stanice/profesionalni-stanice/tabulky/tlak-vzduchu
#define LOCAL_SEA_PRESSURE 1011.6

String call = "TTS9"; // CHANGE THIS!
long pkt_num = 1; // packet number
float batt_voltage;
sensors_event_t humidity, temperature;

float pressure;    //To store the barometric pressure (hPa)
float inner_temperature;  //To store the temperature (°C)
float altimeter;    //To store the altimeter (m)

File myFile;

RF69 radio = new Module(33, 9, RADIOLIB_NC);

// create RTTY client instance using the FSK module
RTTYClient rtty(&radio);

TinyGPSPlus gps;

Adafruit_AHT10 aht;
Adafruit_BMP280 bmp;

String str;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void setup() {

  // init serial comm
  Serial3.pins(12, 13);
  Serial3.begin(115200);
  Serial1.pins(18, 19);
  Serial1.begin(9600);
  Serial3.println("openSTRATOtracker");
  Serial3.println("zirafa a zizala a slon a hroch a vcela a vosa a petr cermak mod");
  Serial3.println();
  Serial3.println("lalalalalalalalalalalalalalalalalalalalalalalalalalalalalalalalalalalala_blablablablablablablablablablablablablablabla");
  // init bmp280
  if (!bmp.begin()) {
    Serial3.println("[BMP280] no BMP280 detected...");
  } else Serial3.println("[BMP280] found...");

  Serial.pins(0, 1);
  Serial.begin(9600);
  str = gsmAtCommand("AT");
  str.trim();
  Serial3.println("SIM800 resp >>" + str + "<<<");
  if (str.indexOf("OK") > -1)
  {
    Serial3.println("[SIM800L] found...");
    gsmAtCommand("AT+CMGF=1");
    gsmSendSms("Sonda GSM Successfully initialized");
  }
  else
  {
    Serial3.println("[SIM800L] not found...");
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // init and check for AHT10
  if (!aht.begin()) {
    Serial3.println("[AHT10] no AHT10 detected...");
  } else Serial3.println("[AHT10] found...");

  // init radio
  Serial3.print(F("[RF69] Initializing ... "));
  SPI.pins(30, 31, 32, 33);
  int state = radio.begin();

  // check for errors
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    resetFunc();
    while (true);
  }

  // radio output power
  Serial3.print(F("[RF69] Setting high power module ... "));
  state = radio.setOutputPower(20, true);
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    resetFunc();
    while (true);
  }

  // set-up rtty comm
  Serial3.print(F("[RTTY] Initializing ... "));
  state = rtty.begin(FREQ, SHIFT, BAUD, ENC, STOPB);
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    resetFunc();
    Serial3.println(state);
    while (true);
  }

  // set-up GPS
  Serial3.println(F("[GPS] Set flight mode ... "));
  setGPS_DynamicModel6();
  delay(500);
  setGps_MaxPerformanceMode();
  delay(500);

  SPI.pins(30, 31, 32, 33);

  digitalWrite(33, 1);

  // init SD card
  //Serial3.print("[SD] Initializing SD card...");
  //if (!SD.begin(10)) {
  //  Serial3.println("initialization failed!");
  //} else Serial3.println(F("success!"));
  //digitalWrite(10, HIGH);
  //SPI.transfer(0xAA);
}

void loop() {



  // get data from AHT10
  temperature.temperature = 0;
  humidity.relative_humidity = 0;
  aht.getEvent(&humidity, &temperature);

  // get data from BMP280
  pressure = bmp.readPressure() / 100;
  inner_temperature = bmp.readTemperature();
  altimeter = bmp.readAltitude(LOCAL_SEA_PRESSURE);
  //test
  String ds;
  ds = "C$$$" + call + ",";    // Call
  ds += String(inner_temperature) + ",";      // temperature internal
  ds += String(pressure) + ",";      // pressure
  ds += String(altimeter) + ",";      // altitude from pressure
  Serial3.println(ds);
  delay(1000);


  // read GPS and send RTTY messages
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
    sendData();
  }

  // try to fix the GPS in case of a malfunction
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial3.println(F("No GPS detected."));
    resetGPS();
    resetFunc();
    while (true);
  }
}

void gsmSendSms(String text) {
  gsmAtCommand("AT+CMGS=\"+420703998961\""); // Techtec prijimac
  Serial.print(text);
  Serial.write(26);
}

String gsmAtCommand(String command) {
  Serial.println(command);
  return gsmReadResponse();
}

String gsmReadResponse()
{
  delay(500);
  if (Serial.available())
  {
    return Serial.readString();

  }
  return "";
}

// compile sensor information and transmit it
void sendData() {

  // calculate battery voltage
  batt_voltage = (analogRead(A3) * 3.3 / 1024) *
                 ((15.0 + 33.0) / 33.0); // divider 15k and 33k

  if (gps.location.isUpdated() && gps.altitude.isUpdated()) {
    String datastring;

    datastring = "$$$$" + call + ",";    // Call
    datastring += String(pkt_num) + ","; // Packet number

    // GPS time
    if (gps.time.hour() < 10)
      datastring += "0" + String(gps.time.hour()) + ":";
    else
      datastring += String(gps.time.hour()) + ":";

    if (gps.time.minute() < 10)
      datastring += "0" + String(gps.time.minute()) + ":";
    else
      datastring += String(gps.time.minute()) + ":";

    if (gps.time.second() < 10)
      datastring += "0" + String(gps.time.second()) + ",";
    else
      datastring += String(gps.time.second()) + ",";

    datastring += String(gps.location.lat(), 6) + ",";    // lat
    datastring += String(gps.location.lng(), 6) + ",";    // long
    datastring += String(gps.altitude.meters(), 0) + ","; // altitude
    //datastring += String(gps.speed.mps()) + ",";          // speed
    //datastring += String(gps.course.deg()) + ",";         // course
    datastring += String(batt_voltage, 2) + ",";          // voltage
    //datastring += String(temp.temperature, 1) + ",";      // temperature internal
    datastring += String(inner_temperature, 1) + ",";      // temperature internal
    datastring += String(pressure, 1) + ",";      // pressure
    datastring += String(altimeter) + ",";      // altitude from pressure
    datastring += String(temperature.temperature, 1) + ",";  // temperature external
    datastring += String(humidity.relative_humidity, 0) + ",";     // humidity external
    datastring += String(gps.satellites.value());         // sats

    // checksum
    unsigned int CHECKSUM = gps_CRC16_checksum(datastring.c_str());
    char checksum_str[6];
    sprintf(checksum_str, "*%04X", CHECKSUM);
    datastring += String(checksum_str);

    // transmit the data
    Serial3.println(F("[RTTY] Sending RTTY data ... "));

    // send out idle condition for 500 ms
    rtty.idle();
    delay(1000);

    Serial3.println(datastring);
    rtty.println(datastring);

    Serial3.println(F("[RTTY] Done!"));
    writeData(datastring); // write a copy to the SD card
    pkt_num++; //advance packet number
  }
}

// write data to the SD card
void writeData(String Str) {
  myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile) {
    Serial3.print("[SD] Writing to data.txt...");
    myFile.println(Str);
    myFile.close();
    Serial3.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial3.println("error opening data.txt");
  }
}

// calculate a CRC16 checksum
uint16_t gps_CRC16_checksum(char *string) {
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first four $s
  for (i = 4; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update(crc, c);
  }
  return crc;
}


void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial1.flush();
  Serial1.write(0xFF); //
  delay(100);
  for (int i = 0; i < len; i++) {
    Serial1.write(MSG[i]);
  }
}

void resetGPS() {
  /*
    Forced (Watchdog)
    Coldstart
  */
  uint8_t set_reset[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5};
  sendUBX(set_reset, sizeof(set_reset) / sizeof(uint8_t));
}

void setGPS_DynamicModel6()
{
  /*
    CFG-NAV5

    Header: 0xB5, 0x62,
    ID: 0x06, 0x24,
    Length 0x24, 0x00,
    mask 0xFF, 0xFF,
    dynModel:  0x06, (Airborne <1g)
    fixMode: 0x03,
    fixedAlt: 0x00, 0x00, 0x00, 0x00,
    fixedAltVar: 0x10, 0x27, 0x00, 0x00,
    minElev 0x05,
    drLimit 0x00,
    pDop 0xFA, 0x00,
    tDop 0xFA, 0x00,
    pAcc 0x64, 0x00,
    tAcc 0x2C, 0x01,
    staticHoldThresh 0x00,
    dgpsTimeOut 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    CK_A 0x16,
    CK_B 0xDC

  */
  int gps_set_sucess = 0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  };
  while (!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setdm6);
  }
  //morse("OK");
}

void setGps_MaxPerformanceMode() {
  /*
    UBX-CFG-RMX - 0 Continuous Mode (Max Performance Mode)
  */
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = {
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91
  }; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax) / sizeof(uint8_t));
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B

  // Calculate the checksums
  for (uint8_t ubxi = 2; ubxi < 8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      return false;
    }

    // Make sure data is available to read
    if (Serial1.available()) {
      b = Serial1.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
      }
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }

    }
  }
}
