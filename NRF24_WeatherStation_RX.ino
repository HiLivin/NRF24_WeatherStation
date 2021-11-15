/**
   TODO:
   NRF noAck test
   backlight control
**/

/**
   WIRING:

   BUZZER           A1
   ROTARY ENCODER   A0 ENC_BTN | A2 ENC_B | A3 ENC_A
   NRF24L01+        2 IRQ | 9 CE | 10 CSN | 11 MOSI | 12 MISO | 13 SCK
   DS3231           I2C
   BME280           I2C
   LCD2004          I2C_EXPANDER ( LCD_RS | NC | LCD_EN | LCD_BKL | LCD4 | LCD5 | LCD6 | LCD7 )
   SD CARD          8 CS | 11 MOSI | 12 MISO | 13 SCK

**/

#include <time.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BME280I2C.h>
#include <DS3231.h>
#include <SPI.h>
#include <Fat16.h>
#include <Fat16util.h>
#include <printf.h>
#include "RF24.h"
#include <PinChangeInterruptBoards.h>
#include <YetAnotherPcInt.h>

//#define   SERIAL_DEBUG
#define   TIME_MENU
#define   SD_CARD

#define   VERSION         "2.2"
#define   BEEP            A1
#define   ENCBTN          A0
#define   ENCB            A2
#define   ENCA            A3

#ifdef SD_CARD
#define   SD_CS           7
#endif

#define   RF24_PA_LEVEL   RF24_PA_MIN
#define   RF24_CHANNEL    17
#define   RF24_DATARATE   RF24_250KBPS
#define   RF24_CRCLENGTH  RF24_CRC_16
#define   RF24_CE_PIN     9
#define   RF24_CS_PIN     10
#define   RADIO_INT_PIN   2
#define   RADIO_NODES     4
#define   PA_LEVELS       4

#define   LCD_COLS        20
#define   LCD_ROWS        4
#define   BLINK_OFF       382
#define   ALARM_INC       0.1
#define   BAT_V_SCALE     6
#define   LATE_TX_TIME    900   //15 minutes

#define   C_DEG           0
#define   C_HOME          1
#define   C_DROP          2
#define   C_ALARM         3
#define   C_SDCARD        4
#define   C_LATE          5

#ifdef TIME_MENU
#define   LATITUDE        12.3456
#define   LONGITUDE       12.3456
#define   TIMEZONE        0
#endif

//degree sign
byte char_deg[8] = {
  0b00110,
  0b01001,
  0b01001,
  0b00110,
  0b00000,
  0b00000,
  0b00000,
};

byte char_home[8] = {
  0b00000,
  0b00100,
  0b01110,
  0b11111,
  0b11001,
  0b11001,
  0b11111,
  0b00000
};

byte char_drop[8] = {
  0b00100,
  0b00100,
  0b01110,
  0b01110,
  0b11111,
  0b10111,
  0b11111,
  0b01110,
};

byte char_alarm[8] = {
  0b00100,
  0b01110,
  0b01110,
  0b01110,
  0b11111,
  0b00000,
  0b00100,
  0b00000
};

byte char_sdcard[8] = {
  0b00111,
  0b01111,
  0b11111,
  0b11111,
  0b10001,
  0b10001,
  0b11111,
  0b00000
};

byte char_late[8] = {
  0b11111,
  0b10001,
  0b01010,
  0b00100,
  0b01110,
  0b11111,
  0b11111,
  0b00000
};

const byte base_address[6] = "baseX";
const byte node_address[] = {0x00, 0x01, 0x02, 0x03}; //OneWire addresses of temperature sensors used with transmitters

//Experimentally acquired constants for linear compensation of DS18B20 inaccuracy
// Treal = BAT_V_SCALE * (Tread + tempCompA * Tread + tempCompB)
//Read more: https://thecavepearlproject.org/2016/03/05/ds18b20-calibration-we-finally-nailed-it/
const float tempCompA[] = {.002, .004, .007, .011};
const float tempCompB[] = {.379, .334, .021, -0.212};

bool alarmOn = true;
float alarmThreshold = 0.00;
const uint16_t battThreshold = 3400;

uint32_t now_t;

struct frame {
  byte address;
  float temperature;
  uint16_t voltage;
  uint8_t power;
} packet;
const uint8_t frame_size = sizeof(frame);
uint32_t lastPacketTime[] = {0, 0, 0, 0};

struct homeStruct {
  float temperature;
  float pressure;
  float humidity;
};

struct sensorData {
  homeStruct inside;
  frame outside[RADIO_NODES];
} sensorData;

volatile struct encoder {
  int8_t position;
  bool button;
} encoder = {0, 0};

typedef void (*f)(void);
#ifdef TIME_MENU
f menu_func[] = {&mainMenu, &batteryMenu, &timeMenu};
char time_s1[6], time_s2[6];
int8_t moonPhase;
#else
f menu_func[] = {&mainMenu, &batteryMenu};
#endif
int8_t currentMenu = 0;
const uint8_t nMenus = sizeof(menu_func) / sizeof(f);
const uint8_t settingsMenuEntries = 4;
const uint16_t menuUpdateInterval = 500;
uint32_t lastMenuUpdate = 0;

RTClib RTC;
DateTime now;
const uint16_t clockSyncInterval = 1000;

RF24 radio(RF24_CE_PIN, RF24_CS_PIN);
const char* PA_Level[] = {"MIN", "LOW", "HIGH", "MAX"};

#ifdef SD_CARD
bool sd_card_in = false;
uint8_t dayOfLastLog = 0;
char fileName[12];
SdCard card;
Fat16 file;
#endif

LiquidCrystal_I2C lcd(0x38, LCD_COLS, LCD_ROWS);

BME280I2C bme;
BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_hPa);
const uint16_t bmeUpdateInterval = 60000;     //1 minute update interval
uint32_t bmeLastUpdate = 0;

/******************************/
/******************************/
//           Setup            //
/******************************/
/******************************/

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  Serial.print(F("Initializing..."));
#endif
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0); lcd.print(F(" Weather    _____   "));
  lcd.setCursor(0, 1); lcd.print(F(" station __(     )_ "));
  lcd.setCursor(0, 2); lcd.print(F("        (__________)"));
  lcd.setCursor(0, 3); lcd.print(F("  v      /**//*/*/  "));
  lcd.setCursor(3, 3); lcd.print(F(VERSION));

  //initialize custom chars
  lcd.createChar(C_DEG, char_deg);
  lcd.createChar(C_HOME, char_home);
  lcd.createChar(C_DROP, char_drop);
  lcd.createChar(C_ALARM, char_alarm);
  lcd.createChar(C_SDCARD, char_sdcard);
  lcd.createChar(C_LATE, char_late);

  pinMode(BEEP, OUTPUT);
  pinMode(ENCBTN, INPUT_PULLUP);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);

  //init BME280 sensor
  bme.begin();
  delay(2500);

  //initialize outside sensors temps
  sensorData.outside[0].temperature = 77.7;
  sensorData.outside[1].temperature = 77.7;
  sensorData.outside[2].temperature = 77.7;
  sensorData.outside[3].temperature = 77.7;
  
  //init and configure radio
  radio.begin();
  radio.setPALevel(RF24_PA_LEVEL);
  radio.setChannel(RF24_CHANNEL);
  radio.setDataRate(RF24_DATARATE);
  radio.setCRCLength(RF24_CRCLENGTH);
  radio.maskIRQ(1, 1, 0);

  radio.openReadingPipe(1, base_address);
  pinMode(RADIO_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RADIO_INT_PIN), radioINT, LOW);
  radio.startListening();
#ifdef SERIAL_DEBUG
  radio.printDetails();
#endif

#ifdef SD_CARD
  if (!card.begin(SD_CS)) {
#ifdef SERIAL_DEBUG
    Serial.print(F("SD Init error..."));
#endif
  } else {
#ifdef SERIAL_DEBUG
    Serial.print(F("SD Initialized..."));
#endif
    if (!Fat16::init(&card)) {
#ifdef SERIAL_DEBUG
      Serial.print(F("SD Fat16 init error..."));
#endif
    } else {
#ifdef SERIAL_DEBUG
      Serial.print(F("SD Fat16 initialized..."));
#endif
      sd_card_in = true;
      file.writeError = false;
    }
  }
#endif

#ifdef TIME_MENU
  set_position(LATITUDE * ONE_DEGREE, LONGITUDE * ONE_DEGREE);
  set_zone(TIMEZONE * ONE_HOUR);
#endif

  //config encoder interrupts
  PcInt::attachInterrupt(ENCBTN, encoderBTN_INT, CHANGE);
  PcInt::attachInterrupt(ENCA, encoderA_INT, CHANGE);
  PcInt::attachInterrupt(ENCB, encoderB_INT, CHANGE);

  //lcd.noBacklight();

  syncRTC();
  readBME();

#ifdef SERIAL_DEBUG
  Serial.println("Done!");
  radio.printDetails();
  delay(1000);
#endif
}

/******************************/
/******************************/
//      Helper functions      //
/******************************/
/******************************/

bool resolveMenu() {
  if (encoder.position) {
    currentMenu += encoder.position;
    encoder.position = 0;
    while (currentMenu < 0)currentMenu += nMenus; //fast rotation proof
    currentMenu %= nMenus;
    return true;
  }
  return false;
}

void resolveRadioPacket() {
  for (uint8_t i = 0; i < RADIO_NODES; ++i) {
    if (packet.address == node_address[i]) {
      lastPacketTime[i] = millis() / 1000;
      //memcpy(&sensorData.outside[1], &sensorData.outside[0], (BUFFER_SIZE-1)*sizeof(nodeStruct));
      memcpy(&sensorData.outside[i], &packet, sizeof(frame));
      sensorData.outside[i].temperature += tempCompA[i] * sensorData.outside[i].temperature + tempCompB[i];
      sensorData.outside[i].voltage *= BAT_V_SCALE;

      //log to the SD card
#ifdef SD_CARD
      char buf[16], tmp[6];
      memset(buf, 0, 14);
      itoa(node_address[i], tmp, HEX);
      strcat (buf, tmp);
      strcat (buf, ",");
      dtostrf(sensorData.outside[i].temperature, 5, 2, tmp);
      strcat (buf, tmp);
      strcat (buf, ",");
      itoa(sensorData.outside[i].voltage, tmp, DEC);
      strcat (buf, tmp);
      strcat (buf, ",");
      itoa(sensorData.outside[i].power, tmp, DEC);
      strcat (buf, tmp);
      logToSdCard(buf);
#endif
      break;
    }
  }
}

void syncRTC() {
  static uint32_t lastClockSync = 0;
  if (millis() - lastClockSync > clockSyncInterval) {
    lastClockSync = millis();
    DateTime sync = RTC.now();
    memcpy(&now, &sync, sizeof(DateTime));
  }
}

void readBME() {
  //shift sensor buffer
  //memcpy(&sensorData.inside[1], &sensorData.inside[0], (BUFFER_SIZE - 1)*sizeof(homeStruct));
  bme.read(sensorData.inside.pressure, sensorData.inside.temperature, sensorData.inside.humidity, tempUnit, presUnit);

  //Log to sd card
#ifdef SD_CARD
  char buf[5];
  itoa((uint16_t)sensorData.inside.pressure, buf, DEC);
  logToSdCard(buf);
#endif
}

#ifdef SD_CARD
void logToSdCard(char str[]) {
  if (sd_card_in) {
    if (dayOfLastLog != now.day()) {
      //prepare new filename
      char tmp[3];
      memset(fileName, 0, 8);
      itoa(now.year() - 2000, tmp, DEC);
      strcat (fileName, tmp);
      if (now.month() < 10) strcat(fileName, "0");
      itoa(now.month(), tmp, DEC);
      strcat (fileName, tmp);
      if (now.day() < 10) strcat(fileName, "0");
      itoa(now.day(), tmp, DEC);
      strcat (fileName, tmp);
      strcat (fileName, ".txt");
      dayOfLastLog = now.day();
#ifdef SERIAL_DEBUG
      Serial.print(F("Filename changed to: "));
      Serial.println(fileName);
#endif
    }

    //start logging
    cli();
    if (!file.open(fileName, O_CREAT | O_WRITE | O_AT_END | O_APPEND)) {
#ifdef SERIAL_DEBUG
      Serial.println(F("Log file open() error"));
#endif
    } else {
#ifdef SERIAL_DEBUG
      Serial.println(F("Log file opened"));
#endif
      //log data
      file.print(now.hour());
      file.write(",");
      file.print(now.minute());
      file.write(",");
      file.println(str);
      file.close();

#ifdef SERIAL_DEBUG
      Serial.println(F("Log file closed"));
#endif

      if (!file.sync() || file.getWriteError()) {
#ifdef SERIAL_DEBUG
        Serial.println(F("Log file sync() error"));
#endif
      }

    }
    sei();
  }
}
#endif

void setCurs(uint8_t pos, bool active = 0) {
  //clear cursor columns
  for (uint8_t i = 0; i < LCD_ROWS; ++i) {
    lcd.setCursor(0, i);
    lcd.print(F("  "));
  }
  lcd.setCursor(0, pos);
  if (active)lcd.print(F("->"));
  else lcd.print(F(">"));
}

//buzzer alarm is incorporated into this function:
void writeSingleTemp(float t) {
  if (t < alarmThreshold && alarmOn) { //blink if temp is lower than threshold
    if ((millis() % 1000) > BLINK_OFF) {
      if (t >= 0)lcd.print(F("+"));
      lcd.print(t, 1);
      lcd.write(C_DEG);
      lcd.print(F("C "));
      analogWrite(BEEP, 255);
    } else {
      lcd.print(F("        "));
      analogWrite(BEEP, 0);
    }
  } else {
    if (t >= 0)lcd.print(F("+"));
    lcd.print(t, 1);
    lcd.write(C_DEG);
    lcd.print(F("C "));
  }
}

void writeTemp() {
  lcd.setCursor(0, 0);
  writeSingleTemp(sensorData.outside[0].temperature);
  lcd.setCursor(0, 1);
  writeSingleTemp(sensorData.outside[1].temperature);
  lcd.setCursor(0, 2);
  writeSingleTemp(sensorData.outside[2].temperature);
  lcd.setCursor(0, 3);
  writeSingleTemp(sensorData.outside[3].temperature);
}

void writeLate() {
  now_t = millis() / 1000;
  lcd.setCursor(8, 0);
  if (now_t - lastPacketTime[0] > LATE_TX_TIME) {
    lcd.write(C_LATE);
  } else lcd.print(F(" "));

  lcd.setCursor(8, 1);
  if (now_t - lastPacketTime[1] > LATE_TX_TIME) {
    lcd.write(C_LATE);
  } else lcd.print(F(" "));

  lcd.setCursor(8, 2);
  if (now_t - lastPacketTime[2] > LATE_TX_TIME) {
    lcd.write(C_LATE);
  } else lcd.print(F(" "));

  lcd.setCursor(8, 3);
  if (now_t - lastPacketTime[3] > LATE_TX_TIME) {
    lcd.write(C_LATE);
  } else lcd.print(F(" "));
}


void writeInsideSensors() {
  lcd.setCursor(10, 1);
  lcd.write(C_HOME);
  lcd.print(F(" "));
  lcd.print(sensorData.inside.temperature, 1);
  lcd.write(C_DEG);
  lcd.print(F("C"));

  lcd.setCursor(10, 2);
  lcd.print(F("p "));
  lcd.print(sensorData.inside.pressure, 0);
  lcd.print(F("hPa "));

  lcd.setCursor(10, 3);
  lcd.write(C_DROP);
  lcd.print(F(" "));
  lcd.print(sensorData.inside.humidity, 1);
  lcd.print(F("%"));
}

void writeSingleVolt(uint16_t t) {
  lcd.print(t);
  lcd.print(F("mV"));
}

void writeVolt() {
  // each value is put in another quarter of the display
  lcd.setCursor(0, 0);
  lcd.print(sensorData.outside[0].voltage);
  lcd.print(F("mV"));
  lcd.setCursor(0, 1);
  lcd.print(sensorData.outside[1].voltage);
  lcd.print(F("mV"));
  lcd.setCursor(0, 2);
  lcd.print(sensorData.outside[2].voltage);
  lcd.print(F("mV"));
  lcd.setCursor(0, 3);
  lcd.print(sensorData.outside[3].voltage);
  lcd.print(F("mV"));

  lcd.setCursor(8, 0);
  lcd.print(sensorData.outside[0].power);
  lcd.setCursor(8, 1);
  lcd.print(sensorData.outside[1].power);
  lcd.setCursor(8, 2);
  lcd.print(sensorData.outside[2].power);
  lcd.setCursor(8, 3);
  lcd.print(sensorData.outside[3].power);
}

void writeLastPacketTime() {
  now_t /= 1000; //change to seconds
  lcd.setCursor(10, 0);
  lcd.print((uint16_t)(now_t - lastPacketTime[0]));
  lcd.print(F("s     "));
  lcd.setCursor(10, 1);
  lcd.print((uint16_t)(now_t - lastPacketTime[1]));
  lcd.print(F("s     "));
  lcd.setCursor(10, 2);
  lcd.print((uint16_t)(now_t - lastPacketTime[2]));
  lcd.print(F("s     "));
  lcd.setCursor(10, 3);
  lcd.print((uint16_t)(now_t - lastPacketTime[3]));
  lcd.print(F("s     "));
}

void writeTime() {
  lcd.setCursor(12, 0);
  if (now.hour() < 10)lcd.print(F("0"));
  lcd.print(now.hour());
  lcd.print(F(":"));
  if (now.minute() < 10)lcd.print(F("0"));
  lcd.print(now.minute(), DEC);
  lcd.print(F(":"));
  if (now.second() < 10)lcd.print(F("0"));
  lcd.print(now.second(), DEC);
}

#ifdef TIME_MENU
void writeDate() {
  lcd.setCursor(0, 0);
  if (now.day() < 10)lcd.print(F("0"));
  lcd.print(now.day());
  lcd.print(F("."));
  if (now.month() < 10)lcd.print(F("0"));
  lcd.print(now.month(), DEC);
  lcd.print(F("."));
  lcd.print(now.year(), DEC);
}

void writeEph() {
  lcd.setCursor(0, 1);
  lcd.print(F("sunrise:"));
  lcd.print(time_s1);

  lcd.setCursor(0, 2);
  lcd.print(F("sunset: "));
  lcd.print(time_s2);

  lcd.setCursor(0, 3);
  lcd.print(F("Moon:    "));
  if (moonPhase > 0)lcd.print(F("+"));
  lcd.print(moonPhase);
  lcd.print(F("%"));
}
#endif

void printSettings() {
  lcd.setCursor(2, 0);
  lcd.print(F("Alarm: "));
  if (alarmOn)lcd.print(F("ON "));
  else lcd.print(F("OFF"));

  lcd.setCursor(2, 1);
  lcd.print(F("Temp. al.: "));
  lcd.print(alarmThreshold, 1);
  lcd.write(C_DEG);
  lcd.print(F("C "));

  lcd.setCursor(2, 2);
  lcd.print(F("Radio: "));
  lcd.print(PA_Level[radio.getPALevel()]);
  lcd.print(F("  "));

  lcd.setCursor(2, 3);
  lcd.print(F("Wyjscie"));
}

/******************************/
/******************************/
//            MENUS           //
/******************************/
/******************************/

void mainMenu() {
#ifdef SERIAL_DEBUG
  Serial.println(F("Main menu"));
#endif
  lcd.clear();

  if (alarmOn) {
    lcd.setCursor(10, 0);
    lcd.write(C_ALARM);
  }

#ifdef SD_CARD
  if (sd_card_in) {
    lcd.setCursor(19, 3);
    lcd.write(C_SDCARD);
  }
#endif

  syncRTC();
  writeTemp();
  writeTime();
  writeInsideSensors();

  while (1) {
    now_t = millis();
    if (now_t - bmeLastUpdate > bmeUpdateInterval) {
      bmeLastUpdate = now_t;
      readBME();
    }
    if (now_t - lastMenuUpdate > menuUpdateInterval) {
      lastMenuUpdate = now_t;

      syncRTC();
      writeTemp();
      writeLate();
      writeTime();
      writeInsideSensors();
    }
    if (encoder.button) {
      settingsMenu();
      return;
    }
    if (resolveMenu()) return;
  }
}

void batteryMenu() {
#ifdef SERIAL_DEBUG
  Serial.println(F("Battery menu"));
#endif
  analogWrite(BEEP, 0); //turn off the buzzer
  lcd.clear();
  while (1) {
    now_t = millis();
    if (now_t - lastMenuUpdate > menuUpdateInterval) {
      lastMenuUpdate = now_t;
      writeVolt();
      writeLastPacketTime();
    }
    if (resolveMenu())return;
  }
}

#ifdef TIME_MENU
void timeMenu() {
#ifdef SERIAL_DEBUG
  Serial.println(F("Time menu"));
#endif
  analogWrite(BEEP, 0); //turn off the buzzer
  lcd.clear();

  static uint8_t dayOfUpdate = 0;

  syncRTC();
  writeDate();
  writeTime();

  if (dayOfUpdate) {
    writeEph();
  }

  while (1) {
    if (millis() - lastMenuUpdate > menuUpdateInterval) {
      lastMenuUpdate = millis();

      syncRTC();
      writeTime();

      //compute new values if the day has changed
      if (dayOfUpdate != now.day()) {
        long n;
        time_t now_t, noon_t, eph_t;
        struct tm local_tm;
        char tmp[3];

        //Compute ephemerals
        now_t = now.unixtime();
        noon_t = solar_noon(&now_t) + 3600; //add 1 hour to compensate for DST
        n = daylight_seconds(&now_t) / 2L;

        //Sunrise
        eph_t = noon_t - n;
        localtime_r(&eph_t, &local_tm);
        memset(time_s1, 0, 6);
        if (local_tm.tm_hour < 10) {
          itoa(0, tmp, DEC);
          strcat(time_s1, tmp);
        }
        itoa(local_tm.tm_hour, tmp, DEC);
        strcat(time_s1, tmp);
        strcat(time_s1, ":");
        if (local_tm.tm_min < 10) {
          itoa(0, tmp, DEC);
          strcat(time_s1, tmp);
        }
        itoa(local_tm.tm_min, tmp, DEC);
        strcat(time_s1, tmp);

        //Sunset
        eph_t = noon_t + n;
        localtime_r(&eph_t, &local_tm);
        memset(time_s2, 0, 6);
        if (local_tm.tm_hour < 10) {
          itoa(0, tmp, DEC);
          strcat(time_s2, tmp);
        }
        itoa(local_tm.tm_hour, tmp, DEC);
        strcat(time_s2, tmp);
        strcat(time_s2, ":");
        if (local_tm.tm_min < 10) {
          itoa(0, tmp, DEC);
          strcat(time_s2, tmp);
        }
        itoa(local_tm.tm_min, tmp, DEC);
        strcat(time_s2, tmp);

        //Moon phase
        moonPhase = moon_phase(&now_t);

        //print new date and ephemerals
        writeDate();
        writeEph();

        //update the flag
        dayOfUpdate = now.day();
      }
    }
    if (resolveMenu())return;
  }
}
#endif

void settingsMenu() {
  while (encoder.button) {} //wait for the button release
#ifdef SERIAL_DEBUG
  Serial.println(F("Settings menu"));
#endif
  analogWrite(BEEP, 0); //turn off the buzzer
  lcd.clear();

  static int8_t encoderLast;
  static uint8_t paLevel = 0;
  encoder.position = 3; //default menu - exit for fast checking
  encoderLast = 0; //force first transition print

  while (1) {
    if (encoderLast != encoder.position) { //if encoder moved
      //normalize the encoder range to [0, entries-1]
      while (encoder.position < 0)encoder.position += settingsMenuEntries;
      encoder.position %= settingsMenuEntries;
      encoderLast = encoder.position;
      printSettings();
      setCurs(encoderLast);
    }
    if (encoder.button) { //if button pushed
      while (encoder.button) {} //wait for the button release
      encoder.position = 0;
      switch (encoderLast) {
        case 0:
          alarmOn = !alarmOn;
          printSettings();
          break;
        case 1:
          setCurs(1, 1);
          while (1) {
            if (encoder.position) {
              alarmThreshold += ALARM_INC * encoder.position;
            }
            encoder.position = 0;
            printSettings();
            if (encoder.button) {
              while (encoder.button) {} //wait for the button release
              break;
            }
          }
          encoder.position = encoderLast;
          setCurs(encoder.position);
          break;
        case 2:
          setCurs(2, 1);
          while (1) {
            if (encoder.position) {
              paLevel += encoder.position;
              while (paLevel < 0)paLevel += PA_LEVELS;
              paLevel %= PA_LEVELS;
              radio.setPALevel(paLevel);
            }
            encoder.position = 0;
            printSettings();
            if (encoder.button) {
              while (encoder.button) {} //wait for the button release
              break;
            }
          }
          encoder.position = encoderLast;
          setCurs(encoder.position);
          break;
        case 3:
          return;
          break;
        default:
          break;
      }
    }
  }
}

/******************************/
/******************************/
//         Interrupts         //
/******************************/
/******************************/

void encoderBTN_INT() {
  delay (2); //debounce
  encoder.button = !digitalRead(ENCBTN);
}

void encoderA_INT() {
  delay (2); //debounce
  if (digitalRead(ENCA) == LOW) {
    if (digitalRead(ENCB) == HIGH) {
      ++encoder.position;
    }
  }
}

void encoderB_INT() {
  delay (2); //debounce
  if (digitalRead(ENCB) == LOW) {
    if (digitalRead(ENCA) == HIGH) {
      --encoder.position;
    }
  }
}

void radioINT() {
  cli();
  bool tx, fail, rx;
  radio.whatHappened(tx, fail, rx);

#ifdef SERIAL_DEBUG
  Serial.println(F("Radio IRQ"));
#endif

  /*if ( tx ) {
    // Have we successfully transmitted?
    Serial.println(F("Ack Payload:Sent"));
    }

    if ( fail ) {
    // Have we failed to transmit?
    Serial.println(F("Ack Payload:Failed"));
    }*/

  if ( rx || radio.available()) {
    radio.read( &packet, frame_size);
#ifdef SERIAL_DEBUG
    Serial.print(packet.address, HEX);
    Serial.print(F(" "));
    Serial.print(packet.temperature);
    Serial.print(F(" "));
    Serial.print(packet.voltage);
    Serial.print(F(" "));
    Serial.println(packet.power);
#endif
    resolveRadioPacket();
  }
  sei();
}

/******************************/
/******************************/
//         Main looop         //
/******************************/
/******************************/

void loop() {
  menu_func[currentMenu]();
} // end loop()

