#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "RF24.h"

//#define SERIAL_DEBUG

#define RF24_PA_LEVEL       RF24_PA_MIN
#define RF24_CHANNEL        17
#define RF24_DATARATE       RF24_250KBPS
#define RF24_CRCLENGTH      RF24_CRC_16
#define RF24_CE_PIN         9
#define RF24_CS_PIN         10

#define ADC_MOSFET_GATE     A1
#define ADC_INPUT           A0  //ADC input on ADC0 (A0)

#define ONE_WIRE_BUS        3

#define PWR_ADAPT_TRIES     5
#define FAILED_TXS_LIMIT    5
#define ON_SLEEP_CYCLES     7                   //1 cycle is 8 seconds + a little delay, 7 would be around 1 minute
#define OFF_SLEEP_CYCLES    15*ON_SLEEP_CYCLES

const byte base_address[6] = "baseX";
uint8_t node_address; //node address is later initialized with OneWire address of the attached DS18B20 temperature sensor 

struct frame {
  byte address;
  float temperature;
  uint16_t voltage;
  uint8_t power;
};

uint8_t failed_txs = 0;
const uint8_t frame_size = sizeof(frame);
volatile uint8_t cycles = 0;

RF24 radio(RF24_CE_PIN, RF24_CS_PIN);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  power_all_disable();
  power_spi_enable();
  power_timer0_enable();
  power_adc_enable();

#ifdef SERIAL_DEBUG
  power_usart0_enable();
  Serial.begin(9600);
  Serial.print("Initializing...");
#endif

  pinMode(ADC_INPUT, INPUT);
  pinMode(ADC_MOSFET_GATE, OUTPUT);
  digitalWrite(ADC_MOSFET_GATE, LOW);

  //Setup the NRF24L01 radio
  radio.begin();
  radio.setPALevel(RF24_PA_LEVEL);
  radio.setChannel(RF24_CHANNEL);
  radio.setDataRate(RF24_DATARATE);
  radio.setCRCLength(RF24_CRCLENGTH);
  radio.openWritingPipe(base_address);
  radio.startListening();

  //Setup the ADC
  ACSR |= (1 << ACD);
  ADMUX = 0x00;
  ADCSRA = 0x00;
  ADMUX |= (1 << REFS1) | (1 << REFS0); // internal VBG as voltage reference, ADC0 as external input
  ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //set ADC clock
  ADCSRB = (1 << ADTS1); //external conversion trigger

  //Setup the WDT
  MCUSR &= ~(1 << WDRF); // Clear the reset flag.
  WDTCSR |= (1 << WDCE) | (1 << WDE); //to change WDE or the prescaler, we need to set WDCE (This will allow updates for 4 clock cycles).
  WDTCSR = (1 << WDP3) | (1 << WDP0); // set the prescaler, so the period is 8.0 seconds
  WDTCSR |= (1 << WDIE); //enable WDT interrupts

  //Setup the DS18B20 temperature sensor
  delay(100);
  sensors.begin();
  sensors.setResolution(12);
  DeviceAddress temp;
  sensors.getAddress(temp, 0);
  node_address = temp[7];     //use sensor's hardware address as radio node identifier

#ifdef SERIAL_DEBUG
  Serial.println("Done!");
#endif

  sendData();
  sleep_tight(1);
  sendData();
  sleep_tight(OFF_SLEEP_CYCLES);
}

bool sendData() {
  //Switch on the MOSFET and enable ADC
  digitalWrite(ADC_MOSFET_GATE, HIGH);
  ADCSRA |= (1 << ADSC);

  while (bit_is_set(ADCSRA, ADSC));
  uint16_t node_voltage = ADCL;
  node_voltage += ADCH << 8;

  //Switch off the MOSFET and disable ADC
  digitalWrite(ADC_MOSFET_GATE, LOW);
  power_adc_disable();

  //Read temperature from the sensor
  sensors.requestTemperatures();
  float node_temp = sensors.getTempCByIndex(0);

  //Get NRF24L01+ PA level
  uint8_t node_power = radio.getPALevel();

  //Form a data packet
  frame packet = {node_address, node_temp, node_voltage, node_power};

#ifdef SERIAL_DEBUG
  Serial.print(F("Sending: "));
  Serial.print(node_address, HEX);
  Serial.print(F(" "));
  Serial.print(node_voltage);
  Serial.print(F(" "));
  Serial.print(node_temp);
  Serial.print(F(" "));
  Serial.println(node_power);
  delay(100); //give some time for serial to write
#endif

  //Send the packet by the radio
  radio.stopListening();
  bool tx_sent = radio.write( &packet, frame_size );
  radio.startListening();

/*
  bool tx_sent = false;
  bool tx_jam = radio.testCarrier();
    #ifdef SERIAL_DEBUG
    Serial.print("tx_sent=");
    Serial.print(tx_sent);
    delay(100);
    //Serial.print(" tx_jam=");
    //Serial.println(tx_jam);
    #endif
  */
  return tx_sent;
}

bool adaptPower() {
  for (int8_t tmp_power = RF24_PA_MAX; tmp_power >= RF24_PA_MIN; --tmp_power) {
    uint8_t tx_sent = 0;
    radio.setPALevel(tmp_power);

    for (uint8_t i = 0; i < PWR_ADAPT_TRIES; ++i) {
      tx_sent += sendData();
      sleep_tight(ON_SLEEP_CYCLES);
    }

    if (tx_sent == 0) {
      if (tmp_power == RF24_PA_MAX) {
        sleep_tight(OFF_SLEEP_CYCLES);  //assume RX is off and go for a long nap
        return false;
      } else {
        radio.setPALevel(tmp_power + 1); //adjust power to the last working level
        break;
      }
    }
  }
  return true;
}

void loop() {
  if (adaptPower()) {
    failed_txs = 0;
    while (failed_txs < FAILED_TXS_LIMIT) {
      if (sendData()) {
        failed_txs = 0; //reset counter
      } else {
        ++failed_txs;
      }
      sleep_tight(ON_SLEEP_CYCLES);
    }
  }
}

void sleep_tight(uint8_t sleep_cycles) {
  cycles = 0;

  radio.powerDown();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  /* Now enter sleep mode. */
  while (cycles != sleep_cycles) {
    sleep_mode();
  }
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  radio.powerUp();
  /* Re-enable the peripherals. */
  //power_all_enable();
  power_spi_enable();
  power_timer0_enable();
  power_adc_enable();
}

ISR(WDT_vect) {
  ++cycles;
}



