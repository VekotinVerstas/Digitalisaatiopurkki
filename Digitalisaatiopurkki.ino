/*
   TTGOV2 boards GPIO33 must be externaly connected to lora module DIO1 and GPIO32 should be connected to DIO2.

   This uses https://github.com/VekotinVerstas/arduino-lmic as lora library. Install it to your arduino/library first.
*/

#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include "settings.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

Adafruit_BME280 bme;
bool bmestatus = 0;

#define LEDPIN 2  // Used to blink blue led and to signal watchtog chip that all is well - while sending data

int wt = 0;
const int wdtTimeout = 12000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

float temp = 0, hum = 0;
float tempold = 0, humold = 0;

static char esp_id[16];
unsigned int counter = 0;
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 0.5 * 60;
char TTN_response[30];

void IRAM_ATTR resetModule() {
  Serial.print(F("SW reset WD or i2c fail: "));
  ets_printf("reboot\n");
  esp_restart_noos();
}

void setup() {
  Serial.begin(115200);
  timer = timerBegin(0, 8000, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);

  sprintf(s_id, "ESP_%02X", BOXNUM);
  Serial.print("Digitalisaatio purkkki name: ");
  Serial.println(s_id);

  Serial.print("Version: ");
  Serial.println(version);

  uint64_t chipid;
  chipid = ESP.getEfuseMac();
  Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(chipid >> 32));
  Serial.printf("%08X\n", (uint32_t)chipid);
  sprintf(esp_id, "%08X", (uint32_t)chipid);

  Serial.printf("LORA dev id: 0x%08X\n", (uint64_t)DEVADDR);
  Serial.printf("LORA dev eui: %01X%01X%01X%01X%01X%01X%01X%01X\n", DEVEUI[0], DEVEUI[1], DEVEUI[2], DEVEUI[3], DEVEUI[4], DEVEUI[5], DEVEUI[6], DEVEUI[7]);
  // Use the Blue pin to signal transmission.
  pinMode(LEDPIN, OUTPUT);

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  //LMIC_setDrTxpow(DR_SF11,14);
  LMIC_setDrTxpow(DR_SF9, 14);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  display.display();
  delay(10);
  
  bmestart(21, 22);

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
  displaybmedata( temp, hum );
}

void displaybmedata(float temp, float hum) {
  if ( temp == tempold && hum == humold ) return; // Do not update if not nessesary
  tempold = temp;
  humold = hum;

  char buf[5];
  hum = round(hum);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.clearDisplay();
  display.print("Lampo: ");
  display.println(temp);
  display.print("Kosteus: ");
  display.println(hum);
  display.display();
}

void bmestart(int pin1, int pin2) {
  Wire.begin(pin1, pin2);
  bmestatus = bme.begin(0x76);
  if (!bmestatus) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
  delay( 500 );
}

void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

void os_getArtEui (u1_t* buf) { }

void os_getDevKey (u1_t* buf) { }

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void do_send(osjob_t* j) {
  // Payload to send (uplink)
  digitalWrite(LEDPIN, HIGH);
  timerWrite(timer, 0); //reset timer (feed watchdog)

  temp = bme.readTemperature();
  hum = bme.readHumidity();
  Serial.println("*****************************************************************");
  Serial.print("TEMP: ");
  Serial.println(temp);
  Serial.print("HUM: ");
  Serial.println(hum, DEC);
  Serial.println("*****************************************************************");

  if ( temp < -50 ) {
    Serial.println("BME error - reboot.");
    resetModule();
  }

  char message[25];
  //char buff[25];
  sprintf( message, "temp=%.2f,hum=%.2f", temp, hum );

  Serial.println(message);
  digitalWrite(LEDPIN, LOW);
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (uint8_t*)message, strlen(message), 0);
    //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
    Serial.println(F("Sending uplink packet..."));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
  if (ev == EV_TXCOMPLETE) {
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK) {
      Serial.println(F("Received ack"));
    }
    if (LMIC.dataLen) {
      int i = 0;
      // data received in rx slot after tx
      Serial.print(F("Data Received: "));
      Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
      Serial.println();

      for ( i = 0 ; i < LMIC.dataLen ; i++ )
        TTN_response[i] = LMIC.frame[LMIC.dataBeg + i];
      TTN_response[i] = 0;
      LMIC.dataLen = 0;
    }
    // Schedule next transmission
    Serial.println("Schedule next transmission");
    Serial.flush();
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
  }
}
