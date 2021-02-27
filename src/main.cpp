#include <Arduino.h>

// Define Serial Port and GPIO pins.
#define RXD2 12 // connects to TX in Victron
#define TXD2 14 // connects to RX in Victron

// Define ESP Sleep time
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 3600        /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;

// Select your modem:
#define TINY_GSM_MODEM_SIM800 // Modem is SIM800L

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

// Your GPRS credentials, if any
const char apn[] = "giffgaff.com"; // APN of SIM provider.
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT details
const char* broker = "jaybro.duckdns.org"; // Public IP address or domain name
const char* mqttUsername = "jason";  // MQTT username
const char* mqttPassword = "QmjsdT6A922UAw";  // MQTT password

int serial_read_loop;
int gsm_retry_loop = 0;
int mqtt_retry_loop = 0;
byte willQoS = 0;
boolean willRetain = false;
const char* topicSerialData = "/jason/shed/esp/serialtx";
const char* topicStatus = "/jason/shed/esp/status";
const char* willTopic = "/jason/shed/esp/status";
const char* battery_charge_Topic = "/jason/shed/esp/battery/chargestate";
const char* battery_percent_Topic = "/jason/shed/esp/battery/percent";
const char* battery_voltage_Topic = "/jason/shed/esp/battery/voltage";
const char* willMessage = "Sleeping";
uint8_t  chargeState = -99;
int8_t   percent     = -99;
uint16_t milliVolts  = -9999;

#include <Wire.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26

//Get Battery statistics
void ttgo_stats() {
  modem.getBattStats(chargeState, percent, milliVolts);
  SerialMon.print("Battery charge state: ");
  SerialMon.println(chargeState);
  SerialMon.print("Battery charge 'percent': ");
  SerialMon.println(percent);
  SerialMon.print("Battery voltage: ");
  SerialMon.println(milliVolts / 1000.0F);
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  boolean status = mqtt.connect("SHED_ESP32", mqttUsername, mqttPassword,willTopic,willQoS,willRetain,willMessage);

  if (status == false) {
    SerialMon.println(" Connection failed.");
    return false;
  }
  SerialMon.println(" connected.");
  mqtt.publish(topicStatus, "Online");
  //mqtt.publish(battery_charge_Topic, chargeState);
  //mqtt.publish(battery_percent_Topic, percent);
  //mqtt.publish(battery_voltage_Topic, milliVolts);
  return mqtt.connected();
}

void start_gsm() {
  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" > connection failed.");
  }
  else {
    SerialMon.println("OK");
  }
  
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }

  // MQTT Broker setup with port.
  mqtt.setServer(broker, 12860);
}

void setup() {
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF); // Puts the board into hibernation.
  esp_sleep_enable_timer_wakeup(3600000000);

  // Set console baud rate for the debug console.
  SerialMon.begin(115200);
  SerialMon.setTimeout(500);// Set time out for
  delay(10);

  //setup Victron Serial connection.
  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  
  start_gsm(); // Start GSM Connection.
}

void loop() {
  ttgo_stats();
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    while ( gsm_retry_loop < 2) { // Try the GSM connection two more times.
    SerialMon.println(" Retrying GSM Connection. ");
    start_gsm();
    gsm_retry_loop++;
  }
  SerialMon.println(" Max GSM connection retries reached. Sleeping zzzzz");
  esp_deep_sleep_start(); // Sleep if GSM Retry timeouts.
  } else {
    mqttConnect(); // Start MQTT Server connection if GSM connection is successful.
  }
    
  if (!mqtt.connected()){
    while ( mqtt_retry_loop < 2) { // Try the MQTT connection two more times.
    SerialMon.println(" Retrying MQTT Connection. ");
    mqttConnect();
    mqtt_retry_loop++;
  }
  SerialMon.println(" Max MQTT server connection retries reached. Sleeping zzzzz");
  esp_deep_sleep_start(); // Sleep if GSM Retry timeouts.
  } else {
    SerialMon.println("Connected to MQTT Server, reading serial data. ");
    delay(1000);
    serial_read_loop = 0;
    while ( serial_read_loop < 21 ) {
      if (Serial2.available() > 0) {
        Serial2.setTimeout(500);
        SerialMon.println("Serial data recieved > ");
        char bfr[501];
        memset(bfr,0, 501);
        Serial2.readBytesUntil('\n',bfr,500);
        delay(500); // reduced from 2000
        if (serial_read_loop != 0) { // This will ignore the first serial read which is typically blank.
          mqtt.publish(topicSerialData, bfr);
          SerialMon.println(bfr);
          SerialMon.println(serial_read_loop);
          delay(500);
        }
      } else {
        SerialMon.println("Serial data is NOT available.");
      }
      serial_read_loop++; //Increase counter
    }  
  }
  
  SerialMon.println("Sleeping zzzzzzz");
  esp_deep_sleep_start();

}