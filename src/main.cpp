#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include "LittleFS.h"
#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

#define BUTTON_PIN              19
#define LED_PIN                 LED_BUILTIN
#define RELAY_PIN               27

#define POWER_PERIOD            0.25         // Laika sprīdis (s), kurā tiek mērīta vidēja jauda
#define MQTT_MESSAGE_PERIOD     30           // Laika intervāls, starp kuriem ir izsūtītas MQTT ziņas
#define SAMPLING_PERIOD         200          // Laika sprīdis (us) starp diviem jaudas mērījumiem
#define CURRENT_COEFFICIENT     1             // Koeficients, kuru izmanto, lai pārveidotu ADC strāvas vērtību ampēros
#define VOLTAGE_COEFFICIENT     1        // Koeficients, kuru izmanto, lai pārveidotu ADC sprieguma vērtību voltos
#define SETTINGS_LED_PERIOD     500
#define BUTTON_CLICK_TIMEOUT    500
#define ADC_WIDTH               ADC_WIDTH_BIT_11
#define ADC_VOLTAGE             ADC1_CHANNEL_6
#define ADC_CURRENT             ADC1_CHANNEL_4
#define VOLTAGE_ZERO            847
#define CURRENT_ZERO            847

#define DEFAULT_AP_SSID         "POWER PETER"
#define MQTT_ID                 "aabbccddeeff"

bool relayState = 0;                                // Releja pašreizējais stāvoklis
volatile bool messageToBeSent = 0;                  // Vai MQTT ziņu vajadzēs nosūtīt tuvakā laikā?
bool areMQTTSettingsProvided = 0;                   // Vai MQTT iestatījumi ir ievadīti?
volatile double lastRealPower = 0;                  // Pēdējā nomērīta aktīvas jaudas vērtība (W)
volatile double lastApparentPower = 0;              // Pēdējā nomērīta kopējās jaudas vērtība (VA)
volatile double lastReactivePower = 0;              // Pēdējā nomērīta reaktīvas jaudas vērtība (VA)
volatile float lastPowerFactor = 0;                 // Pēdējā nomērīta jaudas koeficienta vērtība
volatile double lastEnergy = 0;                     // Paterētā enerģija (kWh)
volatile double lastVoltageRMS = 0;                 // Pēdējā nomērīta sprieguma RMS vērtība (V)
volatile double lastCurrentRMS = 0;                 // Pēdējā nomērīta stravas stipruma RMS vērtība (A)
volatile bool measurementsProcessed = false; 

char wifiSSID[51] = "";
char wifiPassword[51] = "";
char mqttHost[51] = "";
char mqttUsername[51] = "";
char mqttPassword[51] = "";
char mqttPublishTopic[101] = "power_meter";
char mqttCommandTopic[101] = "power_meter/set";

bool settingsInputRegime = false;

const uint8_t atLeastOneTimeSet = 60;
const uint16_t measurementSampleAmount = POWER_PERIOD / (SAMPLING_PERIOD * 0.0000005);
const float measurementFrequency = 1 / POWER_PERIOD;
const uint16_t mqttSampleAmount = MQTT_MESSAGE_PERIOD / POWER_PERIOD;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer server(80);
hw_timer_t* timer = NULL; 
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


IRAM_ATTR double roundDouble(double number, uint16_t ndigits) {
  int8_t coeff = number >= 0;
  return ((int) (number + 0.5 * coeff) / (10 * ndigits)) / (10.0 * ndigits);
}

/*
  Pārbauda, vai spiedpoga tagad ir nospiesta.
  Ja spiedpoga tagad ir nospiesta, atgriež `true`.
*/
bool checkButton() {
  return !digitalRead(BUTTON_PIN); // Lasam pogas pina stāvokli
}

/*
  Saglabā WiFi un MQTT iestatījumus
*/
void putDataIntoEEPROM() {
  uint16_t addr = 0; // Adrese EEPROM atmiņā

  // EEPROM atmiņā ierakstam kādu skaitlisku vērtību, lai pārbaudītu, ka ierīce
  // kaut vienu reizi kaut ko tur ierakstīja
  EEPROM.put(addr, atLeastOneTimeSet);
  addr += sizeof(atLeastOneTimeSet); // Palielinām adresi par mainīga atmiņas apjomu

  EEPROM.put(addr, wifiSSID);
  addr += sizeof(wifiSSID);

  EEPROM.put(addr, wifiPassword);
  addr += sizeof(wifiPassword);

  EEPROM.put(addr, mqttHost);
  addr += sizeof(mqttHost);

  EEPROM.put(addr, mqttUsername);
  addr += sizeof(mqttUsername);

  EEPROM.put(addr, mqttPassword);
  addr += sizeof(mqttPassword);

  EEPROM.put(addr, mqttCommandTopic);
  addr += sizeof(mqttCommandTopic);

  EEPROM.put(addr, mqttPublishTopic);
  addr += sizeof(mqttPublishTopic);

  // Ierakstam izmainītus datus EEPROM atmiņā
  EEPROM.commit(); 
}

/*
  Nolasa WiFi un MQTT iestatījumus no EEPROM atmiņās,
  un ieliec tos mainīgajos operatīvajā atmiņā.
*/
void readDataFromEEPROM() {
  uint16_t addr = 0;

  uint8_t firstByte = 0;
  EEPROM.get(addr, firstByte); // Lasam pirmo EEPROM bitu

  // Ja pirmā bita vērtība nesakrīt ar programma iestatīto, tad
  // tas nozīmē, ka ierīce ir ieslēgta pirmo reizi.
  // Saglabājam noklusētus iestatījumus EEPROM atmiņā
  if (firstByte != atLeastOneTimeSet) putDataIntoEEPROM();
  addr += 1;

  EEPROM.get(addr, wifiSSID); // Lasam WiFi SSID no EEPROM atmiņas
  addr += sizeof(wifiSSID);

  EEPROM.get(addr, wifiPassword);
  addr += sizeof(wifiPassword);

  EEPROM.get(addr, mqttHost);
  addr += sizeof(mqttHost);

  EEPROM.get(addr, mqttUsername);
  addr += sizeof(mqttUsername);

  EEPROM.get(addr, mqttPassword);
  addr += sizeof(mqttPassword);

  EEPROM.get(addr, mqttCommandTopic);
  addr += sizeof(mqttCommandTopic);

  EEPROM.get(addr, mqttPublishTopic);
  addr += sizeof(mqttPublishTopic);
}

/*
  Vada ierīces gaismas diodi.
*/
void commandLED(uint8_t command) {
  if (command == 2) { 
    // Ja komanda ir 2, pārslēdzam gaismas diodi
    // digitalWrite - iestata pina loģisko līmeni
    // digitalRead - lasa pina esošo loģisko līmeni
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  } else if (command == 0 || command == 1) {
    digitalWrite(LED_PIN, command);
  }
}

/*
  Atzīmē, ka nakošajā `loop()` iterācijā, ir jaizsūtā
  jauno MQTT ziņu, jo kādi dati ir mainījusies.
*/
IRAM_ATTR void prepareMQTTMessageSend() {
  messageToBeSent = 1;
}

/*
  Vada releju un nodrošinā atbilstošu gaismas diodes indikāciju.
*/
void commandRelay(uint8_t command) {
  bool newState;
  if (command == 2) {
    newState = !relayState;
  } else if (command == 0 || command == 1) {
    newState = command;
  } else {
    return;
  }

  digitalWrite(RELAY_PIN, newState);
  if (relayState != newState) {
    prepareMQTTMessageSend();
    commandLED(newState);
  }
  relayState = newState;
}

/*
  Nosūtā MQTT ziņu uz MQTT brokeri.
*/
void sendNewMQTTMessage() {
  if (!messageToBeSent) return;
  if (!areMQTTSettingsProvided) return;

  JsonDocument doc; // Izveido jauno JSON objektu
  doc["relay"] = relayState;
  doc["real_power"] = roundDouble(lastRealPower, 1);
  doc["apparent_power"] = roundDouble(lastApparentPower, 1);
  doc["reactive_power"] = roundDouble(lastPowerFactor, 1);
  doc["power_factor"] = roundDouble(lastPowerFactor, 2);
  doc["voltage"] = (int) lastVoltageRMS;
  doc["current"] = roundDouble(lastCurrentRMS, 1);
  doc["energy"] = lastEnergy;

  char message[201]; // Izveido MQTT ziņas buferi
  serializeJson(doc, message); // Pārveido JSON objektu virknē

  mqttClient.publish(mqttPublishTopic, message); // Nosūtā JSON caur MQTT

  messageToBeSent = false; // Marķē ziņu kā izsūtītu
}

/*
  Nolasa MQTT ziņu un vada releju, balstoties uz iegūtām komandam.
*/
void handleMQTTMessage(char* topic, byte* payload, unsigned int length) {
  if (length > 10) return;
  
  char* command = (char*) malloc(length); // Allocate memory for command
  memcpy(command, payload, length); // Ieraksta komandu atmiņā

  // Salidzinā komandas, un atbilstoši vada releju
  if (memcmp(command, "ON", 2) == 0) {
    commandRelay(1);
  }
  else if (memcmp(command, "OFF", 3) == 0) {
    commandRelay(0);
  }
  else if (memcmp(command, "TOGGLE", 6) == 0) {
    commandRelay(2);
  }

  free(command); // Atbrīvo ziņas operatīvo atmiņu
}

/*
  Nolasa no ADC sprieguma vērtību, atgriež to.
*/
IRAM_ATTR int getVoltage() {
  uint16_t adcMeasurement = adc1_get_raw(ADC_VOLTAGE);
  int16_t adcValue = adcMeasurement - VOLTAGE_ZERO;
  return adcValue;
}

/*
  Nolasa no ADC stravas stipruma vērtību, atgriež to.
*/
IRAM_ATTR int getCurrent() {
  uint16_t adcMeasurement = adc1_get_raw(ADC_CURRENT);
  int16_t adcValue = adcMeasurement - CURRENT_ZERO;
  return adcValue;
}

/*
  Aprēķinā jaudas un patērētas enerģijas vērtības. 
  Atgriež `true`, ja kāda no iepriekšējām vērtībām atšķiras no esošām.
*/
IRAM_ATTR bool calculatePowerEnergy(int voltage, int current) {
  static uint16_t counter = 0;
  static int64_t power = 0;
  static uint64_t voltageSquared = 0;
  static uint64_t currentSquared = 0;

  int32_t currentPower = voltage * current;
  power += currentPower;
  voltageSquared += voltage * voltage;
  currentSquared += current * current;
  
  if (counter >= measurementSampleAmount) {
    lastRealPower = double(power / measurementSampleAmount) * CURRENT_COEFFICIENT * VOLTAGE_COEFFICIENT;
    // sqrt() - aprēķina kvadratsakni no skaitļa
    lastVoltageRMS = sqrt(voltageSquared / measurementSampleAmount) * VOLTAGE_COEFFICIENT;
    lastCurrentRMS = sqrt(currentSquared / measurementSampleAmount) * CURRENT_COEFFICIENT;
    lastApparentPower = lastVoltageRMS * lastCurrentRMS;
    lastReactivePower = sqrt(lastApparentPower * lastApparentPower - lastRealPower * lastRealPower);
    lastPowerFactor = lastRealPower / lastApparentPower;
    lastEnergy += lastRealPower * POWER_PERIOD / 3.6e6;

    counter = 0;
    power = 0;
    voltageSquared = 0;
    currentSquared = 0;

    return true;
  }
  else {
    counter += 1;
    return false;
  }
}

/*
  Callback funkcija, kuru izsauc taimeris. Tā īsteno jaudas un enerģijas mērīšanu.
*/
IRAM_ATTR void handleMeasurement() {
  portENTER_CRITICAL_ISR(&timerMux);
  static uint16_t mqttCounter = 0;
  int voltage = getVoltage();
  int current = getCurrent();
  bool powerPeriodFullfilled = calculatePowerEnergy(voltage, current);

  if (powerPeriodFullfilled && mqttCounter >= mqttSampleAmount) {
    prepareMQTTMessageSend();
    mqttCounter = 0;
  } else if (powerPeriodFullfilled) {
    mqttCounter += 1;
  }

  portEXIT_CRITICAL_ISR(&timerMux); 
}

/*
  Ja `HTTP` POST formā ir ierakstīts iestatījums, atgriež to, 
  ja nav, tad atgriež noklusēto vērtību.
*/
String updateSetting(String name, String defaultValue) {
  String value = server.arg(name); // Saņēm POST argumenta vērtību no HTTP servera
  if (value) {
    return value;
  }
  else {
    return defaultValue;
  }
}

/*
  Lasa datni no LittleFS failu sistēmas.
*/
String readFile(const char* filename) {
  // Saņēm faila objektu no LittleFS failu sistēmas
  File file = LittleFS.open(filename, "r");

  // Nolasa failu virknē
  String data = file.readString();

  // Aizvēr faila lasīšanu
  file.close();

  return data;
}

/*
  Atgriež iestatījumu lapu klientam.
*/
void httpHandleSettings() {
  // server.send() nosūtā HTTP klientam atbildi
  server.send(200, "text/html", readFile("/settings.html"));
}

/*
  Atgriež sākuma lapu klientam.
*/
void httpHandleIndex() {
  server.send(200, "text/html", readFile("/index.html"));
}

/*
  Iepako JSON dokumentā datus par ierīces stāvokli,
  un nosūtā to HTTP klientam.
*/
void httpHandleApiData() {
  JsonDocument doc;
  doc["relay"] = relayState;
  doc["real_power"] = roundDouble(lastRealPower, 1);
  doc["apparent_power"] = roundDouble(lastApparentPower, 1);
  doc["reactive_power"] = roundDouble(lastPowerFactor, 1);
  doc["power_factor"] = roundDouble(lastPowerFactor, 2);
  doc["voltage"] = (int) lastVoltageRMS;
  doc["current"] = roundDouble(lastCurrentRMS, 1);
  doc["energy"] = lastEnergy;

  char message[201];
  serializeJson(doc, message);

  server.send(200, "application/json", message);
}

/*
  Iepako JSON dokumentā ierīces iestatījumus,
  un nosuta to HTTP klientam.
*/
void httpHandleApiSettings() {
  JsonDocument doc;
  doc["wifi_ssid"] = wifiSSID;
  doc["wifi_pass"] = wifiPassword;
  doc["mqtt_host"] = mqttHost;
  doc["mqtt_username"] = mqttUsername;
  doc["mqtt_password"] = mqttPassword;
  doc["mqtt_command"] = mqttCommandTopic;
  doc["mqtt_publish"] = mqttPublishTopic;

  char message[801];
  serializeJson(doc, message);

  server.send(200, "application/json", message);
}

/*
  Lasa nosūtītus caur HTTP iestatījumus,
  un saglābā tos EEPROM atmiņā un operatīvajā atmiņā
*/
void httpHandleSettingsForm() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
  }
  else {
    updateSetting("wifi-ssid", "").toCharArray(wifiSSID, 51);
    updateSetting("wifi-pass", "").toCharArray(wifiPassword, 51);
    updateSetting("mqtt-host", "").toCharArray(mqttHost, 51);
    updateSetting("mqtt-username", "").toCharArray(mqttUsername, 51);
    updateSetting("mqtt-pass", "").toCharArray(mqttPassword, 51);
    updateSetting("mqtt-command", "").toCharArray(mqttCommandTopic, 101);
    updateSetting("mqtt-publish", "").toCharArray(mqttPublishTopic, 101);
    server.send(200, "text/plain", "ok");

    putDataIntoEEPROM();
    if (settingsInputRegime) {
      ESP.restart();
    }
  }
}

/*
  Īsteno speciālo iestatījumu ievadīšanas režīmu, saglāba iestatījumus.
*/
void settingsInput() {
  settingsInputRegime = true;

  bool result = WiFi.softAP(DEFAULT_AP_SSID); // Palaiž WiFi tīklu ar definēto nosaukumu
  Serial.print("Soft AP started: ");
  Serial.println(result);

  // server.on() piekārto pie noteikta URL noteikto funkciju, kas to apstrādā
  server.on("/settings/set", httpHandleSettingsForm);
  server.on("/api/settings", httpHandleApiSettings);
  server.on("/", httpHandleSettings);

  // Palaiž HTTP serveri
  server.begin();

  uint32_t lastMillis = millis();
  uint32_t newMillis;
  while (true) {
    newMillis = millis();

    if (newMillis - lastMillis > SETTINGS_LED_PERIOD) {
      commandLED(2);
      lastMillis = newMillis;
    }
    
    // Veic HTTP servera darbības
    server.handleClient();
  }
}

/*
  Pārslēdz releja stāvokli pēc lietotāja komandas caur HTTP.
*/
void httpHandleRelayToggle() {
  commandRelay(2);
  server.send(200, "text/plain", String(relayState));
}

/*
  Ieslēdz releju pēc lietotāja komandas caur HTTP.
*/
void httpHandleRelayOn() {
  commandRelay(1);
  server.send(200, "text/plain", String(relayState));
}

/*
  Izslēdz releju pēc lietotāja komandas caur HTTP.
*/
void httpHandleRelayOff() {
  commandRelay(0);
  server.send(200, "text/plain", String(relayState));
}

void setup() {
  Serial.println("Starting device initialisation..."); // Izvada paziņojumu komunikacijā ar datoru
  LittleFS.begin(); // Uzsāc LittleFS failu sistēmas darbību
  Serial.begin(230400); // Uzsāc komunikāciju caur Serial portu atkļūdošanai
  EEPROM.begin(512); // Iniciē EEPROM atmiņu 512 baitu apjomā

  readDataFromEEPROM();

  pinMode(BUTTON_PIN, INPUT_PULLUP); // Iestata pina režīmu
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  commandLED(0);
  commandRelay(0);

  if (!wifiSSID[0]) settingsInput();
  Serial.println("Settings input regime start due to lack of WiFi settings skipped");

  if (checkButton()) settingsInput();
  Serial.println("Settings input regime start due to pressed button skipped");

  Serial.print("Connecting to ");
  Serial.print(wifiSSID);

  WiFi.mode(WIFI_STA); // Iestata WiFi klienta darbības režīmu
  WiFi.begin(wifiSSID, wifiPassword); // Pieslēdzas pie WiFi tīkla

  while (WiFi.status() != WL_CONNECTED) { // Pieslēdzas pie WiFi klienta
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  if (mqttHost[0]) {
    areMQTTSettingsProvided = true;
    mqttClient.setServer(mqttHost, 1883);

    Serial.print("Connecting to MQTT broker @ ");
    Serial.print(mqttHost);

    if (mqttUsername[0] && mqttPassword[0]) {
      mqttClient.connect(MQTT_ID, mqttUsername, mqttPassword);
      Serial.print(" using username & password");
    }
    else {
      mqttClient.connect(MQTT_ID);
    }
    
    while (!mqttClient.connected()) {
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("MQTT connected");

    mqttClient.setCallback(handleMQTTMessage);
    mqttClient.subscribe(mqttCommandTopic);
  }
  else {
    Serial.println("MQTT settings are not provided");
    areMQTTSettingsProvided = false;
  }

  server.on("/api/data", httpHandleApiData);
  server.on("/api/settings", httpHandleApiSettings);
  server.on("/settings/set", httpHandleSettingsForm);
  server.on("/settings", httpHandleSettings);
  server.on("/", httpHandleIndex);
  server.on("/relay/toggle", httpHandleRelayToggle);
  server.on("/relay/on", httpHandleRelayOn);
  server.on("/relay/off", httpHandleRelayOff);
  server.begin();
  Serial.println("HTTP server had been set up");

  adc1_config_width(ADC_WIDTH);
  adc1_config_channel_atten(ADC_VOLTAGE, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC_CURRENT, ADC_ATTEN_DB_11);

  timer = timerBegin(3, 80, true); // Izveido laika taimera objektu
  spinlock_initialize(&timerMux);
  timerAttachInterrupt(timer, &handleMeasurement, true); // Piekarto pie taimera apstrādāšanas funkciju
  timerAlarmWrite(timer, SAMPLING_PERIOD, true); // Iestata taimera biežumu
  timerAlarmEnable(timer); // Palaiž taimeri
  Serial.println("Measurements had started");

}

void loop() {
  if (measurementsProcessed) return;

  static uint32_t lastButtonClick = 0;

  sendNewMQTTMessage();
  server.handleClient();
  mqttClient.loop();

  if (checkButton()) {
    if (millis() - lastButtonClick > BUTTON_CLICK_TIMEOUT) {
      commandRelay(2);
      lastButtonClick = millis();
    }
  }
}
