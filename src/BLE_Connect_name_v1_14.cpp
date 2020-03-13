
#include <Arduino.h>
#include <esp_now.h>
#include <ArduinoJson.h>

#include <BLEDevice.h>
#include <WiFi.h>

#include <Ticker.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

extern "C"
{
#include "esp_timer.h"
}

#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <AsyncMqttClient.h>

#include <string>
#include <esp_system.h>

#include "../secrets/credentials.h"

WiFiClient espClient;
AsyncMqttClient mqttClient;

Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;

/* 
This will repeat every 20s, disconnect wifi and connect ble for 5 seconds (scanTime).
after that time it will reconnect wifi and mqtt abd sent message and wait 20s to repeat.
There needs to be a boolean to enable the proccess to stop and leave wifi on 100%
*/
Ticker bleReconnectTimer;
// time of repeat seconds - initial 2 mins
int bleStartIn = 20; //120;

// Time Ble scan is on
int scanTime = 5; //In seconds

const char *esp32Ver = "V 1.14.2";
const char *topicIn = "thermo/out"; // Change for your own topic
const char *subTopic = "thermo/in/#";

// Memory allocated for capturing results from scan
char subject[10][80]; // Change for your own topic

char *mywifi;

// Store IP address
String localIp;

// if too many connections reboot esp32
byte retryAttempts = 0;

WiFiEventId_t wifiConnectHandler;
WiFiEventId_t wifiDisconnectHandler;

// if OTA in progress don't reconnect mqtt
bool updateInProgress = false;

char data1[10][120];
char *mqtt_client;
char baseMacChr[18] = {0};
int DeviceCount;

int ledPin = 2;
int powLedPin = 1;
int ledState = HIGH; // ledState used to set the LED

// #define BatteryService BLEUUID((uint16_t)0x180F)
bool otaUpdate = false;
bool bleOn = false;
bool deviceFound = false;
bool serviceFound = false;
unsigned long entry;

int mqttLoop = 0;
int mqttLoopTotal = 5; //max no of retries

#define tempService BLEUUID((uinit16_t)0xFE95)    //For ble Sensor
#define serviceUUIDNut BLEUUID((uint16_t)0x1803)  // immediate alert service
#define serviceUUIDItag BLEUUID((uint16_t)0xFFE0) // immediate alert service

// void WiFiEvent(WiFiEvent_t event);
// void connectToMqtt();
void startBLE();

BLEScan *pBLEScan;
// inizilise leds
// LedControl myLeds;

void getLocalMacAddress()
{
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
}

void BleTempHum(const std::string &serviceData, BLEAdvertisedDevice &advertisedDevice)
{
  char rtnMsg[128], rtnTopic[100], temp[6], hum[6];
  StaticJsonDocument<128> doc;

  dtostrf((serviceData[15] << 8 | serviceData[14]) / 10.0, 5, 2, temp);
  dtostrf((serviceData[17] << 8 | serviceData[16]) / 10.0, 5, 2, hum);

  sprintf(rtnTopic, "%s/%s", topicIn, (char *)advertisedDevice.getAddress().toString().c_str());

  doc["mac"] = (char *)advertisedDevice.getAddress().toString().c_str();
  doc["temp"] = temp;
  doc["hum"] = hum;
  doc["esp32"] = baseMacChr;

  serializeJson(doc, rtnMsg, 128);

  Serial.printf("Device Count: %i, Temp and Hum: ", DeviceCount);
  serializeJson(doc, Serial);
  Serial.println();

  mqttClient.publish(rtnTopic, 0, 0, rtnMsg);
}

void BleBattery(const std::string &serviceData, BLEAdvertisedDevice &advertisedDevice)
{
  char rtnMsg[128], rtnTopic[100], batt[6];
  int serviceCount = serviceData.length();
  StaticJsonDocument<128> doc;

  dtostrf((serviceData[serviceCount - 1]), 5, 2, batt);

  doc["mac"] = (char *)advertisedDevice.getAddress().toString().c_str();
  doc["batt"] = batt;
  doc["esp32"] = baseMacChr;

  sprintf(rtnTopic, "thermo/battery/%s", (char *)advertisedDevice.getAddress().toString().c_str());

  serializeJson(doc, rtnMsg, 128);

  Serial.printf("Device Count: %i, Battery: ", DeviceCount);
  serializeJson(doc, Serial);
  Serial.println();

  mqttClient.publish(rtnTopic, 0, 0, rtnMsg);
}

void BleClearGrass(const std::string &serviceData, BLEAdvertisedDevice &advertisedDevice)
{
  char rtnMsg[128], rtnTopic[100], temp[6], hum[6], batt[6];
  int serviceCount = serviceData.length();

  StaticJsonDocument<128> doc;

  dtostrf((serviceData[serviceCount - 6] << 8 | serviceData[serviceCount - 7]) / 10.0, 5, 2, temp);
  dtostrf((serviceData[serviceCount - 4] << 8 | serviceData[serviceCount - 5]) / 10.0, 5, 2, hum);
  dtostrf(serviceData[serviceCount - 1], 5, 2, batt);

  doc["mac"] = (char *)advertisedDevice.getAddress().toString().c_str();
  doc["temp"] = temp;
  doc["hum"] = hum;
  doc["batt"] = batt;
  doc["esp32"] = baseMacChr;

  serializeJson(doc, rtnMsg, 128);

  Serial.printf("Device Count: %i, ClearGeass: ", DeviceCount);
  serializeJson(doc, Serial);
  Serial.println();

  sprintf(rtnTopic, "%s/%s", topicIn, (char *)advertisedDevice.getAddress().toString().c_str());

  mqttClient.publish(rtnTopic, 0, 0, rtnMsg);

  sprintf(rtnTopic, "thermo/battery/%s", (char *)advertisedDevice.getAddress().toString().c_str());
  mqttClient.publish(rtnTopic, 0, 0, rtnMsg);
}

void BlePresenceDetection(BLEAdvertisedDevice &advertisedDevice)
{
  char rtnMsg[128];
  StaticJsonDocument<128> doc;
  char rtnTopic[100];

  doc["mac"] = (char *)advertisedDevice.getAddress().toString().c_str();
  doc["rssi"] = advertisedDevice.getRSSI();
  doc["esp32"] = baseMacChr;

  serializeJson(doc, rtnMsg, 128);

  Serial.printf("Device Count: %i, Presence: ", DeviceCount);
  serializeJson(doc, Serial);
  Serial.println();

  sprintf(rtnTopic, "ble/locator/%s/%s", (char *)advertisedDevice.getAddress().toString().c_str(), baseMacChr);

  mqttClient.publish(rtnTopic, 0, 0, rtnMsg);
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.println(advertisedDevice.toString().c_str());
    if (advertisedDevice.haveServiceData())
    {
      std::string serviceData = advertisedDevice.getServiceData();
      if (strstr((char *)advertisedDevice.getServiceDataUUID().toString().c_str(), "fe95") != NULL)
      {
        if (serviceData[11] == 0x0d)
        {
          DeviceCount++;
          BleTempHum(serviceData, advertisedDevice);
        }
        if (serviceData[11] == 0x0a)
        {
          DeviceCount++;
          BleBattery(serviceData, advertisedDevice);
        }
      }
      else if (strstr((char *)advertisedDevice.getServiceDataUUID().toString().c_str(), "fdcd") != NULL && serviceData[8] == 0x01)
      {
        DeviceCount++;
        BleClearGrass(serviceData, advertisedDevice);
      }
    }
    if (advertisedDevice.haveServiceUUID())
    {
      if (advertisedDevice.getServiceUUID().equals(serviceUUIDItag) || advertisedDevice.getServiceUUID().equals(serviceUUIDNut))
      {
        DeviceCount++;
        BlePresenceDetection(advertisedDevice);
      }
    }
  }
};

void onOff(String messageReceived)
{
  if (messageReceived == "led-on")
    digitalWrite(ledPin, LOW);
  if (messageReceived == "led-off")
    digitalWrite(ledPin, HIGH);
  // if (messageReceived == "flash-on") ledFlash = true;
  // if (messageReceived == "flash-off") ledFlash = false;
  if (messageReceived == "ble-on")
  {
    bleOn = true;
    otaUpdate = false;
  }
  if (messageReceived == "ble-off")
    bleOn = false;
  if (messageReceived == "otaUpdate")
  {
    otaUpdate = true;
    bleOn = false;
  }
  // sendMQTT();
}

void connectToWifi()
{
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  WiFi.setHostname(baseMacChr);
}
// BLEScanResults results;

void scanCompleteCB(BLEScanResults results)
{

  int test = results.getCount();
  Serial.printf("BLE scan found : %i devices.\n", test);
  if (test > 0)
  {
    Serial.printf("BLE recieved %i relevant results\n", DeviceCount);
  }
  pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory
  DeviceCount = 0;
  bleReconnectTimer.once(bleStartIn, startBLE);
}

void startBLE()
{
  // bleReconnectTimer.detach();
  Serial.printf("Connecting to BLE for %i seconds \n", scanTime);
  pBLEScan->start(scanTime, scanCompleteCB, false);
}

bool handleWifiDisconnect()
{
  Serial.println("Going through WiFi disconnect");
  if (WiFi.isConnected())
  {
    Serial.println("WiFi appears to be connected. Not retrying.");
    return true;
  }
  if (retryAttempts > 10)
  {
    Serial.println("Too many retries. Restarting");
    ESP.restart();
  }
  else
  {
    Serial.println("wifi will try again");
    retryAttempts++;
    connectToWifi();
    return true;
  }

  if (mqttClient.connected())
  {
    Serial.print("wifi ok, will disconnect MQTT");
    // When disconnected the mqtt event hadler will deal with this
    mqttClient.disconnect();
    return true;
  }
  else
  {
    Serial.println("restarted");
    mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    wifiReconnectTimer.once(2, connectToWifi);
    return true;
  }
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  if (WiFi.isConnected() && !updateInProgress)
  {
    mqttClient.setCredentials(mqttUser, mqttPassword);
    Serial.printf("Host name: %s\n", baseMacChr);
    mqttClient.setClientId(baseMacChr);
    mqttClient.connect();
  }
  else
  {
    Serial.println("Cannot reconnect MQTT - WiFi error");
    handleWifiDisconnect();
  }
}

boolean handleMqttDisconnect()
{
  if (updateInProgress)
  {
    Serial.println("Not retrying MQTT connection - OTA update in progress");
    return true;
  }
  if (retryAttempts > 10)
  {
    Serial.println("Too many retries. Restarting");
    ESP.restart();
  }
  else
  {
    retryAttempts++;
    Serial.println("Retrying to connect to mqtt, stopping BLE");
    bleReconnectTimer.detach();
  }
  if (WiFi.isConnected() && !updateInProgress)
  {
    Serial.println("Starting MQTT reconnect timer");
    mqttReconnectTimer.once(2, connectToMqtt);
  }
  else
  {
    Serial.print("Disconnected from WiFi; starting WiFi reconnect timer \t");
    handleWifiDisconnect();
  }
  return false;
}

// void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
// {
// 	// printf("Publish received. Topic: %s Payload: %.*s Qos: %i\n", topic, payload, properties.qos);
// 	if (String(topic) == "cmnd/esp32/led/status/all" || String(topic) == "cmmd/esp32/led/status/" + baseMacChr)
// 	{
// 		String ipaddress = WiFi.localIP().toString();
// 		char ipchar[ipaddress.length() + 1];
// 		ipaddress.toCharArray(ipchar, ipaddress.length() + 1);

// 		String rtnTopic = String(availabilityTopic) + "esp32/"+ baseMacChr;
// 	}

// 	else if (String(topic) == "cmnd/esp32/led/" + String(baseMacChr))
// 	{
// 		StaticJsonDocument<64> docLed;
// 		DeserializationError error = deserializeJson(docLed, payload);

// 		if (error)
// 		{
// 			Serial.print(F("deserializeJson() failed: "));
// 			Serial.println(error.c_str());
// 		}

// 		// Stores the data to private area
// 		int brightness = docLed["brightness"];
// 		int channel = docLed["channel"];
// 		int transition = docLed["transition"];
// 		boolean newLed = false;
// 		// Serial.printf("transition: %i\n", transition);
// 		if (transition == 0)
// 		{
// 			// Serial.println("Turn lights on");
// 			char arr[64]; // create an object to update

// 			ledcWrite(docLed["channel"], docLed["brightness"]);
// 			int pin = myLeds.getLed(docLed["channel"], arr);
// 			String rtnTopic = String(availabilityTopic) "esp32/led/" + baseMacChr + "/" + channel;
// 			mqttClient.publish((char *)rtnTopic.c_str(), 0, 0, arr);
// 			return;
// 		}
// 		else
// 		{
// 			JsonArray leds;
// 			myLeds.getLeds(leds);
// 			int totalLeds = leds.size(); //Total number of array
// 			if (totalLeds > 0)
// 			{
// 				for (int i = 0; i < totalLeds; i++)
// 				{
// 					JsonObject led = leds.getElement(i);
// 					if (led["channel"] == channel)
// 					{
// 						led["brightness"] = brightness; //update ebdVal brightness
// 						newLed = false;
// 					}
// 					else
// 						newLed = true;
// 				}
// 			}
// 			if (newLed || totalLeds == 0) //new led but timer is still running or no timer
// 			{
// 				JsonObject newLed = leds.createNestedObject();
// 				newLed["channel"] = channel;
// 				newLed["brightness"] = brightness; //update ebdVal brightness
// 				ticker.attach_ms(transition, fader);
// 				serializeJson(newLed, Serial);
// 			}
// 		}
// 	}
// }

void sendMQTT()
{
  char rtnMsg[128];
  char rtnTopic[100];
  StaticJsonDocument<128> doc;
  String ipaddress = WiFi.localIP().toString();
  char ipchar[ipaddress.length() + 1];
  ipaddress.toCharArray(ipchar, ipaddress.length() + 1);
  sprintf(rtnTopic, "thermo/device/%s", baseMacChr);
  doc["ip"] = ipchar;
  doc["ver"] = esp32Ver;
  doc["esp32"] = baseMacChr;
  doc["Ble"] = bleOn;
  doc["Ota"] = otaUpdate;
  serializeJson(doc, rtnMsg, 128);
  // serializeJson(rtnMsg, Serial);
  Serial.println(rtnTopic);
  mqttClient.publish(rtnTopic, 0, 0, rtnMsg);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{

  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  payload: ");
  Serial.println(payload);

  char localTopic[30];
  sprintf(localTopic, "thermo/in/%s", baseMacChr);

  if (String(topic) == "thermo/in/info")
  {
    /* reply with info about device */
  }
  else if (String(topic) == localTopic)
  {
    // reply with info about this one device
  }
  else if (String(topic) == "thermo/in/all" || String(topic) == localTopic)
  {
    // control all devices deal with the payload
    // onOff(payload);
  }
}

// On Initial Connect to broker subscribe to the following
void onMqttConnect(bool sessionPresent)
{
  Serial.print("Connected to MQTT. Session present: ");
  Serial.println(sessionPresent);

  bleReconnectTimer.once(bleStartIn, startBLE);

  Serial.println("Sending update message on initioal connection");
  sendMQTT();

  mqttClient.subscribe("cmnd/esp32/led/status/#", 2);

  String subTopic1 = "cmnd/esp32/led/" + String(baseMacChr);
  mqttClient.subscribe((char *)subTopic1.c_str(), 2);

  String subTopic2 = "cmnd/esp32/fade/" + String(baseMacChr);
  mqttClient.subscribe((char *)subTopic2.c_str(), 2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  Serial.print("No Retries: ");
  Serial.println(retryAttempts);
  handleMqttDisconnect();
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %x\n\r", event);

  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    digitalWrite(LED_BUILTIN, !LED_ON);
    Serial.print("IP address: \t");
    Serial.println(WiFi.localIP());
    localIp = WiFi.localIP().toString().c_str();
    Serial.print("Hostname: \t");
    Serial.println(WiFi.getHostname());
    connectToMqtt();
    retryAttempts = 0;
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    digitalWrite(LED_BUILTIN, LED_ON);
    Serial.println("WiFi lost connection, resetting timer\t");
    handleWifiDisconnect();

    break;
  case SYSTEM_EVENT_WIFI_READY:
    Serial.println("Wifi Ready");
    handleWifiDisconnect();
    break;
  case SYSTEM_EVENT_STA_START:
    Serial.println("STA Start");
    // tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, hostname);
    break;
  case SYSTEM_EVENT_STA_STOP:
    Serial.println("STA Stop");
    handleWifiDisconnect();
    break;
  case SYSTEM_EVENT_STA_LOST_IP:
    Serial.println("STA Lost IP");
    handleWifiDisconnect();
    break;
  default:
    break;
  }
}

void configureBLE()
{
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99); // less or equal setInterval value
}

void configureOTA()
{
  ArduinoOTA
      .onStart([]() {
        Serial.println("OTA Start");
        updateInProgress = true;
        mqttClient.disconnect(true);
        mqttReconnectTimer.detach();
      })
      .onEnd([]() {
        updateInProgress = false;
        digitalWrite(LED_BUILTIN, !LED_ON);
        Serial.println("\n\rEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        byte percent = (progress / (total / 100));
        Serial.print("Progress: ");
        Serial.println(percent);

        digitalWrite(LED_BUILTIN, percent % 2);
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
        ESP.restart();
        Serial.println("End Failed");
      });
  ArduinoOTA.begin();
}

void configureMqtt()
{
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);

  mqttClient.onMessage(onMqttMessage);
  // mqttClient.onPublish(onMqttPublish);

  mqttClient.setServer(mqttHost, mqttPort);
  mqttClient.setWill(availabilityTopic, 0, 1, "DISCONNECTED");
  mqttClient.setKeepAlive(60);
}

void setup()
{
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  configureBLE();
  connectToWifi();

  WiFi.onEvent(WiFiEvent);

  Serial.println("Connected...");

  getLocalMacAddress();

  configureMqtt();

  configureOTA();
}

void loop()
{
  // Set BLE Scanning active

  ArduinoOTA.handle();
  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed = 1;
  TIMERG0.wdt_wprotect = 0;
}
