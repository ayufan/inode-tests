#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_coexist.h>

#include "config.h"

#ifdef ENABLE_OLED
#include <SSD1306.h>

//OLED pins to ESP32 GPIOs via this connecthin:
//OLED_SDA — GPIO4
//OLED_SCL — GPIO15
//OLED_RST — GPIO16
SSD1306 display(0x3c, 4, 15);
#endif

#ifdef ENABLE_BLUETOOTH
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#endif

#ifdef ENABLE_OTA
#include <ArduinoOTA.h>
#endif

#ifdef ENABLE_LORA
#include <LoRa.h>

// WIFI_LoRa_32 ports
// GPIO5 — SX1278’s SCK
// GPIO19 — SX1278’s MISO
// GPIO27 — SX1278’s MOSI
// GPIO18 — SX1278’s CS
// GPIO14 — SX1278’s RESET
// GPIO26 — SX1278’s IRQ(Interrupt Request)
#define SS 18
#define RST 14
#define DI0 26
#define BAND 886E6
#endif

WiFiClient espClient;
PubSubClient client(mqtt_server, mqtt_port, espClient);

void publishMqttString(BLEAdvertisedDevice *device, const char *key1, const char *key2, const char *value) {
  char topic[64];
  uint8_t *nativeAddress = *device->getAddress().getNative();

  sprintf(topic, "/inode/%02x%02x%02x%02x%02x%02x/%s/%s",
    nativeAddress[0], nativeAddress[1], nativeAddress[2], nativeAddress[3], nativeAddress[4], nativeAddress[5],
    key1, key2);
  Serial.printf("<< %s = %s\n", topic, value);
  client.publish(topic, value);
}

void publishMqttInteger(BLEAdvertisedDevice *device, const char *key1, const char *key2, int value) {
  char valueString[20];
  sprintf(valueString, "%d", value);
  publishMqttString(device, key1, key2, valueString);
}

void publishMqttFloat(BLEAdvertisedDevice *device, const char *key1, const char *key2, float value) {
  char valueString[20];
  sprintf(valueString, "%.2f", value);
  publishMqttString(device, key1, key2, valueString);
}

std::string parseiNodeMeter(BLEAdvertisedDevice *device, unsigned char *data, size_t size) {
  struct iNodeMeter {
    unsigned short rawAvg;
    unsigned int rawSum;
    unsigned short constant : 14;
    unsigned short unit : 2;
    unsigned char lightLevel : 4;
    unsigned char batteryLevel : 4;
    unsigned short weekDayTotal : 12;
    unsigned short weekDay : 4;
  } __attribute__((packed));

  iNodeMeter *meter = (iNodeMeter*)data;
  if (size < sizeof(iNodeMeter)) {
    return "invalid emeter data";
  }

  auto unitDefault = 1;
  auto unitMultiplier = 1;
  auto unitAvgName = "cnt";
  auto unitSumName = "cnt";

  switch (meter->unit) {
    case 0:
      unitDefault = 100;
      unitMultiplier = 1000;
      unitAvgName = "W";
      unitSumName = "Wh";
      break;
      
    case 1:
      unitDefault = 1000;
      unitMultiplier = 1000;
      unitAvgName = "dm3";
      unitSumName = "dm3";
      break;
  }

  auto constant = meter->constant;
  if (constant <= 0) {
    constant = unitDefault;
  }

  auto avg = 60 * unitMultiplier * (unsigned int)meter->rawAvg / constant;
  auto sum = unitMultiplier * (unsigned int)meter->rawSum / constant;
  auto batteryLevel = 10 * ((meter->batteryLevel < 11 ? meter->batteryLevel : 11) - 1);
  auto batteryVoltage = batteryLevel * 1200 / 100 + 1800;
  auto lightLevel = meter->lightLevel * 100 / 15;

  publishMqttInteger(device, "device", "constant", meter->constant);
  publishMqttInteger(device, "device", "unit", meter->unit);

  publishMqttInteger(device, "avg", "raw", meter->rawAvg);
  publishMqttInteger(device, "avg", unitAvgName, avg);

  publishMqttInteger(device, "total", "raw", meter->rawSum);
  publishMqttInteger(device, "total", unitSumName, sum);

  publishMqttInteger(device, "battery", "level", batteryLevel);
  publishMqttFloat(device, "battery", "mV", batteryVoltage);

  publishMqttInteger(device, "light", "level", lightLevel);

  char weekDay[4];
  sprintf(weekDay, "%1d", meter->weekDay);
  publishMqttInteger(device, "weekDay", weekDay, meter->weekDayTotal);

  return "done";
}

bool isiNodeDevice(BLEAdvertisedDevice *device) {
  if (!device->haveManufacturerData()) {
    return false;
  }
  
  auto address = *device->getAddress().getNative();
  if (address[0] == 0x00 && address[1] == 0x0b && address[2] == 0x57) {
    return true;
  }
  if (address[0] == 0xd0 && address[1] == 0xf0 && address[2] == 0x18) {
    return true;
  }
  return false;
}

std::string parseiNodeData(BLEAdvertisedDevice *device) {
  if (!isiNodeDevice(device)) {
    return std::string("invalid address");
  }

  auto data = (unsigned char*)device->getManufacturerData().c_str();
  auto size = device->getManufacturerData().size();
  if (size < 3) {
    return std::string("not enough data");
  }

  if (data[0] != 0x90 && data[0] != 0xa0) {
    return std::string("not inode data");
  }

  publishMqttString(device, "device", "name", device->getName().c_str());
  publishMqttInteger(device, "device", "tx-power", device->getTXPower());
  publishMqttInteger(device, "device", "rssi", device->getRSSI());

  switch(data[1]) {
    case 0x82:
      return parseiNodeMeter(device, data + 2, size - 2);

    default:
      return "invalid inode device";
  }
}

#ifdef ENABLE_BLUETOOTH
class BluetoothScanner : public BLEAdvertisedDeviceCallbacks
{
public:
  BluetoothScanner()
  {
    enabled = true;
  }

  static void scannerTask(void *data)
  {
    BluetoothScanner *scanner = (BluetoothScanner*)data;

    // This method works, but it is not the best
    // It leads to having high memory pressure
    // as BLEScan, performs O(n) search on array and adds data to an array
    // we also add all received advertisements to process them in main loop()
    // it is possible, that if a lot of advertisements are received
    // the device will run out of the memory

    BLEDevice::init("");
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(scanner, true);
    pBLEScan->setActiveScan(false);
    pBLEScan->setInterval(100/0.625);
    pBLEScan->setWindow(90/0.625);

    while(1) {
      if (scanner->enabled) {
        pBLEScan->start(bluetooth_scan_time);
      } else {
        delay(1000);
      }
    }
  }

  void resume()
  {
    enabled = true;
  }

  void pause()
  {
    enabled = false;
    BLEDevice::getScan()->stop();
  }

  void process()
  {
    int i;

    mutex.take();
    for (i = 0; i < advertisements.size(); ++i) {
      BLEAdvertisedDevice advertisedDevice = advertisements[i];
      mutex.give();
      processAdvertisement(&advertisedDevice);
      mutex.take();
    }
    advertisements.clear();
    mutex.give();
  }

private:
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    if (!isiNodeDevice(&advertisedDevice)) {
      Serial.printf("Advertised Device: %s, RSSI: %d => is not iNode Device\n",
        advertisedDevice.toString().c_str(),
        advertisedDevice.getRSSI());
      return;
    }
    
    mutex.take();
    if (advertisements.size() < 20) {
      advertisements.push_back(advertisedDevice);
    } else {
      Serial.println("Too many advertisements received. Dropping\n");
    }
    mutex.give();
  }

  void processAdvertisement(BLEAdvertisedDevice *advertisedDevice)
  {
    auto parsed_data = parseiNodeData(advertisedDevice);
    if (parsed_data.empty()) {
      parsed_data = "none";
    }

    Serial.printf("Advertised Device: %s, RSSI: %d, parsed: %s\n",
      advertisedDevice->toString().c_str(),
      advertisedDevice->getRSSI(),
      parsed_data.c_str());
  }

  std::vector<BLEAdvertisedDevice> advertisements;
  FreeRTOS::Semaphore mutex;
  bool enabled;
};

BluetoothScanner bluetoothScanner;
#endif

#ifdef ENABLE_LORA
void processLoRa() {
  while (LoRa.available()) {
    char receivedText = (char)LoRa.read();
    Serial.print(receivedText);
  }
}
#endif

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  int i = 0;

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(++i); Serial.print(' ');
  }

  Serial.println('\n');
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

#ifdef ENABLE_OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5, 5, "LoRa Sender");
  display.display();
#endif

#ifdef ENABLE_LORA
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS, RST, DI0);
#endif

  char hostname[64];
  uint8_t mac[6];
  WiFi.macAddress(mac);
  sprintf(hostname, "esp32-inode-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.printf("Compiled: %s %s\n", __DATE__, __TIME__);
  Serial.printf("Device Hostname = %s\n", hostname);

  esp_err_t err = esp_coex_preference_set(ESP_COEX_PREFER_BT);
  Serial.printf("esp_coex_preference_set: %d\n", err);

#ifdef ENABLE_OTA
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
#ifdef ENABLE_BLUETOOTH
      bluetoothScanner.pause();
#endif
    })
    .onEnd([]() {
      Serial.println("\nEnd");
#ifdef ENABLE_BLUETOOTH
      bluetoothScanner.resume();
#endif
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });


  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(ota_password);
  ArduinoOTA.begin();
#endif

  int i = 0;
  Serial.println("Waiting for WiFi... ");
  while (WiFi.status() != WL_CONNECTED) {
    if (i++ > 30) {
      Serial.println("Failed to connect to WiFi. Reboot\n");
      ESP.restart();
      return;
    }
    Serial.print("No WiFi=");
    Serial.println(WiFi.status());
    delay(1000);
  }

  Serial.print("connnected established: ");
  Serial.println(WiFi.localIP());

  i = 0;
  Serial.println("Connecting to MQTT... ");
  while (!client.connect(WiFi.macAddress().c_str(), mqtt_user, mqtt_password)) {
    if (i++ > 30) {
      Serial.println("Failed to connect to MQTT. Reboot\n");
      ESP.restart();
      return;
    }

    Serial.print("failed, rc=");
    Serial.println(client.state());
    delay(1000);
  }

  Serial.println("MQTT connection established!"); 

#ifdef ENABLE_BLUETOOTH
  FreeRTOS::startTask(BluetoothScanner::scannerTask, "bluetoothScanner", &bluetoothScanner);
#endif

#ifdef ENABLE_LORA
  if (!LoRa.begin(BAND)) {
    Serial.println("Failed to start LoRa\n");
    ESP.restart();
    return;
  }
#endif
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No WiFi connection. Reboot");
    ESP.restart();
    return;
  }

  if (!client.connected()) {
    Serial.println("No MQTT connection. Reboot");
    ESP.restart();
    return;
  }

  client.loop();

#ifdef ENABLE_OTA
  ArduinoOTA.handle();
#endif

#ifdef ENABLE_BLUETOOTH
  bluetoothScanner.process();
#endif

#ifdef ENABLE_LORA
  processLoRa();
#endif

  delay(5);
}
