#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#include "config.h"

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

  publishMqttInteger(device, "avg", "raw", meter->rawAvg);
  publishMqttInteger(device, "avg", unitAvgName, avg);

  publishMqttInteger(device, "total", "raw", meter->rawSum);
  publishMqttInteger(device, "total", unitSumName, sum);

  publishMqttInteger(device, "battery", "level", batteryLevel);
  publishMqttFloat(device, "battery", "mV", batteryVoltage);

  publishMqttInteger(device, "light", "level", lightLevel);

  publishMqttInteger(device, "device", "constant", meter->constant);
  publishMqttInteger(device, "device", "unit", meter->unit);

  char weekDay[4];
  sprintf(weekDay, "%1d", meter->weekDay);
  publishMqttInteger(device, "weekDay", weekDay, meter->weekDayTotal);

  return "done";
}

bool isiNodeDevice(unsigned char *address) {
  if (address[0] == 0x00 && address[1] == 0x0b && address[2] == 0x57) {
    return true;
  }
  if (address[0] == 0xd0 && address[1] == 0xf0 && address[2] == 0x18) {
    return true;
  }
  return false;
}

std::string parseiNodeData(BLEAdvertisedDevice *device) {
  if (!device->haveManufacturerData()) {
    return std::string("no manfacturer data");
  }

  auto deviceAddress = *device->getAddress().getNative();
  if (!isiNodeDevice(deviceAddress)) {
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
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
      auto parsed_data = parseiNodeData(&advertisedDevice);
      if (parsed_data.empty()) {
        parsed_data = "none";
      }
      
      Serial.printf("Advertised Device: %s, RSSI: %d, parsed: %s\n",
        advertisedDevice.toString().c_str(),
        advertisedDevice.getRSSI(),
        parsed_data.c_str());
    }
};
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

bool connect() {
  if (client.connected()) {
    return true;
  }

  Serial.println("Waiting for WiFi...");
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("No WiFi=");
    Serial.println(WiFi.status());
    return false;
  }

  Serial.print("WiFi connnected established: ");
  Serial.println(WiFi.localIP());

  Serial.println("Connecting to MQTT...");

  if (!client.connect(WiFi.macAddress().c_str(), mqtt_user, mqtt_password)) {
    Serial.print("failed, rc=");
    Serial.println(client.state());
    return false;
  }

  Serial.println("MQTT connection established!"); 
  return true;
}

void bleScan() {
#ifdef ENABLE_BLUETOOTH
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false);
  BLEScanResults devices = pBLEScan->start(bluetooth_scan_time);
  Serial.print("BLE scan completed: ");
  Serial.println(devices.getCount());
#endif
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  char hostname[64];
  uint8_t mac[6];
  WiFi.macAddress(mac);
  sprintf(hostname, "esp32-inode-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.printf("Device Hostname = %s\n", hostname);

  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(ota_password);
  ArduinoOTA.begin();

#ifdef ENABLE_BLUETOOTH
  BLEDevice::init("");
#endif
}

void loop() {
  if (!connect()) {
    Serial.println(" try again in 1 seconds");
    delay(1000);
    return;
  }

  ArduinoOTA.handle();
  client.loop();

  bleScan();
}

