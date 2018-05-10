#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "config.h"

WiFiClient espClient;
PubSubClient client(mqtt_server, mqtt_port, espClient);

void publishMqttString(BLEAdvertisedDevice *device, const char *key1, const char *key2, const char *value) {
  char topic[64];
  uint8_t *nativeAddress = *device->getAddress().getNative();

  sprintf(topic, "/inode/tests/%02x%02x%02x%02x%02x%02x/%s/%s",
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
    unsigned char batteryLevel : 4;
    unsigned char lightLevel : 4;
    unsigned short weekDay : 4;
    unsigned short weekDayTotal : 12;
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
  auto batteryVoltage = (batteryLevel - 10) * 1.2 / 100 + 1.8;
  auto lightLevel = meter->lightLevel * 100 / 15;

  char weekDay[4];
  sprintf(weekDay, "%1d", meter->weekDay);

  publishMqttInteger(device, "raw", "avg", meter->rawAvg);
  publishMqttInteger(device, "raw", "sum", meter->rawSum);
  publishMqttInteger(device, "raw", "constant", meter->constant);
  publishMqttInteger(device, "total", unitSumName, sum);
  publishMqttInteger(device, "current", unitAvgName, avg);
  publishMqttInteger(device, "battery", "level", batteryLevel);
  publishMqttFloat(device, "battery", "voltage", batteryVoltage);
  publishMqttInteger(device, "light", "level", lightLevel);
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

  publishMqttInteger(device, "tx", "power", device->getTXPower());
  publishMqttFloat(device, "rssi", "level", device->getRSSI());

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

  Serial.print("Attempting WiFi and MQTT connection...");
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("No WiFi=");
    Serial.print(WiFi.status());
    return false;
  }

  if (!client.connect(WiFi.macAddress().c_str(), mqtt_user, mqtt_password)) {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    return false;
  }

  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());
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
  Serial.printf("ESP32 WiFi Mac Address = %s\n", WiFi.macAddress().c_str());
  
  WiFi.begin(ssid, password);

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

  client.loop();
  bleScan();
}

