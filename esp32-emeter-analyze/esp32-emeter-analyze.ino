#include "config.h"

#ifdef ENABLE_OLED
#include <SSD1306.h>
SSD1306 display(oled_address, oled_pin_sda, oled_pin_scl);
#endif

#ifdef ENABLE_BLUETOOTH
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <esp_coexist.h>
#endif

#ifdef ENABLE_LORA
#include <LoRa.h>
#endif

#ifdef ENABLE_WIFI
#include <WiFi.h>

#ifdef ENABLE_WIFI_MQTT
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient mqttClient(mqtt_server, mqtt_port, espClient);
#endif

#ifdef ENABLE_WIFI_OTA
#include <ArduinoOTA.h>
#endif
#endif

#ifdef ENABLE_OLED
enum lineNames {
  DeviceStatus,
  WiFiStatus,
  OTAStatus,
  iNodeStatus,
  iNodeDataStatus,
  lineCount
};

String lines[lineCount];
#endif

void publishMqttString(const char *device, const char *key1, const char *key2, const char *value) {
  char topic[64];

  sprintf(topic, "/inode/%s/%s/%s", device, key1, key2);
  Serial.printf("<< %s = %s\n", topic, value);
#ifdef ENABLE_WIFI_MQTT
  mqttClient.publish(topic, value);
#endif
}

void publishMqttInteger(const char *device, const char *key1, const char *key2, int value) {
  char valueString[20];
  sprintf(valueString, "%d", value);
  publishMqttString(device, key1, key2, valueString);
}

void publishMqttFloat(const char *device, const char *key1, const char *key2, float value) {
  char valueString[20];
  sprintf(valueString, "%.2f", value);
  publishMqttString(device, key1, key2, valueString);
}

String bytesToHex(const unsigned char *data, int size) {
  String output;
  for (int i = 0; i < size; ++i) {
    // TODO: this is slow
    char buffer[3];
    sprintf(buffer, "%02x", data[i] & 0xFF);
    output += buffer;
  }
  return output;
}

String bytesToString(const unsigned char *data, int size) {
  String output;
  output.reserve(size);
  for (int i = 0; i < size; ++i) {
    // TODO: this is slow
    output += (char)data[i];
  }
  return output;
}

struct deviceData {
  String name;
  String address;
  String payload;
  int rssi;
  int txPower;
  int snr;
  long freqError;

  deviceData() {
    rssi = -1;
    txPower = -1;
    snr = -1;
    freqError = -1;
  }

  String rawData() const {
    return bytesToHex((const unsigned char*)payload.c_str(), payload.length());
  }

  String publish() {
    if (payload.length() < 3) {
      return "not enough data";
    }

    auto data = (unsigned char*)payload.c_str();
    if (data[0] != 0x90 && data[0] != 0xa0) {
      return "not inode data";
    }
 
    publishMqttString(address.c_str(), "device", "name", name.c_str());
    if (txPower >= 0)
      publishMqttInteger(address.c_str(), "device", "tx-power", txPower);
    if (rssi >= 0)
      publishMqttInteger(address.c_str(), "device", "rssi", rssi);
    if (snr >= 0)
      publishMqttInteger(address.c_str(), "device", "snr", snr);
    if (freqError >= 0)
      publishMqttInteger(address.c_str(), "device", "freqError", freqError);

    switch(data[1]) {
      case 0x82:
        return publishMeter(data + 2, payload.length() - 2);
  
      default:
        return "invalid inode device";
    }
  }

  String publishMeter(unsigned char *data, size_t size) {
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

    publishMqttInteger(address.c_str(), "device", "constant", meter->constant);
    publishMqttInteger(address.c_str(), "device", "unit", meter->unit);

    publishMqttInteger(address.c_str(), "avg", "raw", meter->rawAvg);
    publishMqttInteger(address.c_str(), "avg", unitAvgName, avg);

    publishMqttInteger(address.c_str(), "total", "raw", meter->rawSum);
    publishMqttInteger(address.c_str(), "total", unitSumName, sum);

    publishMqttInteger(address.c_str(), "battery", "level", batteryLevel);
    publishMqttFloat(address.c_str(), "battery", "mV", batteryVoltage);

    publishMqttInteger(address.c_str(), "light", "level", lightLevel);

    char weekDay[4];
    sprintf(weekDay, "%1d", meter->weekDay);
    publishMqttInteger(address.c_str(), "weekDay", weekDay, meter->weekDayTotal);

#ifdef ENABLE_OLED
    lines[iNodeStatus] = String("Emeter ") + address + " (" + rssi + "dBm)";
    lines[iNodeDataStatus] = String("Current: ") + avg + "W; Total: " + sum + "kW/h";
#endif

    return "done";
  }
};

#ifdef ENABLE_BLUETOOTH
class BluetoothScanner : public BLEAdvertisedDeviceCallbacks {
public:
  BluetoothScanner() {
    enabled = true;
  }

  static void scannerTask(void *data) {
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

    esp_err_t err = esp_coex_preference_set(ESP_COEX_PREFER_BT);
    Serial.printf("esp_coex_preference_set: %d\n", err);

    while(1) {
      if (scanner->enabled) {
        pBLEScan->start(bluetooth_scan_time);
      } else {
        delay(1000);
      }
    }
  }

  void resume() {
    enabled = true;
  }

  void pause() {
    enabled = false;
    BLEDevice::getScan()->stop();
  }

  void process() {
    int i;

    mutex.take();
    for (i = 0; i < advertisements.size(); ++i) {
      deviceData advertisement = advertisements[i];
      mutex.give();
      processAdvertisement(&advertisement);
      mutex.take();
    }
    advertisements.clear();
    mutex.give();
  }

private:
  String toString(const std::string &data) {
    String output;
    output.reserve(data.size());
    for (int i = 0; i < data.length(); ++i) {
      output += data[i];
    }
    return output;
  }

  bool isiNode(const char *address) const {
    if (strstr(address, "000b57") == address)
      return true;
    if (strstr(address, "d0f018") == address)
      return true;
    return false;
  }

  void onResult(BLEAdvertisedDevice advertisedDevice) {
    deviceData data;
    data.address = bytesToHex(*advertisedDevice.getAddress().getNative(), 6);
    data.name = toString(advertisedDevice.getName());
    data.payload = toString(advertisedDevice.getManufacturerData());
    data.rssi = advertisedDevice.getRSSI();
    data.txPower = advertisedDevice.getTXPower();

    if (!isiNode(data.address.c_str())) {
      Serial.printf("Advertised Device: %s, RSSI: %d => is not iNode Device\n",
        advertisedDevice.toString().c_str(),
        advertisedDevice.getRSSI());
      return;
    }

    mutex.take();
    if (advertisements.size() < 20) {
      advertisements.push_back(data);
    } else {
      Serial.println("Too many advertisements received. Dropping\n");
    }
    mutex.give();
  }

  void processAdvertisement(deviceData *data) {
    data->publish();
  }

  std::vector<deviceData> advertisements;
  FreeRTOS::Semaphore mutex;
  bool enabled;
};

BluetoothScanner bluetoothScanner;
#endif

#ifdef ENABLE_WIFI_OTA
static void otaStart() {
  String type;
  if (ArduinoOTA.getCommand() == U_FLASH)
    type = "sketch";
  else // U_SPIFFS
    type = "filesystem";

  // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  Serial.println("OTA update: " + type);
#ifdef ENABLE_BLUETOOTH
  bluetoothScanner.pause();
#endif
#ifdef ENABLE_OLED
  lines[OTAStatus] = String("OTA update: ") + type + "...";
#endif
}

static void otaEnd() {
  Serial.println("\nEnd");
#ifdef ENABLE_BLUETOOTH
  bluetoothScanner.resume();
#endif
#ifdef ENABLE_OLED
  lines[OTAStatus] = "";
#endif
}

static void otaProgress(unsigned int progress, unsigned int total) {
  Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
#ifdef ENABLE_OLED
  lines[OTAStatus] = String("OTA Progress: ") + (progress / (total / 100)) + "%";
#endif
}

static void otaError(ota_error_t error) {
  Serial.printf("OTA Error[%u]: ", error);
  if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
  else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
  else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
  else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
  else if (error == OTA_END_ERROR) Serial.println("End Failed");
#ifdef ENABLE_OLED
  lines[OTAStatus] = String("OTA Error: ") + error;
#endif
}
#endif

#ifdef ENABLE_LORA
String receiveLoRa(const String &packet) {
  if (packet.length() < 7) {
    return "too small packet";
  }

  unsigned char *dataStart = (unsigned char *)packet.c_str();
  unsigned char *dataEnd = dataStart + packet.length();

  unsigned char payloadSize = *dataStart;
  if (packet.length() != payloadSize + 2) {
    return "invalid size";
  }

  deviceData data;
  data.address = bytesToHex(dataStart + 2, 3);
  data.payload = bytesToString(dataStart + 7, packet.length() - 7);
  data.rssi = LoRa.packetRssi();
  data.snr = LoRa.packetSnr();
  data.freqError = LoRa.packetFrequencyError();
  return data.publish();
}

void parseLoRa(int packetSize) {
  String output;
  output.reserve(packetSize);
  while (LoRa.available()) {
    output += (char)LoRa.read();
  }

  Serial.println();
  Serial.println("LoRa Message: " + bytesToHex((unsigned char*)output.c_str(), output.length()));
  Serial.println("LoRa Packet Size: " + String(packetSize));
  Serial.println("LoRa RSSI: " + String(LoRa.packetRssi()));
  Serial.println("LoRa Snr: " + String(LoRa.packetSnr()));
  Serial.println("LoRa Freq: " + String(LoRa.packetFrequencyError()));
  String result = receiveLoRa(output);
  Serial.println("LoRa Result: " + result);
  Serial.println();
}

void processLoRa() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    parseLoRa(packetSize);
  }
}
#endif

void setup() {
  Serial.begin(115200);
  Serial.printf("Compiled: %s %s\n", __DATE__, __TIME__);

#ifdef ENABLE_OLED
  Serial.println("Starting OLED...");

  if (oled_pin_reset >= 0) {
    pinMode(oled_pin_reset, OUTPUT);
    digitalWrite(oled_pin_reset, LOW);
    delay(50); 
    digitalWrite(oled_pin_reset, HIGH);
  }

  display.init();
  display.flipScreenVertically();
#endif

#ifdef ENABLE_LORA
  Serial.println("Starting LoRa...");
  SPI.begin(lora_pin_sck, lora_pin_miso, lora_pin_mosi, lora_pin_ss);
  LoRa.setPins(lora_pin_ss, lora_pin_reset, lora_pin_di0);

  if (!LoRa.begin(lora_band)) {
    Serial.println("Failed to start LoRa!\n");
    ESP.restart();
    return;
  }

  LoRa.setSpreadingFactor(lora_sf);
  LoRa.setSignalBandwidth(lora_bw);
  LoRa.setCodingRate4(lora_cr);
  LoRa.enableCrc();

  Serial.println("LoRa started!");
#endif

#ifdef ENABLE_WIFI
  WiFi.begin(ssid, password);

  char hostname[64];
  uint8_t mac[6];
  WiFi.macAddress(mac);
  sprintf(hostname, "esp32-inode-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("Hostname = %s\n", hostname);
  lines[DeviceStatus] = String("Device: ") + hostname;

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

#ifdef ENABLE_WIFI_OTA
  ArduinoOTA.onStart(otaStart);
  ArduinoOTA.onEnd(otaEnd);
  ArduinoOTA.onProgress(otaProgress);
  ArduinoOTA.onError(otaError);
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(ota_password);
  ArduinoOTA.begin();
#endif

#ifdef ENABLE_WIFI_MQTT
  i = 0;
  Serial.println("Connecting to MQTT... ");
  while (!mqttClient.connect(WiFi.macAddress().c_str(), mqtt_user, mqtt_password)) {
    if (i++ > 30) {
      Serial.println("Failed to connect to MQTT. Reboot\n");
      ESP.restart();
      return;
    }

    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
    delay(1000);
  }

  Serial.println("MQTT connection established!"); 
#endif
#endif

#ifdef ENABLE_BLUETOOTH
  FreeRTOS::startTask(BluetoothScanner::scannerTask, "bluetoothScanner", &bluetoothScanner);
#endif
}

#ifdef ENABLE_OLED
void updateDisplay() {
  lines[WiFiStatus] = "";

#ifdef ENABLE_BLUETOOTH
  lines[WiFiStatus] += "BT ";
#endif

#ifdef ENABLE_LORA
  lines[WiFiStatus] += "LoRa ";
#endif

#ifdef ENABLE_WIFI
  lines[WiFiStatus] += String("WiFi(") + WiFi.status() + ") ";
#endif

#ifdef ENABLE_WIFI_MQTT
  lines[WiFiStatus] += String("MQTT(") + mqttClient.connected() + ") ";
#endif

#ifdef ENABLE_WIFI_OTA
  lines[WiFiStatus] += String("OTA ");
#endif
}

void refreshDisplay() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  for(int i = 0, line = 0; i < sizeof(lines)/sizeof(lines[0]); ++i) {
    if (lines[i].length() == 0) {
      continue;
    }
    display.drawStringMaxWidth(0, 13 * line++, 128, lines[i]);
  }
  display.display();
}
#endif

void loop() {
#ifdef ENABLE_WIFI
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No WiFi connection. Reboot");
    ESP.restart();
    return;
  }

#ifdef ENABLE_WIFI_OTA
  ArduinoOTA.handle();
#endif

#ifdef ENABLE_WIFI_MQTT
  if (!mqttClient.connected()) {
    Serial.println("No MQTT connection. Reboot");
    ESP.restart();
    return;
  }

  mqttClient.loop();
#endif
#endif

#ifdef ENABLE_BLUETOOTH
  bluetoothScanner.process();
#endif

#ifdef ENABLE_LORA
  processLoRa();
#endif

#ifdef ENABLE_OLED
  updateDisplay();
  refreshDisplay();
#endif

  delay(5);
}
