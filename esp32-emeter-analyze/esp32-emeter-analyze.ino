#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 5;

struct iNodeMeter {
  unsigned short rawAvg;
  unsigned int rawSum;
  unsigned short constant : 14;
  unsigned short unit : 2;
  unsigned char batteryLevel : 4;
  unsigned char lightLevel : 4;
  unsigned short weekDayData;
} __attribute__((packed));

std::string parseiNodeMeter(unsigned char *data, size_t size) {
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

  auto avg = 60 * unitMultiplier * meter->rawAvg / constant;
  auto sum = unitMultiplier * meter->rawSum / constant;
 
  char buffer[128];
  sprintf(buffer, "rawAvg=%d rawSum=%d avg=%d%s sum=%d%s constant=%d batteryLevel=%d lightLevel=%d weekDayData=%d",
    meter->rawAvg, meter->rawSum, avg, unitAvgName, sum, unitSumName, constant,
    meter->batteryLevel, meter->lightLevel, meter->weekDayData);

  return buffer;
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

std::string parseiNodeData(BLEAdvertisedDevice device) {
  if (!device.haveManufacturerData()) {
    return std::string("no manfacturer data");
  }

  auto deviceAddress = *device.getAddress().getNative();
  if (!isiNodeDevice(deviceAddress)) {
    return std::string("invalid address");
  }

  auto data = (unsigned char*)device.getManufacturerData().c_str();
  auto size = device.getManufacturerData().size();
  if (size < 3) {
    return std::string("not enough data");
  }

  if (data[0] != 0x90 && data[0] != 0xa0) {
    return std::string("not inode data");
  }

  switch(data[1]) {
    case 0x82:
      return parseiNodeMeter(data + 2, size - 2);

    default:
      return "invalid inode device";
  }
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
      auto parsed_data = parseiNodeData(advertisedDevice);
      if (parsed_data.empty()) {
        parsed_data = "none";
      }
      
      Serial.printf("Advertised Device: %s, RSSI: %d, parsed: %s\n",
        advertisedDevice.toString().c_str(),
        advertisedDevice.getRSSI(),
        parsed_data.c_str());
    }
};


void setup() {
  Serial.begin(115200);
  Serial.println("Scanning...");

  BLEDevice::init("");
}

void loop() {
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false);

  BLEScanResults foundDevices = pBLEScan->start(scanTime);

  Serial.print("Devices found: ");
  Serial.println(foundDevices.getCount());
  Serial.println("Scan done!");

  delay(0);
}

