# iNode helper scripts and tools

This repository contains a bunch of scripts (shell) and applications (esp32) that are useful for reading the data for iNode Energy Meters / and Magneto.

## `inode-bt.sh`

This is a simple script that should be run as root that decodes raw frames received by USB BT adapter. This was tested on CSR4.0 DONGLE.

## `inode-hcidump.sh`

This is a simple script that can decode raw frames (manfacturer advertisement data) if received via different channel:

```bash
$ ./inode-hcidump.sh 90821300b0960700dc05c00080
user: rawAvg=19 rawSum=497328 avg=760W sum=331552Wh constant=1500 batteryLevel=110% lightLevel=0% weekDayData=192 powerLevel= rssi=
```

```bash
$ ./inode-hcidump.sh 90 82 36 00 b1 65 07 00 dc 05 c0 00 80
user: rawAvg=54 rawSum=484785 avg=2160W sum=323190Wh constant=1500 batteryLevel=110% lightLevel=0% weekDayData=192 powerLevel= rssi=
```

## `inode-server`

Simple iNode Server Hub written in Go to be used with iNode USB adapters. _Currently supports only iNode USB LoRa adapter._

### Binaries

Download and run one of the binaries from https://github.com/ayufan/inode-tests/releases/latest.

### Manual compliation

Install Go language runtime and run from the terminal:

```bash
go get -v github.com/ayufan/inode-tests/inode-server
inode-server
```

## `esp32-emeter-analyze`

This is a simple ESP32 app (using Arduino Core) that can be used to receive emeter BT data. Currently it is only decoding, but is very simple to extend: for example to add HTTP POST or MQTT.

## License

MIT

## Author

Kamil Trzciński, 2018
