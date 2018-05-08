#!/bin/bash

DIR=$(dirname "$0")
DEVICE=${1:-hci0}

if [ $(id -u) -ne 0 ]; then
  echo "$0: run as root"
  exit 1
fi

cd $DIR
hcitool -i "$DEVICE" lescan --passive --duplicates &
hcidump -i "$DEVICE" --raw | ./inode-hcidump.sh
