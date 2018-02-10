#!/bin/bash

DIR=$(dirname "$0")

if [ $(id -u) -ne 0 ]; then
  echo "$0: run as root"
  exit 1
fi

cd $DIR
hcitool lescan --passive --duplicates &
hcidump --raw | ./inode-hcidump.sh
