#!/bin/bash

DIR=$(dirname "$0")

cd $DIR
sudo hcitool lescan --passive --duplicates &
sudo hcidump --raw | ./inode-hcidump.sh
