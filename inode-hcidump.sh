#!/bin/bash

CURRENT=()

# > $0
#   04 3E 21 02 01 00 00
#   31 D6 43 18 F0 D0
#   15 # length
#   02 01 06
#   0E FF 90
#   82
#   00 00 E8 03 00 00 E8 43 B3 00 80 02 0A FE A8 

reversemerge() {
  local o=""
  for i; do
    o="$i$o"
  done
  echo $o
}

rawdec() {
  local R=$(reversemerge "$@")
  echo $((0x$R))
}

rawdecneg() {
  local R=$(rawdec "$@")
  if [[ $R -gt 128 ]]; then
    echo "$(($R-256))"
  else
    echo "$R"
  fi
}

batteryLevel() {
  local battery=$(($1/16))
  if [[ $battery -eq 1 ]]; then
    batteryLevel=100
  elif [[ $batteryLevel -gt 11 ]]; then
    batteryLevel=0
  else
    batteryLevel=$((10*battery-10))
  fi
  
  echo "$batteryLevel%"
}

lightLevel() {
  local lightLevel=$((($1%16)*100/15))
  echo "$lightLevel%"
}

send_mqtt() {
  mosquitto_pub -h hass.home -t "$1" -m "${@:2}" &
}

parse_82() {
  local MAC="$1"
  shift 1

  rawAvg=$(rawdec "${@:1:2}")
  shift 2

  rawSum=$(rawdec "${@:1:4}")
  shift 4

  options=$(rawdec "${@:1:2}")
  shift 2
  unit=$((options/0x4000))
  constant=$((options%0x4000))

  case $unit in
    0)
      unitDefault=100
      unitMultiplier=1000
      unitAvgName=W
      unitSumName=Wh
      ;;
    
    1)
      unitDefault=1000
      unitMultiplier=1000
      unitAvgName=dm3
      unitSumName=dm3
      ;;
    
    *)
      unitDefault=1
      unitMultiplier=1
      unitAvgName=cnt
      unitSumName=cnt
      ;;
  esac

  if [[ $constant -eq 0 ]]; then
    constant=$unitDefault
  fi

  avg=$((60*unitMultiplier*rawAvg/constant))
  sum=$((unitMultiplier*rawSum/constant))

  batteryAndLight=$(rawdec "${@:1:1}")
  shift 1

  weekDayData=$(rawdec "${@:1:2}")
  shift 2

  send_mqtt "/inode/$MAC/raw/avg" "$rawAvg"
  send_mqtt "/inode/$MAC/raw/sum" "$rawSum"
  send_mqtt "/inode/$MAC/batteryLevel" "$(batteryLevel "$batteryAndLight")"
  send_mqtt "/inode/$MAC/lightLevel" "$(lightLevel "$batteryAndLight")"
  send_mqtt "/inode/$MAC/total/$unitAvgName" "$avg"
  send_mqtt "/inode/$MAC/current/$unitSumName" "$sum"
  send_mqtt "/inode/$MAC/powerLevel" "$POWER_LEVEL"
  send_mqtt "/inode/$MAC/rssiLevel" "$RSSI"

  echo "$MAC:" \
    "rawAvg=$rawAvg" \
    "rawSum=$rawSum" \
    "avg=$avg$unitAvgName" \
    "sum=$sum$unitSumName" \
    "constant=$constant" \
    "batteryLevel=$(batteryLevel "$batteryAndLight")" \
    "lightLevel=$(lightLevel "$batteryAndLight")" \
    "weekDayData=$batteryAndLight" \
    "powerLevel=$POWER_LEVEL" \
    "rssi=$RSSI"
}

parse_data() {
  local MAC="$1"
  shift 1

  if [[ "$1" != "FF" ]]; then
    return 1
  fi

  local TYPE="$3"
  shift 3

  case "$TYPE" in
    82)
      parse_$TYPE "$MAC" "$@"
      ;;
    
    *)
      echo "Unknown device."
      return 1
      ;;
  esac
}

parse() {
  if [[ "${13}" != "D0" ]]; then
    return 1
  fi
  if [[ "${12}" != "F0" ]]; then
    return 1
  fi

  shift 7
  
  local MAC=$(reversemerge ${@:1:6})
  MAC="${MAC,,}"
  shift 6

  local LEN=$((0x$1))
  shift 1

  if [[ "$#" -lt "$LEN" ]]; then
    return 1
  fi

  local AD_LEN=$((0x$1))
  shift 1
  shift $AD_LEN

  local PACKET_LEN=$((0x$1))
  shift 1
  local PACKET="${@:1:$PACKET_LEN}"
  shift $PACKET_LEN

  local POWER_LEN=$((0x$1))
  shift 1
  local POWER="${@:1:$POWER_LEN}"
  POWER_LEVEL="$(rawdecneg "$2")"
  shift $POWER_LEN

  RSSI="$(rawdecneg "$1")"
  shift 1

  parse_data "$MAC" $PACKET
}

while read -a ITEMS; do
  if [[ "${ITEMS[0]}" == ">" ]]; then
    unset 'ITEMS[0]'
    CURRENT=()
  elif [[ "${ITEMS[0]}" == ">" ]]; then
    continue
  fi

  CURRENT+=( ${ITEMS[@]} )

  parse "${CURRENT[@]}"
done
