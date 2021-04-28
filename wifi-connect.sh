#!/bin/sh

. ./config.sh

sudo nmcli d wifi connect $SSID 
# password <password>

