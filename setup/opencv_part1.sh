#!/bin/sh

# Install OpenCV 4.5 on Jetson Nano
# Source: https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html

# Part 1 create swapfile

# a fresh start, so check for updates
sudo apt-get update
sudo apt-get install nano
# install dphys-swapfile
sudo apt-get install dphys-swapfile
# give the required memory size
sudo vi /etc/dphys-swapfile
# reboot afterwards
sudo reboot


