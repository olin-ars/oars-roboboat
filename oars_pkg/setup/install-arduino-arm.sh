#!/bin/sh

cd /tmp

if [ ! -f arduino-1.8.1-linuxarm.tar.xz ] ; then
	echo "Downloading files"
	wget -v https://downloads.arduino.cc/arduino-1.8.1-linuxarm.tar.xz 
fi

echo "Extracting zip"

tar -xJf arduino-1.8.1-linuxarm.tar.xz

echo "Moving files to /usr/local"

sudo mv arduino-1.8.1 /usr/local


