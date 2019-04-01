#!/bin/sh
echo "Shell script starting"
until (/home/pi/pyspar/spab.py --device /dev/ttyUSB4 --modem /dev/ttyUSB3 --baudrate 115200 >> /tmp/spab.log); do
	echo "Server 'spab.py' crashed with exit code $?. Respawning.."
	sleep 5
done
