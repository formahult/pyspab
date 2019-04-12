#!/bin/sh
until (/home/pi/pyspab/spab.py --device /dev/ttyAMA0 --modem /dev/ttyUSB0 >> /tmp/spab.log); do
	echo "Server 'spab.py' crashed with exit code $?. Respawning.."
	sleep 5
done
