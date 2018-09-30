#!/usr/bin/sh
until /home/pi/pyspab/spab.py --device /dev/ttyUSB0 --modem /dev/ttyAMA0 > /tmp/spab.log; do
	echo "Server 'spab.py' crashed with exit code $?. Respawning.." >&2
	sleep 5
done
