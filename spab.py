#!/bin/python2
import sys
from pymavlink import mavutil
import F2414Modem as Modem
from optparse import OptionParser

def handle_heartbeat(msg):
    mode = mavutil.mode_string_v10(msg)
    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

def handle_rc_raw(msg):
    channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)

def handle_hud(msg):
    hud_data = (msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb)
    print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
    print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % hud_data

def handle_attitude(msg):
    attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)
    print "Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd"
    print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data

def read_loop(m):
    while(True):
        msg=m.recv_match(blocking=False)
        if not msg:
            continue
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        elif msg_type == "RC_CHANNELS_RAW":
            handle_rc_raw(msg)
        elif msg_type == "HEARTBEAT":
            handle_heartbeat(msg)
        elif msg_type == "VFR_HUD":
            handle_hud(msg)
        elif msg_type == "ATTITUDE":
            handle_attitude(msg)

def main():
    parser = OptionParser("spab.py [options]")
    parser.add_option("--baudrate", dest="baudrate", type='int', help='master port baud rate', default=57600)
    parser.add_option("--device", dest="device", default=None, help="serial device")
    parser.add_option("--rate", dest="rate", default=4, type='int', help='requested stream rate')
    parser.add_option("--source-system", dest="SOURCE_SYSTEM", type='int', default=255, help="MAVLink source system for this GCS")
    parser.add_option("--showmessages", dest="showmessages", action='store_true', help="show incoming messages", default=False)
    (opts, args) = parser.parse_args()
    if opts.device is None:
        print("You must specify a serial device")
        sys.exit(1)
    master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)
    master.wait_heartbeat()

    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)
    read_loop(master)


if __name__ == '__main__':
    main()
