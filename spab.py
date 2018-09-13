#!/bin/python3
import sys, signal, os
import json #for API
import sched, time
from pymavlink import mavutil
import F2414Modem
from optparse import OptionParser


task = sched.scheduler(time.time, time.sleep)
loggingPeriod = 30 # seconds
commandPeriod = 30
timeOffset    = 0 # avoid having task overlap in time

modem = None
Locations = {}

def remoteTelemetry():
    task.enter(loggingPeriod, 1, requestCommands, ())
    body = json.dumps(Locations)
    length = len(body)
    req = "POST http://therevproject.com/spab/data\r\nContent-Type:application/json\r\nContent-Length:" + str(length) + "\n\n" + body + "\r\n\r\n"
    print(req)
    modem.send(req)


def requestCommands():
    task.enter(commandPeriod, 1, requestCommands, ())
    req = "GET http://therevproject.com/spab/command\r\n\r\n"
    print(req)
    modem.send(req)

def onModemDataReceived(sender, earg):
    s = earg.decode("utf-8")
    print(s)
    commands = json.loads(s)
    

def handle_heartbeat(msg):
    mode = mavutil.mode_string_v10(msg)
    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

def handle_rc_raw(msg):
    channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)

def handle_hud(msg):
    hud_data = (msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb)
    #print("Aspd\tGspd\tHead\tThro\tAlt\tClimb")
    #print("%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % hud_data)

def handle_attitude(msg):
    attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)
    #print("Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd")
    #print("%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data)

def handle_gps_raw(msg):
    gps_data = (msg.time_usec, float(msg.lat)/(10**7), float(msg.lon)/(10**7), msg.alt, msg.eph, msg.epv, msg.vel, msg.cog, msg.fix_type, msg.satellites_visible)
    Locations.update(zip(('timestamp','latitude','longitude','temperature','salinity'),gps_data[0:3]+(0,0)))
    #print("Time\t\tLat\t\tLon")
    #print("%i\t%f\t%f" % gps_data[0:3])

def read_loop(m):
    while(True):
        task.run(blocking=False)
        msg=m.recv_match(blocking=False)
        if not msg:
            continue
        msg_type = msg.get_type()
        #print(str(msg_type))
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
        elif msg_type == "GPS_RAW_INT":
            handle_gps_raw(msg)

def catch(sig, frame):
    print("\r")
    os._exit(0)

def main():
    parser = OptionParser("spab.py [options]")
    parser.add_option("--baudrate", dest="baudrate", type='int', help='master port baud rate', default=57600)
    parser.add_option("--device", dest="device", default=None, help="serial device")
    parser.add_option("--modem", dest="mport", default=None, help="modem serial port")
    parser.add_option("--rate", dest="rate", default=4, type='int', help='requested stream rate')
    parser.add_option("--source-system", dest="SOURCE_SYSTEM", type='int', default=255, help="MAVLink source system for this GCS")
    parser.add_option("--showmessages", dest="showmessages", action='store_true', help="show incoming messages", default=False)
    (opts, args) = parser.parse_args()
    if opts.device is None or opts.mport is None:
        print("You must specify a serial device")
        sys.exit(1)

    signal.signal(signal.SIGINT, catch)

    global modem
    modem = F2414Modem.F2414Modem(opts.mport, opts.baudrate)
    modem += onModemDataReceived

    master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)
    master.wait_heartbeat()
    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)

    task.enter(loggingPeriod, 1, requestCommands, ())

    read_loop(master)


if __name__ == '__main__':
    main()
