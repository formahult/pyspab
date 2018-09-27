#!/bin/python3
import sys, signal, os
import sched, time
from pymavlink import mavutil
import F2414Modem
from optparse import OptionParser
import collections
import TelemManager
import MavlinkManager


task = sched.scheduler(time.time, time.sleep)
telemPeriod = 30   # seconds

modem = None
master = None

Locations = collections.deque(maxlen=10)    # circular buffer to limit memory use
Waypoints = []
Delegates = {}
Seq = 0
Count = 0

def handle_bad_data(msg):
    if mavutil.all_printable(msg.data):
        sys.stderr.write(msg.data)
        sys.stderr.flush()

def handle_heartbeat(msg):
    mode = mavutil.mode_string_v10(msg)
    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED


def handle_rc_raw(msg):
    channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)


def handle_hud(msg):
    hud_data = (msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb)


def handle_attitude(msg):
    attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)


def handle_gps_filtered(msg):
    gps_data = (msg.time_boot_ms, float(msg.lat)/10**7, float(msg.lon)/(10**7), msg.alt, msg.relative_alt, msg.vx, msg.vz, msg.hdg)
    Locations.append(dict(zip(('timestamp', 'latitude', 'longitude', 'temperature', 'salinity'), gps_data[0:3]+(0, 0))))


def handle_mission_request(msg):
    print(msg)
    append_waypoint(msg.seq, 0.0, 15.0, 0)


def handle_mission_ack(msg):
    global Count, Seq
    print(msg)
    if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("mission upload failed")
    else:
        print("mission upload success")
        Waypoints.clear()
        Count = 0
        Seq = 0


def handle_mission_count(msg):
    global Count, Seq
    Count = msg.count
    Seq = 0
    print(str(Count) + " waypoints")
    master.mav.mission_request_send(master.target_system,
                                    mavutil.mavlink.MAV_COMP_ID_ALL,
                                    Seq)


def handle_mission_item(msg):
    global Count, Seq
    print("WP " + str(msg.seq) + " " + str(msg.x) + " " + str(msg.y))
    if(Seq != Count):
        master.mav.mission_request_send(master.target_system,
                                    mavutil.mavlink.MAV_COMP_ID_ALL,
                                    Seq)
        Seq += 1
    else:
        master.mav.mission_ack_send(master.target_system,
                                    mavutil.mavlink.MAV_COMP_ID_ALL,
                                    mavutil.mavlink.MAV_MISSION_ACCEPTED)


def append_waypoint(seq, hold_time, acceptance_radius, pass_radius):
    print("appending waypoints")
    print(seq)
    waypoint = Waypoints[seq]
    print(waypoint)
    master.mav.mission_item_send(master.target_system,
                                mavutil.mavlink.MAV_COMP_ID_ALL,
                                seq,
                                mavutil.mavlink.MAV_FRAME_GLOBAL,
                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1, 1,
                                1.0, 15.0, 0.0,
                                0.0, waypoint[0], waypoint[1], waypoint[2])


def read_loop(m):
    global stat
    while True:
        task.run(blocking=False)
        msg = m.recv_match(blocking=False)
        if not msg:
            continue
        msg_type = msg.get_type()
        try:
            Delegates[msg_type](msg)
        except KeyError:
            pass
        #time.sleep(0.05)


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
        print("You must specify a mavlink device and a modem device")
        sys.exit(1)

    signal.signal(signal.SIGINT, catch)

    # init objects
    global modem
    global master
    master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)
    modem = F2414Modem.F2414Modem(opts.mport, opts.baudrate)
    telemManager = TelemManager.TelemManager(task, Locations, Waypoints, modem, telemPeriod)
    mavlinkManager = MavlinkManager.MavlinkManager(task, Waypoints, telemPeriod, master)

    #init delegates
    global Delegates
    Delegates = {
        "HEARTBEAT":handle_heartbeat,
        "VFR_HUD":handle_hud,
        "ATTITUDE":handle_attitude,
        "GLOBAL_POSITION_INT":handle_gps_filtered,
        "MISSION_REQUEST":handle_mission_request,
        "MISSION_ACK":handle_mission_ack,
        "RC_CHANNELS_RAW":handle_rc_raw,
        "BAD_DATA":handle_bad_data,
        "MISSION_COUNT":handle_mission_count,
        "MISSION_ITEM":handle_mission_item
    }

    # set to run
    master.wait_heartbeat()
    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)
    master.mav.set_mode_send(master.target_system, 216, 216)
    telemManager.start()
    mavlinkManager.start()
    task.run(False)
    read_loop(master)


if __name__ == '__main__':
    main()
