#!/bin/python3
import sys, signal, os
import json #for API
import sched, time
from pymavlink import mavutil
import F2414Modem
from optparse import OptionParser
import collections


task = sched.scheduler(time.time, time.sleep)
telemPeriod = 30   # seconds

modem = None
master = None
Locations = collections.deque(maxlen=10)    # circular buffer to limit memory use
AcceptedCommands = collections.deque(maxlen=10)
Waypoints = []    # waypoint[0] = lat, waypoint[1] = lng, waypoint[2] = alt
stat = None


def remoteTelemetry():
    print('remote telemetry')
    body = json.dumps(list(Locations))
    length = len(body)
    req = """POST /spab/data.cgi HTTP/1.1
Host: therevproject.com
Accept: */*
Connection: close
content-type: application/json
Content-Length: """
    req += str(length) + "\n\n"
    req += body + "\r\n\r\n"
    modem.send(req)
    task.enter(telemPeriod, 1, remoteTelemetry, ())


def requestCommands():
    print('request commands')
    """Requests new commands JSON from control server and registers a callback handler"""
    req = "GET http://therevproject.com/spab/command\r\n\r\n"
    modem.send(req)
    task.enter(telemPeriod, 1, requestCommands, ())     # schedule alternating tasks

def missionSender():
    if Waypoints:
        start_waypoint_send(len(Waypoints))
    task.enter(5, 1, missionSender, ())




def HandleCommand(cmdList):
    print('handling commands')
    print(cmdList)
    for elem in cmdList:
        if(elem["taskId"] in AcceptedCommands):
            continue
        print(elem["action"])
        AcceptedCommands.append(elem["taskId"])
        # append new tuple (lat, long, alt)
        Waypoints.append( ( float(elem["latitude"]), float(elem["longitude"]) , 0 ) )
    print(Waypoints)
    #start_waypoint_send(len(Waypoints))
    #append_waypoints(master, len(Waypoints), 0, 15, 0)
    #append_waypoint(1, 0.0, 15.0, 0)


def fake_waypoint():
    waypoint = [-30.1505*10**7, 116.8705*10**7, 0]
    Waypoints.append(waypoint)
    print(Waypoints)
    task.enter(telemPeriod, 1, fake_waypoint, ())
    start_waypoint_send(len(Waypoints))


def HandleTelemAck(json):
    print(json[1]["message"])


def HandleReceipt(sender, earg):
    s = earg.decode("utf-8")
    # deal with http headers
    lines = s.splitlines()
    if(lines[0] == "HTTP/1.1 200 OK"):
        s = lines[10]
    # deal with json
    try:
        data = json.loads(s)
        if(data[0]["type"]=="telemAck"):
            print("received telemAck")
            HandleTelemAck(data[1:])
        elif(data[0]["type"]=="command"):
            print("received command")
            HandleCommand(data[1:])
    except Exception as e:
        print(str(e))


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


#I think we should use this instead of GPS_RAW
def handle_gps_filtered(msg):
    gps_data = (msg.time_boot_ms, float(msg.lat)/10**7, float(msg.lon)/(10**7), msg.alt, msg.relative_alt, msg.vx, msg.vz, msg.hdg)
    Locations.append(dict(zip(('timestamp', 'latitude', 'longitude', 'temperature', 'salinity'), gps_data[0:3]+(0, 0))))
    # print(Locations)


def handle_mission_request(msg):
    print(msg)
    append_waypoint(msg.seq, 0.0, 15.0, 0)

def handle_mission_request_list(msg):
    start_waypoint_send(len(Waypoints))

def handle_mission_ack(msg):
    print(msg)
    if msg.type != "MAV_MISSION_ACCEPTED":
        print("mission upload failed")
    else:
        print("mission upload success")


def start_waypoint_send(waypoint_count):
    global stat
    print("start_waypoint_send")
    print(master.target_system, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER, waypoint_count)
    #time.sleep(1)
    #while not stat:
    #    master.mav.mission_count_send(master.target_system, master.target_component, waypoint_count)
    #    read_loop(master)
    master.mav.mission_count_send(master.target_system, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER, len(Waypoints))


def append_waypoint(seq, hold_time, acceptance_radius, pass_radius):
    print("appending waypoints")
    print(seq)
    while Waypoints:
        waypoint = Waypoints.pop()
        print(waypoint)
        master.mav.mission_item_send(master.target_system,
                                     mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
                                    seq,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
                                    hold_time, acceptance_radius, pass_radius,
                                    0, waypoint[0], waypoint[1], waypoint[2])



'''def append_waypoints(master, waypoint_count, hold_time, acceptance_radius, pass_radius):
    print("appending waypoints")
    master.mav.mission_count_send(master.target_system, master.target_component, waypoint_count)
    print("entering Waypoints")
    while Waypoints:
        print(Waypoints)
        msg = get_req_message(master, "MISSION_REQUEST_INT")
        seq = msg.seq
        waypoint = Waypoints.pop()
        print(waypoint)
        master.mav.mission_item_int_send(master.target_system, master.target_component, seq, "MAV_FRAME_GLOBAL_INT",
                                         "MAV_CMD_NAV_WAYPOINT", 0, 1, hold_time, acceptance_radius, pass_radius,
                                         float('nan'), waypoint[0], waypoint[1], waypoint[2])

    print("left waypoints")
    ack = get_req_message("MISSION_ACK")
    if ack.type == "MAV_MISSION_ACCEPTED":
        return True
    else:
        print("MAV_MISSION_ERROR")
        return False
'''

def read_loop(m):
    global stat
    while True:
        task.run(blocking=False)
        msg = m.recv_match(blocking=False)
        if not msg:
            continue
        msg_type = msg.get_type()
        #print(msg_type)
        if msg_type == "BAD_DATA":
            pass
        #    if mavutil.all_printable(msg.data):
        #        sys.stderr.write("bad data")
        #        sys.stderr.write(msg.data)
        #        sys.stderr.flush()
        elif msg_type == "RC_CHANNELS_RAW":
            handle_rc_raw(msg)
        elif msg_type == "HEARTBEAT":
            handle_heartbeat(msg)
        elif msg_type == "VFR_HUD":
            handle_hud(msg)
        elif msg_type == "ATTITUDE":
            handle_attitude(msg)
        elif msg_type == "GLOBAL_POSITION_INT":
            handle_gps_filtered(msg)
        elif msg_type == "MISSION_REQUEST":
            stat = True
            print("MISSION_REQUEST")
            handle_mission_request(msg)
        elif msg_type == "MISSION_REQUEST_INT":
            stat = True
            print("MISSION_REQUEST")
            handle_mission_request(msg)
        elif msg_type == "MISSION_REQUEST_LIST":
            stat = True
            print("MISSION_REQUEST_LIST")
            handle_mission_request_list(msg)
        elif msg_type == "MISSION_ACK":
            handle_mission_ack(msg)
        time.sleep(0.05)


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

    global stat
    stat = False

    global modem
    modem = F2414Modem.F2414Modem(opts.mport, opts.baudrate)
    modem += HandleReceipt

    global master
    master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)
    master.wait_heartbeat()
    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)
    master.mav.set_mode_send(master.target_system, 216, 216)

    task.enter(telemPeriod, 1, requestCommands, ())
    task.enter(5, 1, missionSender, ())
    #task.enter(telemPeriod, 1, remoteTelemetry, ())
    task.run(False)

    read_loop(master)


if __name__ == '__main__':
    main()
