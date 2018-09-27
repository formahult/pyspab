#!/bin/python3
import sched
import time
from pymavlink import mavutil


class MavlinkManager:
    def __init__(self, Scheduler, LocationBuffer, WaypointBuffer, PollingPeriod, Master):
        self.task = Scheduler
        self.Waypoints = WaypointBuffer
        self.Locations = LocationBuffer    # circular buffer to limit memory use
        self.PollingPeriod = PollingPeriod
        self.master = Master

    def missionSender(self):
        if self.Waypoints:
            self.start_waypoint_send(len(self.Waypoints))
        self.task.enter(5, 1, self.missionSender, ())

    def start_waypoint_send(self, waypoint_count):
        print("start_waypoint_send")
        print(self.master.target_system, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER, waypoint_count)
        self.master.mav.mission_count_send(self.master.target_system, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER, len(self.Waypoints))

    def getWaypoints(self):
        self.master.mav.mission_request_list_send(self.master.target_system, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
        self.task.enter(20, 1, self.getWaypoints, ())

    def start(self):
        self.task.enter(self.PollingPeriod, 1, self.missionSender, ())
        self.task.enter(20, 1, self.getWaypoints, ())

    def handle_bad_data(self, msg):
        if mavutil.all_printable(msg.data):
            sys.stderr.write(msg.data)
            sys.stderr.flush()

    def handle_heartbeat(self, msg):
        mode = mavutil.mode_string_v10(msg)
        is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

    def handle_rc_raw(self, msg):
        channels = (msg.chan1_raw, msg.chan2_raw,
                    msg.chan3_raw, msg.chan4_raw,
                    msg.chan5_raw, msg.chan6_raw,
                    msg.chan7_raw, msg.chan8_raw)

    def handle_hud(self, msg):
        hud_data = (msg.airspeed, msg.groundspeed,
                    msg.heading, msg.throttle,
                    msg.alt, msg.climb)

    def handle_attitude(self, msg):
        attitude_data = (msg.roll, msg.pitch,
                        msg.yaw, msg.rollspeed,
                        msg.pitchspeed, msg.yawspeed)

    def handle_gps_filtered(self, msg):
        gps_data = (msg.time_boot_ms, float(msg.lat)/10**7,
                    float(msg.lon)/(10**7), msg.alt,
                    msg.relative_alt, msg.vx, msg.vz, msg.hdg)
        self.Locations.append(dict(zip(('timestamp', 'latitude', 'longitude', 'temperature', 'salinity'), gps_data[0:3]+(0, 0))))

    def handle_mission_request(self, msg):
        print(msg)
        append_waypoint(msg.seq, 0.0, 15.0, 0)

    def handle_mission_ack(self, msg):
        global Count, Seq
        print(msg)
        if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("mission upload failed")
        else:
            print("mission upload success")
            Waypoints.clear()
            Count = 0
            Seq = 0

    def handle_mission_count(self, msg):
        global Count, Seq
        Count = msg.count
        Seq = 0
        print(str(Count) + " waypoints")
        self.master.mav.mission_request_send(self.master.target_system,
                                        mavutil.mavlink.MAV_COMP_ID_ALL,
                                        Seq)

    def handle_mission_item(self, msg):
        global Count, Seq
        print("WP " + str(msg.seq) + " " + str(msg.x) + " " + str(msg.y))
        if(Seq != Count):
            self.master.mav.mission_request_send(self.master.target_system,
                                        mavutil.mavlink.MAV_COMP_ID_ALL,
                                        Seq)
            Seq += 1
        else:
            self.master.mav.mission_ack_send(self.master.target_system,
                                        mavutil.mavlink.MAV_COMP_ID_ALL,
                                        mavutil.mavlink.MAV_MISSION_ACCEPTED)

    def append_waypoint(self, seq, hold_time, acceptance_radius, pass_radius):
        print("appending waypoints")
        print(seq)
        waypoint = Waypoints[seq]
        print(waypoint)
        self.master.mav.mission_item_send(self.master.target_system,
                                    mavutil.mavlink.MAV_COMP_ID_ALL,
                                    seq,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1, 1,
                                    1.0, 15.0, 0.0,
                                    0.0, waypoint[0], waypoint[1], waypoint[2])
