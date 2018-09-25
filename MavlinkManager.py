#!/bin/python3
import sched, time
from pymavlink import mavutil

class MavlinkManager:
    def __init__(self, Scheduler, WaypointBuffer, PollingPeriod, Master):
        self.task = Scheduler
        self.Waypoints = WaypointBuffer
        self.PollingPeriod = PollingPeriod
        self.master = Master

    def missionSender(self):
        if self.Waypoints:
            self.start_waypoint_send(len(self.Waypoints))
        self.task.enter(5, 1, self.missionSender, ())

    def start_waypoint_send(self, waypoint_count):
        global stat
        print("start_waypoint_send")
        print(self.master.target_system, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER, waypoint_count)
        #time.sleep(1)
        #while not stat:
        #    self.master.mav.mission_count_send(self.master.target_system, self.master.target_component, waypoint_count)
        #    read_loop(self.master)
        self.master.mav.mission_count_send(self.master.target_system, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER, len(self.Waypoints))

    def getWaypoints(self):
        self.master.mav.mission_request_list_send(self.master.target_system, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
        self.task.enter(20, 1, self.getWaypoints, ())

    def start(self):
        self.task.enter(self.PollingPeriod, 1, self.missionSender, ())
        self.task.enter(20, 1, self.getWaypoints, ())

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
