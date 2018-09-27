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
        print("start_waypoint_send")
        print(self.master.target_system, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER, waypoint_count)
        self.master.mav.mission_count_send(self.master.target_system, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER, len(self.Waypoints))

    def getWaypoints(self):
        self.master.mav.mission_request_list_send(self.master.target_system, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
        self.task.enter(20, 1, self.getWaypoints, ())

    def start(self):
        self.task.enter(self.PollingPeriod, 1, self.missionSender, ())
        self.task.enter(20, 1, self.getWaypoints, ())


