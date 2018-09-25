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

    def start(self):
        self.task.enter(self.PollingPeriod, 1, self.missionSender, ())
