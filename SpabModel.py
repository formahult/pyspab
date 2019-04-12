#!/usr/bin/python3
class SpabModel:

    def __init__(self):
        self.Waypoints = []
        self.pendingWaypoints = []
        self.Home = ()
        self.LastLocation = {}
        self.mode = None
        self.is_armed = None
        self.is_enabled = None
        self.channels = None
        self.attitude_data = None
