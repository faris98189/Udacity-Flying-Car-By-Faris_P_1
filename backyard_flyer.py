#project 1 backyard flying done by Muhammd Tahir Faris


import argparse 
import time      
from enum import Enum
import datetime  
import csv 

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection, waypoint_error_distance = .5):
        self.run_preffix = f'{datetime.datetime.now():%Y-%m-%d-%H-%M}' 
        telemetry_file_name = self.run_preffix + 'TLog.txt'  
        super().__init__(connection, tlog_name=telemetry_file_name ) 
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.waypoint_error_distance = waypoint_error_distance 

        # initial state
        self.flight_state = States.MANUAL

        # Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
     
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                if len(self.all_waypoints) == 0:
                    self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
                self.flight_state = States.WAYPOINT
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < self.waypoint_error_distance:
                if len(self.all_waypoints) > 0: # if the are more waypoint, continue
                    self.waypoint_transition()
                    self.flight_state = States.WAYPOINT
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()
                        self.flight_state = States.LANDING

    def velocity_callback(self):
    
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()
                    self.flight_state = States.DISARMING

    def state_callback(self):
     
        if self.in_mission: # if in a mission
            if self.flight_state == States.MANUAL:
                self.arming_transition()
                self.flight_state = States.ARMING
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
                    self.flight_state = States.TAKEOFF
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()
                    self.flight_state = States.MANUAL

    def calculate_box(self):
     
        print('Calculating waypoints')
        local_waypoints = [[10.0, 0.0, 10.0], [10.0, 10.0, 10.0], [0.0, 10.0, 10.0], [0.0, 0.0, 10.0]]
        return local_waypoints

    def arming_transition(self):
       
        print('arming transition')
        self.take_control()
        self.arm()
        #  current position is home
        self.set_home_position(self.global_position[0], self.global_position[1],
                               self.global_position[2])



    def takeoff_transition(self):
      
        print('takeoff transition')
        target_altitude = 10.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)

    def waypoint_transition(self):
      
        print('waypoint transition')
        waypoint = self.all_waypoints.pop(0) #  the next waypoint
        self.target_position = waypoint # Assigning it to goal position
        print('waypoint loaded', waypoint)
        # instruct the drone to move to the waypoint
        self.cmd_position(waypoint[0], waypoint[1], waypoint[2], 0.0)

    def landing_transition(self):
      
        print('landing transition')
        self.land()

    def disarming_transition(self):
      
        print('disarm transition')
        self.disarm()
        self.release_control()

    def manual_transition(self):
       
        print('manual transition')
        distance_from_home = 100*np.linalg.norm(self.local_position)
        print(f'Distance from home : {distance_from_home:3.2f} cm')

        self.release_control()
        self.stop()
        self.in_mission = False

    def start(self):
      
        print('Creating log file')
        self.start_log('Logs', self.run_preffix + 'NavLog.txt')
        print('starting connection')
        self.connection.start()
        while self.in_mission:
           pass
        print('Closing log file')
        self.stop_log()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help='host address, i.e. 127.0.0.1')
    parser.add_argument('--waypoint_file', type=str, default='', help='csv containing waypoints, i.e. square_waypoints.csv')
    args = parser.parse_args()
    waypoints = []
    if args.waypoint_file != '':
        print('Using waypoint file : ' + args.waypoint_file )
        with open(args.waypoint_file) as dataFile:
            dataReader = csv.reader(dataFile)
            for row in dataReader:
                waypoint = [ float(row[0]), float(row[1]), float(row[2]) ]
                waypoints.append(waypoint)

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=True, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    drone.all_waypoints = waypoints
    time.sleep(2)
    drone.start()