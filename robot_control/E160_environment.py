from E160_robot import *
from E160_state import *
from E160_wall import *
import serial
import time
#import ../path_planning/GraphCreator
from xbee import XBee


class E160_environment:

    def __init__(self):
        self.width = 3.5
        self.height = 3.0

        # set up walls, putting top left point first
        self.walls = []
        lines = [[7,10,7,0], [0,0,10,0], [2,2,2,-2], [2,1,7,1],\
                [7,6,15,6], [7,4,15,4], [10,2,10,0], [10,1,15,1] ,[15,6,15,-2]]

        for wall in lines:
            # Scale the walls down to reasonable sizes
            wall[:] = [x/5 for x in wall]

            # Shift it into the frame
            wall[0] = wall[0] - 1.5
            wall[2] = wall[2] - 1.5

            wall[1] = wall[1] - 1
            wall[3] = wall[3] - 1

            if wall[0] == wall[2]:
                self.walls.append(E160_wall(wall, "vertical"))
            elif wall[1] == wall[3]:
                self.walls.append(E160_wall(wall, "horizontal"))
            else:
                print('Discarded wall:', wall)


        # self.walls.append(E160_wall([0.5, 0.5, 0.5, -0.5], "vertical"))
        # self.walls.append(E160_wall([-0.5, 0.5, 0.5, 0.5], "horizontal"))
        # self.walls.append(E160_wall([0.5, -0.5, 1.0, -0.5], "horizontal"))
        # self.walls.append(E160_wall([0.0, -0.5, 0.0, -1.0], "vertical"))

        # create vars for hardware vs simulation
        self.robot_mode = "SIMULATION MODE"  # "SIMULATION MODE" or "HARDWARE MODE"
        self.control_mode = "LINE FOLLOW MODE"

        # setup xbee communication
        if (self.robot_mode == "HARDWARE MODE"):
            self.serial_port = serial.Serial(
                '/dev/tty.usbserial-DN02Z1BF', 9600)
            print(" Setting up serial port")
            try:
                self.xbee = XBee(self.serial_port)
            except:
                print("Couldn't find the serial port")

        # Setup the robots
        self.num_robots = 1
        self.robots = []
        for i in range(0, self.num_robots):

            # TODO: assign different address to each bot
            r = E160_robot(self, '\x00\x0C', i)
            self.robots.append(r)

    def plan_path(self):
        start = [15, -2]
        end = [2, 1]

        for r in self.robots:
            #r.route = directions(self.walls, start, end, True)
            r.route = [1, 1, 1]
            print(r.route)

    def update_robots(self, deltaT):

        # loop over all robots and update their state
        for r in self.robots:

            # set the control actuation
            r.update(deltaT)

    def log_data(self):

        # loop over all robots and update their state
        for r in self.robots:
            r.log_data()

    def quit(self):
        self.xbee.halt()
        self.serial_port.close()
