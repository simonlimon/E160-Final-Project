import random
import time
from E160_environment import *
from E160_graphics import *

def gui_main(directions = None):

    # instantiate robot navigation classes
    environment = E160_environment()
    graphics = E160_graphics(environment)

    environment.plan_path(directions)

    # set time step size in seconds
    deltaT = 0.1
    # loop over time
    while True:
        # update graphics, but stop the thread if user stopped the gui
        if not graphics.update():
            break

        # update robots
        environment.update_robots(deltaT)

        # log all the robot data
        environment.log_data()

        # maintain timing
        if environment.robot_mode == "SIMULATION MODE":
            time.sleep(deltaT)

if __name__ == '__main__':
    gui_main()
