# https://www.pololu.com/file/0J195/line-maze-algorithm.pdf
from threading import Timer

class LineFollowerControl():

    def __init__(self, directions):
        self.directions = directions
        self.sensor_readings = [0, 0, 0, 0, 0]
        self.intersection_to_resolve = None
        self.resolve_intersection = False
        self.line_state = [0,1,0]
        
    def interpret_sensor_readings(self):
        if self.sensor_readings in ([1, 1, 1, 0, 0], [0, 0, 1, 1, 1], [1, 1, 1, 1, 1]):
            self.intersection_to_resolve = self.sensor_readings
            return [None, None, None]

        elif self.sensor_readings == [0, 0, 0, 0, 0]:
            if self.intersection_to_resolve is None:
                return [0, 0, 0]                
            else:                  
                return self.resolve(0)

        else:
            if self.intersection_to_resolve is None:
                return [0, 1, 0]  
            else:                                     
                return self.resolve(1)

    def resolve(self, line_in_front):
        if self.resolve_intersection:
            self.resolve_intersection = False
            if self.intersection_to_resolve == [1, 1, 1, 0, 0]:
                return [1, line_in_front, 0]                
            if self.intersection_to_resolve == [0, 0, 1, 1, 1]:
                return [0, line_in_front, 1] 
            if self.intersection_to_resolve == [1, 1, 1, 1, 1]:
                return [1, line_in_front, 1] 
        else:
            return [None, None, None]

    def stay_on_line(self):
        for i in range(len(self.sensor_readings)):
            if self.sensor_readings[i] == 1:
                return (50 - (i-2) * 5, 50 + (i-2) * 5)

    def trigger_resolve(self):
        self.resolve_intersection == True

    def update(self, sensor_readings):
        self.sensor_readings = sensor_readings
        self.line_state = self.interpret_sensor_readings()
        print(self.sensor_readings)


        if self.line_state == [0, 1, 0]:
            desiredWheelSpeedR, desiredWheelSpeedL = self.stay_on_line()
        elif self.line_state == [None, None, None]:
            timer = Timer(1, self.trigger_resolve)
            timer.start()
            desiredWheelSpeedR, desiredWheelSpeedL = (50, 50)
        else:
            desiredWheelSpeedR, desiredWheelSpeedL = (0, 0)
            # TODO: implement route following at intersections
            # execute self.route[self.curr_step], increment curr_step

        return desiredWheelSpeedR, desiredWheelSpeedL
