# https://www.pololu.com/file/0J195/line-maze-algorithm.pdf
from threading import Timer

class LineFollowerControl():

    def __init__(self, directions):
        self.directions = directions
        self.sensor_readings = [0, 0, 0, 0, 0]
        self.intersection_to_resolve = None
        self.resolve_step = None
        self.intersection_step = None
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
        if self.intersection_to_resolve == [1, 1, 1, 0, 0]:
            state = [1, line_in_front, 0]                
        if self.intersection_to_resolve == [0, 0, 1, 1, 1]:
            state = [0, line_in_front, 1] 
        if self.intersection_to_resolve == [1, 1, 1, 1, 1]:
            state = [1, line_in_front, 1] 
        
        self.intersection_to_resolve = None
        return state

    def stay_on_line(self):
        left = 50 
        right = 50
        for i in range(len(self.sensor_readings)):
            if self.sensor_readings[i] == 1:
                left -= (i-2) * 5
                right += (i-2) * 5
        return left, right

    def resolve_intersection(self):
        if self.resolve_step < 6:
            speed = (50, 50)
        elif self.resolve_step == 6:
            self.line_state = self.interpret_sensor_readings()
            speed = (0, 0)
        elif self.resolve_step < 12:
            speed = (-50, -50)
        else:
            self.resolve_step = None
            self.intersection_step = 0
            return (0, 0)
        
        self.resolve_step += 1
        return speed
    
    def intersection_mode(self):
        if self.intersection_step < 6:
            speed = (-50, 50)
            self.intersection_step += 1            
        elif self.intersection_step == 6:
            if self.sensor_readings != [1,1,1,0,0]:
                speed = (-50, 50)         
            else:
                speed = (0, 0)
                self.intersection_step += 1   
        elif self.intersection_step < 12:
            speed = (50, 50)
            self.intersection_step += 1 
        else:
            self.intersection_step = None
            return (0,0)
        return speed

    def update(self, sensor_readings):
        self.sensor_readings = sensor_readings
        print(sensor_readings)
        if self.resolve_step is not None:
            desiredWheelSpeedR, desiredWheelSpeedL = self.resolve_intersection()
        elif self.intersection_step is not None:                
            desiredWheelSpeedR, desiredWheelSpeedL = self.intersection_mode()
        else:
            self.line_state = self.interpret_sensor_readings()
            if self.line_state == [0, 1, 0]:
                desiredWheelSpeedR, desiredWheelSpeedL = self.stay_on_line()
            elif self.line_state == [None, None, None]:
                self.resolve_step = 0
                desiredWheelSpeedR, desiredWheelSpeedL = (0, 0)
            else:
                desiredWheelSpeedR, desiredWheelSpeedL = (0, 0)
                
        
        return desiredWheelSpeedR, desiredWheelSpeedL
