
from E160_state import *
import math
import datetime
from LineFollowerControl import LineFollowerControl


class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.environment = environment
        self.state_est = E160_state()
        self.state_est.set_state(1.5, -1.4, -1.57)
        self.state_des = E160_state()
        self.state_des.set_state(1.5, -1.4, -1.57)
        #self.v = 0.05
        #self.w = 0.1
        self.R = 0
        self.L = 0
        self.radius = 0.069
        self.width = 2*self.radius
        self.wheel_radius = 0.0344
        self.wheel_circ = self.wheel_radius * 2 * math.pi
        self.address = address
        self.ID = self.address.encode().__str__()[-1]
        self.last_measurements = []
        self.robot_id = robot_id
        self.manual_control_left_motor = 0
        self.manual_control_right_motor = 0
        self.file_name = 'Log/Bot9' + str(self.robot_id) + '_' + datetime.datetime.now(
        ).replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.csv'
        self.make_headers()
        self.data = None  # String that will be logged every time step.
        self.encoder_resolution = 1437

        self.last_encoder_measurements = [0, 0]
        self.encoder_measurements = [0, 0]
        self.range_measurements = [0, 0, 0]
        self.light_measurements = [0, 0, 0, 0, 0]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0

        """
        routes are encoded as an array of turns to perform at each intersection,
        the options for each element are:
            1 to turn to the right
            0 to keep going straight
            -1 to turn to the left
        """
        self.route = [1,-1]
        self.route_step = 1
        self.at_intersection = False

        self.Kpho = 1 / self.wheel_radius
        self.Kalpha = 6 / self.wheel_radius
        self.Kbeta = -15 / self.wheel_radius
        # self.Kpho = 1.5
        # self.Kalpha = 4
        # self.Kbeta = -1.04

        self.max_velocity = 0.05
        self.point_tracked = True
        self.encoder_per_sec_to_rad_per_sec = 10

        # Path point: [x, y, theta, reset_robot_state_when_done?]
        self.path_points = [
            [0, 0, 0, True, 0.05, 0.1, -0.05],
            [0.35, 0.35, 1.57, False],
            [0, 0, 0, True, 0.05, 0.1, -0.05]
        ]

        self.whichPoint = 0

    def reset(self):
        self.state_est.set_state(0, 0, 0)
        self.state_des.set_state(0, 0, 0)
        self.R = 0
        self.L = 0
        self.last_measurements = []
        self.file_name = 'Log/Bot9' + str(self.robot_id) + '_' + datetime.datetime.now(
        ).replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.csv'
        self.make_headers()
        self.data = None  # String that will be logged every time step.
        self.last_encoder_measurements = [0, 0]
        self.encoder_measurements = [0, 0]
        self.range_measurements = [0, 0, 0]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0
        self.point_tracked = True

    def update(self, deltaT):

        # get sensor measurements
        self.encoder_measurements,\
            self.range_measurements,\
            self.light_measurements = self.update_sensor_measurements(deltaT)

        # localize
        self.state_est = self.localize(
            self.state_est, self.encoder_measurements, self.range_measurements)

        # call motion planner
        # self.motion_planner.update_plan()

        # determine new control signals
        self.R, self.L = self.update_control(self.range_measurements)

        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)

    def update_sensor_measurements(self, deltaT):

        if self.environment.robot_mode == "HARDWARE MODE":
            command = '$S @'
            self.environment.xbee.tx(dest_addr=self.address, data=command)

            update = self.environment.xbee.wait_read_frame()

            data = update['rf_data'].decode().split(' ')
            data = [int(x) for x in data]
            encoder_measurements = data[-2:]
            range_measurements = [data[0], 0, 0]
            light_raw = data[1:-2][::-1]
            offset = 75
            threshold = [776 + offset, 831 + offset, 591 + offset, 637 + offset, 743 + offset][::-1]
            light_measurements = [1 if l > thresh else 0 for l, thresh in zip(light_measurements_raw, threshold)]

            print('light measurements:', light_measurements)
            print('raw measurements:', light_measurements_raw)


            # TODO: Update threshold
            thresh = 0

            for i in range(len(light_measurements)):
                if light_measurements[i] > thresh:
                    light_measurements[i] = 1
                else:
                    light_measurements[i] = 0



        elif self.environment.robot_mode == "SIMULATION MODE":
            encoder_measurements = self.simulate_encoders(
                self.R, self.L, deltaT)
            range_measurements = [0, 0, 0]
            light_measurements = self.simulate_light_sensors()

        return encoder_measurements, range_measurements, light_measurements

    def simulate_light_sensors(self):
        sensor_offsets = [(0, -0.06), (0, -0.03), (0, 0), (0, 0.03), (0, 0.06)]
        measurements = [0, 0, 0, 0, 0]
        for i in range(len(sensor_offsets)):
            x = self.state_est.x + sensor_offsets[i][0] * math.cos(self.state_est.theta) \
                - sensor_offsets[i][1] * math.sin(self.state_est.theta)
            y = self.state_est.y + sensor_offsets[i][0] * math.sin(self.state_est.theta) \
                                 + sensor_offsets[i][1] * \
                math.cos(self.state_est.theta)
            for wall in self.environment.walls:
                if self.is_point_over_wall(wall, x, y):
                    measurements[i] = 1
                    break
                else:
                    measurements[i] = 0
        return measurements

    def is_point_over_wall(self, wall, x, y):
        if wall.slope == 'horizontal':
            between_x = wall.points[0] <= x <= wall.points[-2]
            between_y = wall.points[1] <= y <= wall.points[3]
        elif wall.slope == 'vertical':
            between_x = wall.points[0] <= x <= wall.points[2]
            between_y = wall.points[-1] <= y <= wall.points[1]
        return between_x and between_y

    def localize(self, state_est, encoder_measurements, range_measurements):
        delta_s, delta_theta = self.update_odometry(encoder_measurements)
        state_est = self.update_state(state_est, delta_s, delta_theta)

        return state_est

    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi

        return a

    def update_control(self, range_measurements):

        if self.environment.control_mode == "MANUAL CONTROL MODE":
            desiredWheelSpeedR = self.manual_control_right_motor
            desiredWheelSpeedL = self.manual_control_left_motor

        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":
            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()

        elif self.environment.control_mode == "PATH TRACKER":
            p = self.path_points[self.whichPoint]
            self.state_des.set_state(p[0], p[1], p[2])

            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()

            if self.point_tracked and self.whichPoint < len(self.path_points) - 1:
                if p[3]:
                    self.reset()
                    self.Kpho = p[4] / self.wheel_radius
                    self.Kalpha = p[5] / self.wheel_radius
                    self.Kbeta = p[6] / self.wheel_radius
                self.whichPoint += 1
                self.point_tracked = False

        elif self.environment.control_mode == "LINE FOLLOW MODE":
            if not self.point_tracked:
                desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()
            else:
                desiredWheelSpeedR, desiredWheelSpeedL = self.line_follow_control()

        return desiredWheelSpeedR, desiredWheelSpeedL

    def line_follow_control(self):
        if sum(self.light_measurements) >= 3: # Intersection
            target_t = - math.pi/2 - sum(self.route[:self.route_step]) * math.pi/2
            target_t = self.angle_wrap(target_t)
            self.state_des.set_state(
                self.state_est.x - 0.1 * math.cos(target_t),
                self.state_est.y - 0.1 * math.sin(target_t),
                target_t
            )
            self.route_step += 1
            self.point_tracked = False
            return 0, 0

        if sum(self.light_measurements) == 0: # No line
            return 0, 0

        # Follow the line:
        left = 50
        right = 50
        for i in range(len(self.light_measurements)):
            if self.light_measurements[i] == 1:
                left -= (i-2) * 2
                right += (i-2) * 2
        return left, right

    def point_tracker_control(self):
        # If the desired point is not tracked yet, then track it
        if not self.point_tracked:

            ############ Student code goes here ############################################

            # Calculate the rho, alpha, beta coordinates
            delta_x = self.state_des.x - self.state_est.x
            delta_y = self.state_des.y - self.state_est.y
            theta = self.angle_wrap(self.state_est.theta)

            pho = math.sqrt(delta_x**2 + delta_y**2)
            alpha = self.angle_wrap(-theta + math.atan2(delta_y, delta_x))

            if -math.pi/2 < alpha and alpha < math.pi/2:  # check that goal is in front
                desiredV = self.Kpho * pho
            else:
                alpha = self.angle_wrap(-theta +
                                        math.atan2(-delta_y, -delta_x))
                desiredV = -self.Kpho * pho

            beta = self.angle_wrap(self.state_des.theta - theta - alpha)

            desiredW = self.Kalpha * alpha + self.Kbeta * beta

            if abs(delta_x) < 0.02 and abs(delta_y) < 0.02:
                beta = self.angle_wrap(theta - self.state_des.theta)

                desiredV = 0
                desiredW = self.Kbeta * beta
                if abs(self.angle_wrap(self.state_des.theta - theta)) < 0.1:
                    self.point_tracked = True

            omega1 = (self.radius*desiredW + desiredV) / self.width
            omega2 = -(-self.radius*desiredW + desiredV) / self.width

            desiredVelocityR = omega1 * self.width
            desiredVelocityL = -omega2 * self.width

            if abs(desiredVelocityL) > self.max_velocity or \
                    abs(desiredVelocityR) > self.max_velocity:
                vel = self.normalize_vel(desiredVelocityL, desiredVelocityR)
                desiredVelocityL = vel[0]
                desiredVelocityR = vel[1]

            # check if velocities are small enough

            control = self.velocity_to_wheel_control(
                desiredVelocityL, desiredVelocityR)
            desiredWheelSpeedL = control[0]
            desiredWheelSpeedR = control[1]

            # print desiredV, desiredW
            # print desiredVelocityL, desiredVelocityR

            self.data = [self.state_des.x, self.state_est.x, self.state_des.y, self.state_est.y, self.state_des.theta,
                         self.state_est.theta, pho, alpha, beta, self.Kpho, self.Kalpha, self.Kbeta, desiredV, desiredW]

        # the desired point has been tracked, so don't move
        else:
            desiredWheelSpeedR = 0
            desiredWheelSpeedL = 0

        return desiredWheelSpeedR, desiredWheelSpeedL

    def velocity_to_wheel_control(self, lv, rv):
        deltaEML = self.encoder_measurements[1] - \
            self.last_encoder_measurements[1]
        if deltaEML == 0:
            adjustment = 12 if lv > 0 else -12
            l_control = -287.52156 * lv - adjustment
        else:
            l_control = -289.35185 * lv

        deltaEMR = self.encoder_measurements[0] - \
            self.last_encoder_measurements[0]
        if deltaEMR == 0:
            adjustment = 12 if rv > 0 else -12
            r_control = -287.52156 * rv - adjustment
        else:
            r_control = -291.29041 * rv

        return (r_control, l_control)

    def normalize_vel(self, lv, rv):
        scale = max(abs(lv), abs(rv))/self.max_velocity
        return (lv/scale, rv/scale)

    def send_control(self, R, L, deltaT):

        # send to actual robot !!!!!!!!
        if self.environment.robot_mode == "HARDWARE MODE":
            if (L < 0):
                LDIR = 0
            else:
                LDIR = 1

            if (R < 0):
                RDIR = 0
            else:
                RDIR = 1
            RPWM = int(abs(R))
            LPWM = int(abs(L))

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + \
                ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            self.environment.xbee.tx(dest_addr=self.address, data=command)

    def simulate_encoders(self, R, L, deltaT):
        right_encoder_measurement = - \
            int(R*self.encoder_per_sec_to_rad_per_sec*deltaT) + \
            self.last_simulated_encoder_R
        left_encoder_measurement = - \
            int(L*self.encoder_per_sec_to_rad_per_sec*deltaT) + \
            self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement

        # print "simulate_encoders", R, L, right_encoder_measurement, left_encoder_measurement
        return [left_encoder_measurement, right_encoder_measurement]

    def make_headers(self):
        f = open(self.file_name, 'a+')
        f.write(','.join(['time', 'state_des.x', 'state_est.x', 'state_des.y', 'state_est.y', 'state_des.theta',
                          'state_est.theta', 'pho', 'alpha', 'beta', 'Kpho', 'Kalpha', 'Kbeta', 'desiredV', 'desiredW']) + '\n')
        f.close()

    def log_data(self):
        if self.data:
            f = open(self.file_name, 'a+')

            data = [str(x) for x in self.data]
            data.insert(0, str(datetime.datetime.now()))

            f.write(','.join(data) + '\n')
            f.close()

    def set_manual_control_motors(self, R, L):

        self.manual_control_right_motor = int(R*256/100)
        self.manual_control_left_motor = int(L*256/100)

    def update_odometry(self, encoder_measurements):

        delta_s = 0
        delta_theta = 0

        # ****************** Additional Student Code: Start ************

        deltaEML = encoder_measurements[1] - self.last_encoder_measurements[1]
        deltaEMR = encoder_measurements[0] - self.last_encoder_measurements[0]

        self.last_encoder_measurements[1] = encoder_measurements[1]
        self.last_encoder_measurements[0] = encoder_measurements[0]

        if abs(deltaEML) > 250 or abs(deltaEMR) > 250:
            deltaEML = 0
            deltaEMR = 0

        wheelDistanceL = float(
            deltaEML) / float(self.encoder_resolution) * float(self.wheel_circ)
        wheelDistanceR = float(
            deltaEMR) / float(self.encoder_resolution) * float(self.wheel_circ)

        # print("encoder: " + str(encoder_measurements[0]) + " last encoder: " + str(self.last_encoder_measurements[0]))

        delta_theta = (wheelDistanceR - wheelDistanceL) / (2 * self.radius)
        # Wrap the angle
        delta_theta = ((delta_theta + math.pi) % (2 * math.pi)) - math.pi

        delta_s = (wheelDistanceR + wheelDistanceL)/2.0

        # ****************** Additional Student Code: End ************

        # keep this to return appropriate changes in distance, angle
        return delta_s, delta_theta

    def update_state(self, state, delta_s, delta_theta):

        # ****************** Additional Student Code: Start ************

        x = self.state_est.x + delta_s * \
            math.cos(self.state_est.theta + delta_theta/2)
        y = self.state_est.y + delta_s * \
            math.sin(self.state_est.theta + delta_theta/2)

        theta = self.state_est.theta + delta_theta
        # Wrap the angle
        theta = ((theta + math.pi) % (2 * math.pi)) - math.pi

        state.set_state(x, y, theta)

        # ****************** Additional Student Code: End ************

        # keep this to return the updated state
        return state
