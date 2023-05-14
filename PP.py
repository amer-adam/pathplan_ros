#!/usr/bin/python3

from math import cos, sin, atan2, radians
from numpy import array, dot
import rospy
from std_msgs.msg import Float64, Bool
from time import sleep


class PathPlanner:

    pos_x = 0.0
    pos_y = 0.0
    yaw = 0.0
    prev_x = 0.0
    prev_y = 0.0
    prev_real_x = 0.0
    prev_real_y = 0.0
    del_pos_x = 0.0
    del_pos_y = 0.0
    yaw_constant = 0.0
    prev_yaw = 0.0
    real_x = 0.0
    real_y = 0.0
    real_z = 0.0
    real_z_rad = 0.0
    target_vel = []
    target_x = []
    target_y = []
    target_z = []
    target_accurate = []
    error_x = 0.0
    error_y = 0.0
    error_z = 0.0
    rvx = 0.0
    rvy = 0.0
    point_count = 0
    points_number = 0.0
    # point lock calculations
    lock_enable = False
    lock = 0.0
    lock_count = 0.0
    lock_calc = 0.0
    lock_final = 0.0
    # pid
    outz = 0.0
    setpointMsg = Float64()
    stateMsg = Float64()
    pidEnableMsg = Bool()

    def __init__(self) -> None:
        self.pp_start = False
        rospy.init_node('PP')
        rospy.Subscriber("control_effort", Float64,
                         self.control_effort_callback)
        self.statePub = rospy.Publisher("state", Float64, queue_size=10)
        self.setpointPub = rospy.Publisher("setpoint", Float64, queue_size=10)
        self.pidEnablePub = rospy.Publisher("pid_enable", Bool, queue_size=10)

        dur = rospy.Duration(1)
        rospy.sleep(dur)

        # while self.pidEnablePub.get_num_connections() < 1: 
        #     pass
        self.pidEnableMsg.data = False
        self.pidEnablePub.publish(self.pidEnableMsg)
        

        # self.reset()

    def control_effort_callback(self, controlEffort):
        self.outz = controlEffort.data *5.5

    def set_xyz(self, x, y, z):
        self.pos_x = x
        self.pos_y = y
        self.yaw = z

    def reset(self):
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_real_x = 0.0
        self.prev_real_y = 0.0
        self.del_pos_x = 0.0
        self.del_pos_y = 0.0
        self.yaw_constant = 0.0
        self.prev_yaw = 0.0
        self.real_x = 0.0
        self.real_y = 0.0
        self.real_z = 0.0
        self.real_z_rad = 0.0
        self.target_vel = []
        self.target_x = []
        self.target_y = []
        self.target_z = []
        self.target_accurate = []
        self.error_x = 0.0
        self.error_y = 0.0
        self.error_z = 0.0
        self.rvx = 0.0
        self.rvy = 0.0
        self.point_count = 0
        self.points_number = 0.0
        # point lock calculations
        self.lock_enable = False
        self.lock = 0.0
        self.lock_count = 0.0
        self.lock_calc = 0.0
        self.lock_final = 0.0

    def stop(self):
        self.pidEnableMsg.data = False
        self.pidEnablePub.publish(self.pidEnableMsg)
        self.pp_start = False
        self.reset()

    def lock_stop(self):
        if (self.target_accurate[self.point_count]):
            if (self.lock_enable):
                self.stop()
        else:
            self.stop()

    def start(self, points, pointsNumber):
        self.pidEnableMsg.data = True
        self.pidEnablePub.publish(self.pidEnableMsg)
        for point in range(0, pointsNumber):
            self.target_vel.append(points[point][0])
            self.target_x.append(points[point][1])
            self.target_y.append(points[point][2])
            self.target_z.append(points[point][3])
            self.target_accurate.append(points[point][4])
        self.points_number = pointsNumber

        self.pp_start = True

    def calculate(self, tol_x, tol_y, tol_z):
        self.update_real_yaw()
        self.update_real_position()

        if self.pp_start:
            self.calculate_errors()
            self.calculate_locking(tol_x, tol_y, tol_z)

            if self.check_tolerance(tol_x, tol_y, tol_z):
                if (self.point_count == self.points_number - 1):
                    self.lock_stop()
                    return 0, 0, 0
                else:
                    self.point_count += 1
                    self.calculate_errors()
            return self.calculate_velocity()

    def update_real_yaw(self):
        # Real Yaw Calculations
        if self.yaw < 30 and self.prev_yaw > 330:
            self.yaw_constant += 1
        elif self.prev_yaw < 30 and self.yaw > 330:
            self.yaw_constant -= 1

        self.prev_yaw = self.yaw
        self.real_z = self.yaw + self.yaw_constant * 360
        self.real_z_rad = radians(self.real_z)

    def update_real_position(self):
        # Real X & Y Calculations
        cos_z = cos(self.real_z_rad)
        sin_z = sin(self.real_z_rad)

        self.rotation_matrix = array([[cos_z, sin_z], [-sin_z, cos_z]])
        delta_matrix = array(
            [self.pos_x - self.prev_x, self.pos_y - self.prev_y])
        delta = dot(self.rotation_matrix, delta_matrix)
        self.del_pos_x, self.del_pos_y = delta

        self.real_x += self.del_pos_x
        self.real_y += self.del_pos_y

        self.prev_x = self.pos_x
        self.prev_y = self.pos_y
        self.prev_real_x = self.real_x
        self.prev_real_y = self.real_y

    def calculate_errors(self):
        self.error_x = self.target_x[self.point_count] - self.real_x
        self.error_y = self.target_y[self.point_count] - self.real_y
        self.error_z = self.target_z[self.point_count] - self.real_z

    def check_tolerance(self, tolX, tolY, tolZ):
        return abs(self.error_x) <= tolX and abs(self.error_y) <= tolY and abs(self.error_z) <= tolZ

    def check_average_tolerance(self, tolX, tolY, tolZ):
        return (abs(self.error_x) + abs(self.error_y))/2 < (tolX + tolY)/2 and abs(self.error_z) < tolZ

    def calculate_velocity(self):
        # Calculate velocity for the 4 motors
        delta_x = self.target_x[self.point_count] - self.prev_real_x
        delta_y = self.target_y[self.point_count] - self.prev_real_y
        heading = atan2(delta_y, delta_x)

        vx = self.target_vel[self.point_count] * cos(heading)
        vy = self.target_vel[self.point_count] * sin(heading)

        self.PID()

        velocity_vector = array([vx, vy])
        rotated_velocity = dot(self.rotation_matrix, velocity_vector)

        self.rvx, self.rvy = rotated_velocity

        return self.rvx, self.rvy, self.outz

    def calculate_locking(self, tol_x, tol_y, tol_z):
        if (self.check_average_tolerance(tol_x, tol_y, tol_z)):
            self.lock = 1.0
        else:
            self.lock = 0.0

        self.lock_count += 1.0

        if (self.lock_count <= 60):
            self.lock_calc += self.lock
        else:
            self.lock_final = self.lock_calc / self.lock_count
            if (self.lock_final >= 0.95):
                self.lock_enable = True
            else:
                self.lock_enable = False
                self.lock_calc = 0.0
                self.lock_count = 0.0

    def PID(self):
        self.setpointMsg.data = self.target_z[self.point_count] / 90
        self.stateMsg.data = self.real_z / 90
        self.setpointPub.publish(self.setpointMsg)
        self.statePub.publish(self.stateMsg)
