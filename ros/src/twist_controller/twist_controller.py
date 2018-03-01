from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        wheel_base = kwargs['wheel_base']
        steer_ratio = kwargs['steer_ratio']
        min_speed = 0.001
        max_lat_accel = kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']

        self.linear_pid = PID(kp=0.1, ki=0.0, kd=0.03, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.last_time = rospy.get_time()
        pass

    def get_interval(self):
    	now = rospy.get_time()
    	interval = now - self.last_time
    	self.last_time = now
    	return interval

    def control(self, linear_velocity, angular_velocity, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        interval = self.get_interval()
        linear_err = linear_velocity - current_velocity
       	throttle = self.linear_pid.step(linear_err, interval)
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        return throttle, 0., steer
