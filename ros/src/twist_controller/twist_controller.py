from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        wheel_base = kwargs['wheel_base']
        steer_ratio = kwargs['steer_ratio']
        max_lat_accel = kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']
	vehicle_mass = kwargs['vehicle_mass']
	decel_limit = kwargs['decel_limit']
	wheel_radius = kwargs['wheel_radius']	
		
	min_speed = 0.001
	throttle = 0.0
	brake = 0.0
	steer = 0.0
	self.prev_throttle = 0.0
	self.torque_for_max_brake = 0.6*vehicle_mass*abs(decel_limit)*wheel_radius
        #self.linear_pid = PID(kp=0.1, ki=0.0, kd=0.03, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
	self.linear_pid = PID(kp=5.0, ki=0.05, kd=0.0, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
        self.steer_pid = PID(kp=2., ki=0.1, kd=0.3, mn=-kwargs['max_steer_angle'], mx=kwargs['max_steer_angle'])
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.last_time = rospy.get_time()
        #pass

    def gradual(self, value, maxvalue, scalevalue):
	if value <= 0.0:
	   return 0.0
	else:
	   return maxvalue*math.tanh(value*scalevalue)

    def get_interval(self):
    	now = rospy.get_time()
    	interval = now - self.last_time
    	self.last_time = now
    	return interval

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        #rospy.logwarn('linear_velocity %s', linear_velocity)
        interval = self.get_interval()
	if dbw_enabled:
	   linear_err = linear_velocity - current_velocity
	   throttle = self.linear_pid.step(linear_err, interval)
	   if throttle >= 0.0:
	   	throttle = self.gradual(throttle, 0.75, 0.6)
		if throttle - self.prev_throttle > 0.01: #max throttle change
		   throttle = self.prev_throttle + 0.01
		brake = 0.0
		self.prev_throttle = throttle
	   else:
		brake = self.gradual(-throttle, self.torque_for_max_brake, 0.3)
		throttle = 0.0
		self.prev_throttle = 0.0
           steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
           #steer = self.steer_pid.step(steer, interval)
           return throttle, brake, steer
	else:
	   self.linear_pid.reset()
	   return throttle, brake, steer

