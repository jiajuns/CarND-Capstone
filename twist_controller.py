import rospy
from pid import PID
from yaw_controller import yaw_controller
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, vehicle_mass, fuel_capacity, brake_deadband, wheel_radius, decel_limit, acel_limit)):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.pid = PID(5, 0.5,0.5, decel_limit, accel_limit)
        self.steer_lpf = LowPassFilter(3, 1)#25% weight on new, 75%on previous 
        self.accel_lpf = LowPassFilter(3, 1)
        
    def reset(self):
        self.pid.reset()

    def control(self, twist_cmd, current_velocity, del_time):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        linear_vel   = abs(twsit_cmd.twist.linear.x)
        angular_vel  = twist_cmd.twist.angular.z
        velocity_err = linear_vel - current_velocity.twist.linear.x
        
        next_steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_volicty.twist.linear.x)
        next_steer = self.steer_lpf.filt(next_steer)
        
        acceleration = self.pid.step(velocity_err, del_time)
        acceleration = self.accel_lpf.filt(acceleration)
        
        if acceleration > 0.0:
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            deceleration = -1.0 * acceleration
        
            if deceleration < self.brake_deadband:
                deceleration = 0.0
            brake = deceleration * (self.vehicle_mass + self.fuel_capacity* GAS_DENSITY) * self.wheel_radius
        return throttle, brake, next_steer
