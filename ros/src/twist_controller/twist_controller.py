import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, decel_limit, accel_limit, brake_deadband, vehicle_mass, fuel_capacity, wheel_radius):
        self.yaw_controller = YawController(
            wheel_base,
            steer_ratio,
            min_speed,
            max_lat_accel,
            max_steer_angle)

        self.pid = PID(kp=5, ki=0.5, kd=0.5, mn = decel_limit, mx=accel_limit)
        self.s_lpf = LowPassFilter(tau = 3, ts = 1)
        self.t_lpf = LowPassFilter(tau = 3, ts = 1)
        self.brake_deadband = brake_deadband
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.wheel_radius = wheel_radius

    def reset(self):
        self.pid.reset()

    def control(self, twist_cmd, current_velocity, del_time):
        
        lin_vel = abs(twist_cmd.twist.linear.x)
        ang_vel = twist_cmd.twist.angular.z
        vel_err = lin_vel - current_velocity.twist.linear.x

        next_steer = self.yaw_controller.get_steering(lin_vel, ang_vel, current_velocity.twist.linear.x)
        next_steer = self.s_lpf.filt(next_steer)

        acceleration = self.pid.step(vel_err, del_time)
        acceleration = self.t_lpf.filt(acceleration)

        if acceleration > 0.0:
            throttle = acceleration
            deceleration = 0.0
        else:
            throttle = 0.0
            deceleration = -acceleration

            if deceleration < self.brake_deadband:
                deceleration = 0.0

            brake = deceleration * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius

        # Return throttle, brake, steer
        return throttle, brake, next_steer
