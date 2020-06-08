#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import sys
import os

if len(sys.argv) == 1:
    namespace = "/first"
else:
    namespace = sys.argv[1]
# Based on: https://github.com/m-lundberg/simple-pid/blob/master/simple_pid/PID.py
print(namespace)
class PID(object):
    """A simple PID controller."""

    def __init__(
        self,
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        setpoint=0,
        sample_time=0.01,
        output_limits=(None, None),
        auto_mode=True,
        proportional_on_measurement=False,
    ):   #omg so sad

        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self._min_output, self._max_output = output_limits
        self._auto_mode = auto_mode
        self.proportional_on_measurement = proportional_on_measurement

        self.reset()

    def _current_time(self):
        return rospy.get_time()

    def _clamp(self, value, limits):
        lower, upper = limits
        if value is None:
            return None
        elif upper is not None and value > upper:
            return upper
        elif lower is not None and value < lower:
            return lower
        return value

    def __call__(self, input_, dt=None):
        """
        Update the PID controller.
        Call the PID controller with *input_* and calculate and return a control output if
        sample_time seconds has passed since the last update. If no new output is calculated,
        return the previous output instead (or None if no value has been calculated yet).
        :param dt: If set, uses this value for timestep instead of real time. This can be used in
            simulations when simulation time is different from real time.
        """
        if not self.auto_mode:
            return self._last_output

        now = self._current_time()
        if dt is None:
            dt = now - self._last_time if now - self._last_time else 1e-16
        elif dt <= 0:
            raise ValueError('dt has nonpositive value {}. Must be positive.'.format(dt))

        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # only update every sample_time seconds
            return self._last_output

        # compute error terms
        error = self.setpoint - input_
        d_input = input_ - (self._last_input if self._last_input is not None else input_)

        # compute the proportional term
        if not self.proportional_on_measurement:
            # regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            # add the proportional error on measurement to error_sum
            self._proportional -= self.Kp * d_input

        # compute integral and derivative terms
        self._integral += self.Ki * error * dt
        self._integral = self._clamp(self._integral, self.output_limits)  # avoid integral windup

        self._derivative = -self.Kd * d_input / dt

        # compute final output
        # print("Proportional: {}, Integral: {}, Derivative: {}".format(self._proportional,self._integral,self._derivative))
        output = self._proportional + self._integral + self._derivative
        output = self._clamp(output, self.output_limits)

        # keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_time = now

        return output

    def __repr__(self):
        return (
            '{self.__class__.__name__}('
            'Kp={self.Kp!r}, Ki={self.Ki!r}, Kd={self.Kd!r}, '
            'setpoint={self.setpoint!r}, sample_time={self.sample_time!r}, '
            'output_limits={self.output_limits!r}, auto_mode={self.auto_mode!r}, '
            'proportional_on_measurement={self.proportional_on_measurement!r}'
            ')'
        ).format(self=self)

    @property
    def components(self):
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful
        for visualizing what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
         return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings):
        """Set the PID tunings."""
        self.Kp, self.Ki, self.Kd = tunings

    @property
    def auto_mode(self):
        """Whether the controller is currently enabled (in auto mode) or not."""
        return self._auto_mode

    @auto_mode.setter
    def auto_mode(self, enabled):
        """Enable or disable the PID controller."""
        self.set_auto_mode(enabled)

    def set_auto_mode(self, enabled, last_output=None):
        """
        Enable or disable the PID controller, optionally setting the last output value.
        This is useful if some system has been manually controlled and if the PID should take over.
        In that case, pass the last output variable (the control variable) and it will be set as
        the starting I-term when the PID is set to auto mode.
        :param enabled: Whether auto mode should be enabled, True or False
        :param last_output: The last output, or the control variable, that the PID should start
            from when going from manual mode to auto mode
        """
        if enabled and not self._auto_mode:
            # switching from manual mode to auto, reset
            self.reset()

            self._integral = last_output if last_output is not None else 0
            self._integral = self._clamp(self._integral, self.output_limits)

        self._auto_mode = enabled

    @property
    def output_limits(self):
        """
        The current output limits as a 2-tuple: (lower, upper).
        See also the *output_limts* parameter in :meth:`PID.__init__`.
        """
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        """Set the output limits."""
        if limits is None:
            self._min_output, self._max_output = None, None
            return

        min_output, max_output = limits

        if None not in limits and max_output < min_output:
            raise ValueError('lower limit must be less than upper limit')

        self._min_output = min_output
        self._max_output = max_output

        self._integral = self._clamp(self._integral, self.output_limits)
        self._last_output = self._clamp(self._last_output, self.output_limits)

    def reset(self):
        """
        Reset the PID controller internals.
        This sets each term to 0 as well as clearing the integral, the last output and the last
        input (derivative calculation).
        """
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = self._current_time()
        self._last_output = None
        self._last_input = None

class Vehicle(PID):
    def __init__(self,
    namespace,
    Kp=1.0,
    Ki=0.0,
    Kd=0.0,
    setpoint=0,
    sample_time=0.01,
    output_limits=(None, None),
    auto_mode=True,
    proportional_on_measurement=False):
        self.namespace = namespace
        rospy.init_node(self.namespace[1:]+"_" + 'ang_vel_pub_node', anonymous=True)
        super(Vehicle,self).__init__(Kp,
        Ki,
        Kd,
        setpoint,
        sample_time,
        output_limits,
        auto_mode,
        proportional_on_measurement)

        self.lane_correction = Float64()
        self.speed = Twist()
        self.pub = rospy.Publisher(self.namespace+'/lane_keep_ang_vel', Float64, queue_size=10)
        self.filename = __file__[:-19] + namespace[1:]+"_pid_tunings.txt"

    def read_pid_tunings(self):
        with open(self.filename,"r") as f:
            contents = f.read()
        tunings_as_string = contents.split(",")
        try:
            self.Kp = float(tunings_as_string[0])
            self.Ki = float(tunings_as_string[1])
            self.Kd = float(tunings_as_string[2])
        except Exception as e:
            print(e)


    def _subscriber(self):
        rospy.Subscriber(self.namespace+'/angular_error', Float64, self.callback)
        rospy.Subscriber(self.namespace+"/pid_tunings",Float64MultiArray, self._tuner_callback)
        rospy.spin()

    def callback(self,ang_error):
        self.__call__(ang_error.data)
        self._publisher()

    def _tuner_callback(self,tunings):
        self.Kp = tunings.data[0]
        self.Ki = tunings.data[1]
        self.kd = tunings.data[2]

    def _publisher(self):
        publ_lane_keep_ang_vel= Float64()
        publ_lane_keep_ang_vel.data = self._last_output
        self.pub.publish(publ_lane_keep_ang_vel)


car = Vehicle(namespace,Kp=0.0027,Ki=0.001,Kd=0,setpoint=0,sample_time=0.002,output_limits=(-2,2))
car.read_pid_tunings()
print(car.tunings)
car._subscriber()
