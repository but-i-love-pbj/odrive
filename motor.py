#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
import fibre.protocol
import fibre.utils
import fibre.remote_object
from fibre.utils import Event, Logger
from fibre.protocol import ChannelBrokenException, TimeoutError


class Motor:
    def __init__(self,motor_number, odrive_instance=None, m_count=None):
        """
        Connect by serial numbers

        serial numbers:
            207C37823548

        params:
            odrive_serial_id - string id unique to each odroid
            m_count - 0 or 1 indicating if it is on m0 or m1
        """
        motor_map = {1:("207C37823548", 0)}
        

        if not odrive_instance:
            if motor_number in motor_map:
                print("Attempting connection to motor " + str(motor_map[motor_number]))
                odrive_serial_id, m_count = motor_map[motor_number]

            # odrive_search = list(odrive.find_all("usb", odrive_serial_id, did_discover_object, Event(None),None,Logger(verbose=False)))
            self.odrive = odrive.find_any(serial_number=odrive_serial_id)
            print("Found odrive #%d!".format(motor_number))
        else:
            self.odrive = odrive_instance
        
        self.axis = self.odrive.axis1 if m_count else self.odrive.axis0
        self.motor = self.axis.motor
        self.encoder = self.axis.encoder
        self.controller = self.axis.controller
        self.reset_pos()
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


    def reset_pos(self):
        # resets initial, perhaps also triggers to set default positions
        self.initial_count = self.controller.pos_setpoint


    def index_search(self):
        self.axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        while self.axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        self.reset_pos()

    def startup_sequence(self):
        """
        When using gimbal motors, current_lim and calibration_current actually mean 
        “voltage limit” and “calibration voltage”, since we don’t use current feedback.
        """
        self.motor.config.current_lim = 1
        self.motor.config.calibration_current = 1
        self.index_search()
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def get_error(self):
        print("Axis: " + str(hex(self.axis.error)))
        print("Motor: " + str(hex(self.motor.error)))
        print("Controller: " + str(hex(self.controller.error)))
        print("Encoder: " + str(hex(self.encoder.error)))


    def set_trajectory_limits(self, params):
        if params[0]: self.axis.trap_traj.config.vel_limit = params[0]
        if params[1]: self.axis.trap_traj.config.accel_limit = params[1]
        if params[2]: self.axis.trap_traj.config.decel_limit = params[2]
        if params[3]: self.axis.trap_traj.config.A_per_css = params[3]

    def actuate_angle(self, angle, is_absolute=False):
        self.controller.config.control_mode = CTRL_MODE_TRAJECTORY_CONTROL
        if is_absolute:
            target = self.initial_count + angle/360 * self.encoder.config.cpr
        else:
            target = self.controller.pos_setpoint + angle/360 * self.encoder.config.cpr
        self.controller.pos_setpoint = target
        # self.controller.move_to_pos(target)

    def actuate_current(self, current):
        assert (current < self.motor.config.current_lim), "Target current of %dA exceeds limit of %dA".format(current, self.motor.config.current_lim)
        self.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
        self.controller.current_setpoint = current
    

    def set_max_current(self, current):
        self.motor.config.calibration_current = current
        self.motor.config.current_lim = current

    def set_max_force(self, force):
        raise NotImplementedError

    def actuate_force(self, force):
        raise NotImplementedError
