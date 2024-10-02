#!/usr/bin/env python
from enum import Enum

class RP_DataType(Enum):
    two_byte_signed = 0x01
    beacon = 0x40
    gps = 0x41

#VNET is the same as CAN ID
class RP_VNET_ID(Enum):
    engine_rpm = 0x200
    digital_in_2 = 0x7c1
    digital_in_3 = 0x7c2
    digital_in_4 = 0x7c3
    aux_digital_1 = 0x7d1
    aux_digital_2 = 0x7d2
    aux_digital_3 = 0x7d3
    lf_shock = 0x3a4
    rf_shock = 0x3a5
    analog_in_3 = 0x7ca
    analog_in_4 = 0x7cb
    lr_shock = 0x3a6
    rr_shock = 0x3a7
    analog_in_7 = 0x7ce
    analog_in_8 = 0x7cf
    lateral_g = 0x3a1
    forward_g = 0x3a0
    battery_voltage =  0x440
    aux_digital_4 = 0x7d3
    steering_pos = 0x3a8
    throttle_pos = 0x30b
    brake_press_1 = 0x421
    brake_press_2 = 0x422
    lf_wheel_rpm = 0x3b4
    rf_wheel_rpm = 0x3b5
    lr_wheel_rpm = 0x3b6
    rr_wheel_rpm = 0x3b7

