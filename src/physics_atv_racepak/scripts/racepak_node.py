#!/usr/bin/env python3
import rospy
from racepak.msg import rp_controls
from racepak.msg import rp_shock_sensors
from racepak.msg import rp_wheel_encoders
from nav_msgs.msg import Odometry
from struct import *
import csv
import time
#install https://pypi.org/project/pyserial/
import serial

class RacepakNode(object):
    def __init__(self, datafilename=''):

        if len(datafilename)!=0: # in the debugging mode
            self.racepak = open(datafilename, 'rb')
        else: 
            # open the serial port
            port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
            self.baud_rate = 57600
            parity = serial.PARITY_NONE
            data_width = serial.EIGHTBITS
            stop_bits = serial.STOPBITS_ONE
            self.racepak = serial.Serial(port,self.baud_rate, data_width, parity, stop_bits, xonxoff=False)
            self.racepak.flushInput()
            rospy.loginfo("[Racepak] Serial Port {} Opened...".format(port))

        self.pub_rp_controls = rospy.Publisher('controls', rp_controls, queue_size=10)
        self.pub_rp_shocks = rospy.Publisher('shock_pos', rp_shock_sensors, queue_size=10)
        self.pub_rp_rpm = rospy.Publisher('wheel_rpm', rp_wheel_encoders, queue_size=10)
        self.pub_speed = rospy.Publisher('/vehicle_state/velocity', Odometry, queue_size=10)

        # class variables
        self.channel_num = 0
        self.channel_size_list = []
        self.can_id_list = []
        self.data_type_list = []
        self.data_exponent_list = []

    def start_read(self):
        self.racepak.write(b'\x91')
        rospy.loginfo("[Racepak] Sent Start Command...")

    def stop_read(self):
        self.racepak.write(b'\x92')
        rospy.loginfo("[Racepak] Sent Stop Command...")

    def log_data(self, logfilename, logtime=100): # log data to file for debugging
        logfile = open(logfilename, 'wb')
        bytenum = int(self.baud_rate/8 * logtime)
        for k in range(bytenum):
            logfile.write(self.racepak.read(1))
            if k * 1000 == 0:
                rospy.loginfo("[Racepak] Logged {} bytes into file..".format(k))
        rospy.loginfo("[Racepak] Totally logged {} bytes into file..".format(bytenum))
        logfile.close()

    def update(self):
        rx_bytes = self.racepak.read(1)
        while(rx_bytes != b'\xA0' and rx_bytes != b'\xA1'):
            rx_bytes = self.racepak.read(1) # wait for the header byte
            if len(rx_bytes)==0:
                rospy.logwarn("[Racepak] Error No Data Received while Waiting for Header!")
                return False

        if rx_bytes == b'\xA0': # receive the "Start Telemetry Package"
            rospy.loginfo("[Racepak] Receive Telemetry Starter..")
            payload_size = int.from_bytes(self.racepak.read(2), byteorder='little')
            # check the payload_size: we know when channel_num==22, the payload_size should be 18 + 22*6 = 150
            if payload_size!=150:
                rospy.logwarn("[Racepak] Error Telemetry Starter Playload Size {}, should be 150!".format(payload_size))
                return # discard the sequence and wait for the header byte

            data_raw = self.racepak.read(payload_size)
            checksum = int.from_bytes(self.racepak.read(2), byteorder='little')
            datasum = sum(data_raw)
            if checksum != datasum:
                rospy.logwarn("[Racepak] Error Start Telemetry Checksum {}, but got {}!".format(checksum, datasum))
                return # discard the sequence and wait for the header byte

            self.read_telemetry_header(data_raw[:18]) # 18 bytes
            self.read_channel_header(data_raw[18:]) # 132 bytes

        elif rx_bytes == b'\xA1': # receive the "Telemetry Streaming Package"
            payload_size = int.from_bytes(self.racepak.read(2), byteorder='little')
            if(payload_size != 76):
                rospy.logwarn("[Racepak] Error Data Stream Payload Size {}, should be 76! ".format(payload_size))
                return

            data_raw = self.racepak.read(payload_size)
            checksum_data = int.from_bytes(self.racepak.read(2), byteorder='little')
            datasum = sum(data_raw)
            if checksum_data != datasum:
                rospy.logwarn("[Racepak] Error Data Streaming Checksum {}, but got {}! ".format(checksum, datasum))
                return

            self.read_data_stream(data_raw)

        return True

    def read_telemetry_header(self, data_raw):
        racepak_sn = int.from_bytes(data_raw[0:4], byteorder='little')
        print('    Racepak serial number is ', racepak_sn)

        racepak_txr_1 = int.from_bytes(data_raw[4:8], byteorder='little')
        # print(racepak_txr_1)

        racepak_time_ms = int.from_bytes(data_raw[8:12], byteorder='little')
        print('    Racepak time alive is ', racepak_time_ms, ' ms')

        racepak_txr_2 = int.from_bytes(data_raw[12:16], byteorder='little')
        print('    Racepak TX rate is ', racepak_txr_2, ' Hz')

        self.channel_num = int(data_raw[16])
        print('    Racepak num channels is ', self.channel_num)

        racepak_zer_check = int(data_raw[17])


    def read_channel_header(self, channel_headers):
        self.channel_size_list = []
        self.can_id_list = []
        self.data_type_list = []
        self.data_exponent_list = []

        for i in range(self.channel_num):
            channel_header = channel_headers[i*6:(i+1)*6]
            #Get CAN ID
            can_id_unparsed = channel_header[:2][::-1]
            can_id = can_id_unparsed.hex()
            self.can_id_list.append(can_id)

            # Number given in bits
            channel_headers_size_in_bits = channel_header[2]
            channel_headers_size_in_bytes = int(channel_headers_size_in_bits / 8)
            self.channel_size_list.append(channel_headers_size_in_bytes)

            #Get Data type
            data_type = channel_header[3]
            self.data_type_list.append(data_type)
            
            #Get data exponent
            data_exponent = channel_header[4]
            if data_exponent > 127: 
                data_exponent = (256-data_exponent) * (-1)
            self.data_exponent_list.append(data_exponent)


    def read_data_stream(self, data_raw): 
        # import ipdb;ipdb.set_trace()
        timestamp = rospy.Time.now()
        # This is Telemetry Streaming, data in the order of channel headers
        start_index = 0
        end_index = 0
        shock_data = rp_shock_sensors()
        controls_data = rp_controls()
        rpm_data = rp_wheel_encoders()
        # Add time to header in each message
        controls_data.header.stamp = timestamp
        rpm_data.header.stamp = timestamp
        shock_data.header.stamp = timestamp

        for i in range(self.channel_num):
            end_index = start_index + self.channel_size_list[i]
            # print(end_index)
            channel_data_raw = data_raw[start_index : end_index]
            channel_data = (int.from_bytes(channel_data_raw, byteorder='little'))
            channel_data = channel_data * (10 ** self.data_exponent_list[i])
            start_index = end_index    

            # Sort data into appropriate category
            if(self.can_id_list[i] == '03a4'):
                # Left front shock
                shock_data.front_left = channel_data
            elif(self.can_id_list[i] == '03a5'):
                # Right front shock
                shock_data.front_right = channel_data
            elif(self.can_id_list[i] == '03a6'):
                # Left rear shock
                shock_data.rear_left = channel_data
            elif(self.can_id_list[i] == '03a7'):
                # Right rear shock
                shock_data.rear_right = channel_data
            elif(self.can_id_list[i] == '03b4'):
                # Left front rpm
                rpm_data.front_left = channel_data
            elif(self.can_id_list[i] == '03b5'):
                # Right front rpm
                rpm_data.front_right = channel_data
            elif(self.can_id_list[i] == '03b6'):
                # Left rear rpm
                rpm_data.rear_left = channel_data
            elif(self.can_id_list[i] == '03b7'):
                # Right rear rpm
                rpm_data.rear_right = channel_data
            elif(self.can_id_list[i] == '0421'):
                # Brake
                controls_data.brake = channel_data
            elif(self.can_id_list[i] == '030b'):
                # Throttle
                controls_data.throttle = channel_data

        # Publish all that data
        avg_wheel_speed = (rpm_data.front_left + rpm_data.front_right)/2
        lin_vel = 2.46 * avg_wheel_speed / 60

        speed_msg = Odometry()
        speed_msg.header.stamp = rospy.Time.now()
        speed_msg.twist.twist.linear.y = lin_vel

        self.pub_speed.publish(speed_msg)
        self.pub_rp_controls.publish(controls_data)
        self.pub_rp_rpm.publish(rpm_data)
        self.pub_rp_shocks.publish(shock_data)


if __name__ == '__main__':
    rospy.init_node('racepak_node', anonymous=True)

    racepaknode = RacepakNode()
    racepaknode.start_read() # send starting command
    while not rospy.is_shutdown(): 
        res = racepaknode.update()
        if not res:
            break
    racepaknode.stop_read()

    # # LOG Data to File
    # debugfile = './datafile'
    # racepaknode = RacepakNode(datafilename='')
    # racepaknode.log_data(debugfile)

    # # Debug from Logfile
    # debugfile = './datafile'
    # racepaknode = RacepakNode(datafilename=debugfile)
    # while not rospy.is_shutdown(): 
    #     res = racepaknode.update()
    #     if not res:
    #         break
