#!/usr/bin/env python
# Implements a serial port listener and buffer for a datalogger

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Header
from racepak.msg import rp_controls
from racepak.msg import rp_shock_sensors
from racepak.msg import rp_wheel_encoders
from array import array
from struct import *
# import racepak_datatypes
import csv
import sys
import time
#install https://pypi.org/project/pyserial/
import serial

port = '/dev/ttyUSB0'
baud_rate = 57600
parity = serial.PARITY_NONE
data_width = serial.EIGHTBITS
stop_bits = serial.STOPBITS_ONE

# Uncomment to log to csv
# timestr = time.strftime("%Y%m%d-%H%M%S")
# csv_filename = 'racepak_data_' + timestr
# f = open('/home/airlab/racepak/' + csv_filename + '.csv', mode='a')
# writer = csv.writer(f)
# writer.writerow(['TIME','CAN ID','DATA'])

# Start ROS node and topics
rospy.init_node('racepak_handler', anonymous=True)
pub_rp_controls = rospy.Publisher('controls', rp_controls, queue_size=10)
pub_rp_shocks = rospy.Publisher('shock_pos', rp_shock_sensors, queue_size=10)
pub_rp_rpm = rospy.Publisher('wheel_rpm', rp_wheel_encoders, queue_size=10)

#If permission denied https://stackoverflow.com/questions/27858041/oserror-errno-13-permission-denied-dev-ttyacm0-using-pyserial-from-pyth
#Edit all files mentioned and add user to dialout group
racepak = serial.Serial(port,baud_rate, data_width, parity, stop_bits, xonxoff=False)
racepak.flushInput()

# Write 0x91 to start telemetry
racepak.write(b'\x91')

next_byte = b''
data_raw = b''
payload_size = 0
racepak_time_ms = 0
racepak_time_zero = int(round(time.time() * 1000))

#Throw this all in a loop
while not rospy.is_shutdown():

    # Read a line until we find 0xA0
    while True:
        try:
            rx_bytes = racepak.read(1)
            # print(rx_bytes)
            if rx_bytes == b'\xA0':
                # Found start of telemetry header
                # print(rx_bytes.hex())
                # Read next 2 bytes for data payload size
                payload_size = int.from_bytes(racepak.read(2), byteorder='little')
                # print("payload size first is", payload_size)
                # Read the payload size
                racepak_time_zero = int(round(time.time() * 1000))
                data_raw = racepak.read(payload_size)
                # Read 2 byte checksum
                checksum_data = int.from_bytes(racepak.read(2), byteorder='little')
                # Check that the next byte is a command byte (0xA0 or 0xA1) 
                next_byte = racepak.read(1)
                if next_byte == b'\xA0':
                    # Check checksum
                    total = 0
                    for i in range(0,150):
                        total += data_raw[i]
                    if checksum_data == total:
                        print("Checksum doesn't match")
                        # break
                elif next_byte == b'\xA1':
                    # I think it should be 0xA1 every time since 0xA0 is only sent every 5 seconds
                    # Check checksum
                    total = 0
                    for i in range(0,150):
                        total += data_raw[i]
                    if checksum_data == total:
                        # Checksum matches, time to read data
                        break     
                # Dont break the loop if no satisfactory chunk found
        except:
            print('Never found 0xA0')
            break

    # Process Telemetry Header manually the first time
    racepak_sn = int.from_bytes(data_raw[0:3+1], byteorder='little')
    print('Racepak serial number is ', racepak_sn)

    racepak_txr_1 = int.from_bytes(data_raw[4:7+1], byteorder='little')
    # print(racepak_txr_1)

    racepak_time_ms = int.from_bytes(data_raw[8:11+1], byteorder='little')
    print('Racepak time alive is ', racepak_time_ms, ' ms')

    racepak_txr_2 = int.from_bytes(data_raw[12:15+1], byteorder='little')
    print('Racepak TX rate is ', racepak_txr_2, ' Hz')

    racepak_num_channels = int.from_bytes(data_raw[16:16+1], byteorder='little')
    print('Racepak num channels is ',racepak_num_channels)

    racepak_zer_check = int.from_bytes(data_raw[17:17+1], byteorder='little')


    # Process channels manually first time
    # Process each individual channel header
    next_index = 0
    decode_string = ""
    channel_size_list = []
    channel_size_list_2 = []
    can_id_list = []
    data_type_list = []
    data_exponent_list = []
    # Re-zero to make this easier
    channel_headers = data_raw[18:payload_size+1]
    for i in range(0, racepak_num_channels):
        # Keep track of next channel start index by knowing this channel start index and data size
        start_index = next_index

        #Get CAN ID
        can_id_unparsed = channel_headers[start_index + 1:start_index + 2] + channel_headers[start_index + 0:start_index + 1]
        can_id = can_id_unparsed.hex()
        #Add it to a list
        can_id_list.append(can_id)

        # Number given in bits
        channel_headers_size_in_bits = channel_headers[start_index + 2]
        channel_headers_size_in_bytes = int(channel_headers_size_in_bits / 8)
        channel_size_list_2.append(channel_headers_size_in_bytes)

        #Get Data type
        data_type = channel_headers[start_index + 3]
        data_type_list.append(data_type)
        
        #Get data exponent
        data_exponent = channel_headers[start_index + 4]
        if data_exponent > 127:
            data_exponent = (256-data_exponent) * (-1)
        data_exponent_list.append(data_exponent)

        #Set next index for next loop
        next_index = start_index + 6

        # print('can_id = ', can_id_list[i], ' size = ', channel_size_list_2[i], ' datatype = ', data_type_list[i], ' exponent = ', data_exponent_list[i])

    # Time to put this into a loop and check properly
    while True:
        while True:
            # Always read next byte at the end of the loop, should be populated first by proceeding code to sync
            # print("next_byte is = ", next_byte)
            if next_byte == b'\xA1':
                payload_size = int.from_bytes(racepak.read(2), byteorder='little')

                # Until a a better buffer is in place, use this magic number to ensure we are reading the correct A1 byte. 
                # This keeps us from getting too out of sync if we read A1 which is data as a command
                if(payload_size != 76):
                    print("Incorrect payload size of ", payload_size, " bytes.")
                    next_byte = racepak.read(1)
                    break
                # print("payload A1 size is ", payload_size)

                # Record relative time we read data
                channel_time = racepak_time_ms + (int(round(time.time() * 1000)) - racepak_time_zero)

                # Read the payload size
                data_raw = racepak.read(payload_size)
                # Read 2 byte checksum
                checksum_data = int.from_bytes(racepak.read(2), byteorder='little')
                # Check checksum
                total = 0
                for i in range(0,payload_size):
                    total += data_raw[i]
                if checksum_data != total:
                    sys.stdout.flush()
                    print("Checksum doesn't match")
                    break


                    
                # Data must be valid, let's parse it
                # This is Telemetry Streaming, data in the order of channel headers
                
                start_index = 0
                end_index = 0
                shock_data = rp_shock_sensors()
                controls_data = rp_controls()
                rpm_data = rp_wheel_encoders()

                # sys.stdout.flush()
                for i in range(0, racepak_num_channels):
                    end_index = start_index + channel_size_list_2[i]
                    # print(end_index)
                    channel_data_raw = data_raw[start_index : end_index]
                    channel_data = (int.from_bytes(channel_data_raw, byteorder='little'))
                    channel_data = channel_data * (10 ** data_exponent_list[i])
                    # print('Time = ', channel_time, ' can_id = ', can_id_list[i], ' size = ', channel_size_list_2[i], ' datatype = ', data_type_list[i], ' exponent = ', data_exponent_list[i], ' data = ', channel_data)
                    # writer.writerow([channel_time, can_id_list[i],channel_data])
                    start_index = end_index    

                    # Sort data into appropriate category
                    if(can_id_list[i] == '03a4'):
                        # Left front shock
                        shock_data.front_left = channel_data
                    elif(can_id_list[i] == '03a5'):
                        # Right front shock
                        shock_data.front_right = channel_data
                    elif(can_id_list[i] == '03a6'):
                        # Left rear shock
                        shock_data.rear_left = channel_data
                    elif(can_id_list[i] == '03a7'):
                        # Right rear shock
                        shock_data.rear_right = channel_data
                    elif(can_id_list[i] == '03b4'):
                        # Left front rpm
                        rpm_data.front_left = channel_data
                    elif(can_id_list[i] == '03b5'):
                        # Right front rpm
                        rpm_data.front_right = channel_data
                    elif(can_id_list[i] == '03b6'):
                        # Left rear rpm
                        rpm_data.rear_left = channel_data
                    elif(can_id_list[i] == '03b7'):
                        # Right rear rpm
                        rpm_data.rear_right = channel_data
                    elif(can_id_list[i] == '0421'):
                        # Brake
                        controls_data.brake = channel_data
                    elif(can_id_list[i] == '030b'):
                        # Throttle
                        controls_data.throttle = channel_data

                # Add time to header in each message
                controls_data.header.stamp = rospy.Time.now()
                rpm_data.header.stamp = rospy.Time.now()
                shock_data.header.stamp = rospy.Time.now()

                # Publish all that data
                pub_rp_controls.publish(controls_data)
                pub_rp_rpm.publish(rpm_data)
                pub_rp_shocks.publish(shock_data)

                # Check that the next byte is a command byte (0xA0 or 0xA1) 
                next_byte = racepak.read(1)                   


            elif next_byte == b'\xA0':
                payload_size = int.from_bytes(racepak.read(2), byteorder='little')
                # print("payload A0 size is ", payload_size)
                # Read the payload size
                # Until a a better buffer is in place, use this magic number to ensure we are reading the correct A1 byte. 
                # This keeps us from getting too out of sync if we read A1 which is data as a command
                if(payload_size != 150):
                    print("Incorrect payload size of ", payload_size, " bytes.")
                    next_byte = racepak.read(1)
                    break
                # Record the time we read the timestamp
                racepak_time_zero = int(round(time.time() * 1000))

                data_raw = racepak.read(payload_size)
                # Read 2 byte checksum
                checksum_data = int.from_bytes(racepak.read(2), byteorder='little')

                # Check checksum
                total = 0
                for i in range(0,payload_size):
                    total += data_raw[i]
                if checksum_data != total:
                    sys.stdout.flush()
                    print("Checksum doesn't match")
                    break

                # Check that the next byte is a command byte (0xA0 or 0xA1) 
                next_byte = racepak.read(1)
                

                # Data must be valid, let's parse it
                # This is a new Telemetry Start packet, replace all of the exist values

                racepak_sn = int.from_bytes(data_raw[0:3+1], byteorder='little')
                # print('Racepak serial number is ', racepak_sn)

                racepak_txr_1 = int.from_bytes(data_raw[4:7+1], byteorder='little')
                # print(racepak_txr_1)

                racepak_time_ms = int.from_bytes(data_raw[8:11+1], byteorder='little')
                # print(racepak_time_ms)

                racepak_txr_2 = int.from_bytes(data_raw[12:15+1], byteorder='little')
                # print(racepak_txr_2)

                racepak_num_channels = int.from_bytes(data_raw[16:16+1], byteorder='little')
                # print('Racepak num channels is ',racepak_num_channels)

                racepak_zer_check = int.from_bytes(data_raw[17:17+1], byteorder='little')

                # Re-zero to make this easier
                channel_headers = data_raw[18:payload_size+1]
                
                start_index = 0
                
                # print(channel_headers)
                for i in range(0, racepak_num_channels):
                    #Get CAN ID
                    can_id_unparsed = channel_headers[start_index + 1:start_index + 2] + channel_headers[start_index + 0:start_index + 1]
                    can_id = can_id_unparsed.hex()
                    #Add it to a list
                    can_id_list.append(can_id)

                    # Number given in bits
                    # print("i is ", i, " and index is ", start_index)
                    channel_headers_size_in_bits = channel_headers[start_index + 2]
                    channel_headers_size_in_bytes = int(channel_headers_size_in_bits / 8)
                    channel_size_list_2.append(channel_headers_size_in_bytes)

                    #Get Data type
                    data_type = channel_headers[start_index + 3]
                    data_type_list.append(data_type)
                    
                    #Get data exponent
                    data_exponent = channel_headers[start_index + 4]
                    if data_exponent > 127:
                        data_exponent = (256-data_exponent) * (-1)
                    data_exponent_list.append(data_exponent)

                    
                    # print('can_id = ', can_id_list[i], ' size = ', channel_size_list_2[i], ' datatype = ', data_type_list[i], ' exponent = ', data_exponent_list[i], ' data = ', channel_data)

                    #Set start_index for next loop
                    start_index = start_index + 6

                    next_byte = racepak.read(1)

            else:
                # Keep reading until you find something!
                next_byte = racepak.read(1)


    
