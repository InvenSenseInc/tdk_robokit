#!/usr/bin/env python
#
# Copyright 2019-2020 TDK Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy, sys, time, math
from sensor_msgs.msg import Range, Imu
from redswallow.control import DeviceControl
import redswallow.boards as boards
from redswallow.chsensors import Ch101Sensor

class ROSPublisher(DeviceControl.EventHandler):

    def __init__(self):

        # Publishers
        self.pub_range = rospy.Publisher('/tdk_robokit/range', Range, queue_size=10)
        self.pub_imu = rospy.Publisher('/tdk_robokit/imu', Imu, queue_size=10)

        # ROS Node
        rospy.init_node('tdk_robokit', anonymous=True)

        # Register shutdown callback
        rospy.on_shutdown(self.close_device)

        # Clear Imu raw data buffers
        self.acc_raw_x = 0
        self.acc_raw_y = 0
        self.acc_raw_z = 0
        self.gyr_raw_x = 0
        self.gyr_raw_y = 0
        self.gyr_raw_z = 0

        # Get parameter, COM port for Robokit
        com_port = rospy.get_param('~robokit_com_port', '/dev/ttyUSB0')

        # Connect to Robokit
        self.device = boards.SmartSonic(com_port=com_port, event_handler=self)        
        endtime = time.time() + 5 # 5sec timeout
        while time.time() < endtime:
            try:
                self.device.open()
                break
            except Exception as e:
                time.sleep(0.5)
        if not self.device.openned:
            print("Board not found")
            exit(1)

        # Setup Robokit
        rx_length = 115         # do not change
        self.device.setSRange(min(rx_length, Ch101Sensor()['max_samples']))
        sample_rate_hz = 16.6   # do not change
        self.device.setODR(round(1000 / sample_rate_hz))
        self.device.enableAsicData()
        self.device.enableIQStream()
        self.device.enableImu()
        self.device.startDeviceProcessing()
        print('Board setup successful')

    def ch_data_handler(self, rx_sensor_id, tx_sensor_id, timestamp, idata, qdata, target_detected, range_cm, amplitude):
        # publish Range
        if rx_sensor_id == tx_sensor_id:
            msg = Range()
            msg.header.stamp = rospy.get_rostime()
            msg.header.frame_id = "ch-%d" % tx_sensor_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = math.pi / 2.0
            msg.min_range = 0.14
            msg.max_range = 1.0
            msg.range = range_cm / 100.0
            self.send(self.pub_range, msg)

    def imu_data_handler(self, timestamp, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z):
        # Will be published with Orientation when GRV is available
        # Note: Imu raw and GRV are being updated at the same rate
        self.acc_raw_x = acc_x
        self.acc_raw_y = acc_y
        self.acc_raw_z = acc_z
        self.gyr_raw_x = gyr_x
        self.gyr_raw_y = gyr_y
        self.gyr_raw_z = gyr_z

    def grv_data_handler(self, timestamp, grv_w, grv_x, grv_y, grv_z):
        # publish Imu
        gyr_scale = (500.0 / 32768.0) / 180.0 * math.pi # 500dps FSR
        acc_scale = (4.0 / 32768.0) * 9.81              # 4g FSR
        msg = Imu()
        msg.header.stamp = rospy.get_rostime()
        #msg.header.frame_id = 'base_link' # for test
        msg.angular_velocity.x = self.gyr_raw_x * gyr_scale
        msg.angular_velocity.y = self.gyr_raw_y * gyr_scale
        msg.angular_velocity.z = self.gyr_raw_z * gyr_scale
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.linear_acceleration.x = self.acc_raw_x * acc_scale
        msg.linear_acceleration.y = self.acc_raw_y * acc_scale
        msg.linear_acceleration.z = self.acc_raw_z * acc_scale
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.orientation.w = float(grv_w) / 2**30
        msg.orientation.x = float(grv_x) / 2**30
        msg.orientation.y = float(grv_y) / 2**30
        msg.orientation.z = float(grv_z) / 2**30
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.send(self.pub_imu, msg)

    def exception_handler(self, exception):
        # Error message from lower layer
        print('ROS publisher Exception', str(exception))

    def send(self, pub, msg):
            try:
                if not rospy.is_shutdown():
                    pub.publish(msg)
            except rospy.ROSInterruptException:
                print(''.join((pub.name, 'interrupted')))

    def close_device(self):
        # close device safely
        if self.device.openned:
            print("Closing device ...")
            self.device.disableImu()
            time.sleep(0.1)
            self.device.stopDeviceProcessing()
            time.sleep(0.1)
            self.device.close()
            print("Done")

if __name__ == '__main__':

    ros_publisher = ROSPublisher()
    rospy.spin()
