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

import serial
from serial import SerialException
from serial.tools import list_ports


class UartInterface():
    @staticmethod
    def setup(vid_pid=None, com_port='', baudrate=921600):

        if com_port:
            return UartInterface(port=com_port, baud=baudrate)
        else:
            devices = UartInterface.enum_device(vid_pid)

            if not devices:
                raise RuntimeError("No UART device found with {}".format(vid_pid))
            else:
                for port in devices:
                    try:
                        return UartInterface(port=port[0], baud=baudrate)
                    except SerialException:
                        continue

        raise RuntimeError("Cound not connect to any device with {}".format(vid_pid))

    def __init__(self, port, baud=1000000, rtscts=True):
        self.mode_blocking = True
        self.blocking_timeout = 2.0
        self.__ser = serial.Serial(port, baud, rtscts=rtscts)
        self.__ser.reset_input_buffer()
        self.__ser.reset_output_buffer()

    def __del__(self):
        try:
            self.close()
        except:
            pass

    def write(self, message):
        try:
            return self.__ser.write(message)
        except Exception:
            raise RuntimeError('Error writing to device')

    def read(self, number_of_bytes):
        try:
            return bytearray(self.__ser.read(number_of_bytes))
        except Exception:
            raise RuntimeError('Error while reading from device')
        

    def set_mode_nonblocking(self):
        if self.mode_blocking:
            # self.__ser.setTimeout(0)
            self.__ser.timeout = 0
            self.mode_blocking = False

    def set_mode_blocking(self):
        if not self.mode_blocking:
            # self.__ser.setTimeout(self.blocking_timeout)
            self.__ser.timeout = self.blocking_timeout
            self.mode_blocking = True

    def get_timeout(self):
        return self.__ser.timeout

    def set_timeout(self, new_timeout):
        prev_timeout = self.__ser.timeout
        self.__ser.timeout = new_timeout
        return prev_timeout

    def close(self):
        self.clear_buffer()
        self.__ser.close()

    def clear_buffer(self):
        self.__ser.reset_input_buffer()
        self.__ser.reset_output_buffer()

    @staticmethod
    def enum_device(vid_pid):
        return list_ports.grep(vid_pid)
