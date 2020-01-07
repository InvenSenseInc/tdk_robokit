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

import struct

SUPPORTED_PROTOCOL_VERSION = 3

class PacketType:
	RESERVED                      = 0x00
	
	#/* COMMAND packets (HOST -> DEVICE) */
	CMD_GET_PROTOCOL_VERSION      = 0x01
	CMD_SOFT_RESET                = 0x02
	CMD_RESET_SENSOR              = 0x03
	CMD_RESET_PDALGO              = 0x04
	CMD_ENABLE_PDALGO             = 0x05
	CMD_ENABLE_IQSTREAM           = 0x06
	CMD_START                     = 0x07
	CMD_STOP                      = 0x08
	CMD_ENABLE_ASIC_DATA          = 0x09
	CMD_ENABLE_IMU                = 0x0a

	CMD_GET_SENSOR_PARAM          = 0x20
	CMD_GET_VERSION               = 0x21
	CMD_GET_SENSORS               = 0x22
	CMD_GET_ALGO_CONFIG           = 0x23
	CMD_GET_STATUS                = 0x24

	CMD_SET_ALGO_CONFIG           = 0x30
	CMD_SET_ODR                   = 0x31
	CMD_SET_RANGE_MM              = 0x32
	CMD_SET_SAMPLE_RANGE          = 0x33
	CMD_SET_PULSE_LENGTH          = 0x34
	CMD_SET_LOW_GAIN_RXLEN        = 0x35

	#/* RESPONSE packets (DEVICE -> HOST) */
	RESP_GET_PROTOCOL_VERSION     = 0x81
	RESP_SOFT_RESET               = 0x82
	RESP_RESET_SENSOR             = 0x83
	RESP_RESET_PDALGO             = 0x84
	RESP_ENABLE_PDALGO            = 0x85
	RESP_ENABLE_IQSTREAM          = 0x86
	RESP_START                    = 0x87
	RESP_STOP                     = 0x88
	RESP_ENABLE_ASIC_DATA         = 0x89
	RESP_ENABLE_IMU               = 0x8a

	RESP_GET_SENSOR_PARAM         = 0xa0
	RESP_GET_VERSION              = 0xa1
	RESP_GET_SENSORS              = 0xa2
	RESP_GET_ALGO_CONFIG          = 0xa3
	RESP_GET_STATUS               = 0xa4

	RESP_SET_ALGO_CONFIG          = 0xb0
	RESP_SET_ODR                  = 0xb1
	RESP_SET_RANGE_MM             = 0xb2
	RESP_SET_SAMPLE_RANGE         = 0xb3
	RESP_SET_PULSE_LENGTH         = 0xb4
	RESP_SET_LOW_GAIN_RXLEN       = 0xb5

	#/* ASYNCRONOUS packets (DEVICE -> HOST) */
	ASYNC_IMU_DATA                = 0xf9
	ASYNC_CH_DATA                 = 0xfa
	ASYNC_PD_OUT                  = 0xfb
	ASYNC_STATUS                  = 0xfc
	ASYNC_DEBUG_MESSAGE           = 0xfd
	ASYNC_GRV_DATA                = 0xfe

class PacketDecoder():
	HEADER = (0x55, 0xaa)
	FOOTER = (0xbe, 0xef)
	def __init__(self):
		self.head = 0
		self.size = 0

		self.decoders = {
			PacketType.RESP_GET_PROTOCOL_VERSION: (self.generic_decoder, (
				('protocol_version', 'B', 1, 1, True),
			)),
			PacketType.RESP_GET_VERSION: (self.get_version_decoder, None),
			PacketType.ASYNC_IMU_DATA: (self.generic_decoder, (
				('timestamp', 'Q', 8, 1, True),
				('acc_x', 'h', 2, 1, True),
				('acc_y', 'h', 2, 1, True),
				('acc_z', 'h', 2, 1, True),
				('gyr_x', 'h', 2, 1, True),
				('gyr_y', 'h', 2, 1, True),
				('gyr_z', 'h', 2, 1, True),
			)),
			PacketType.ASYNC_CH_DATA: (self.chdata_decoder, (
				('target_detected', 'I', 4, 1, True),
				('range_cm', 'I', 4, 1, True),
				('amplitude', 'I', 4, 1, True),
			)),
			PacketType.ASYNC_DEBUG_MESSAGE: (self.dmsg_decoder, None),
			PacketType.ASYNC_PD_OUT: (self.generic_decoder, (
				('sensor_id', 'B', 1, 1, True),
				('timestamp', 'Q', 8, 1, True),
				('presence', 'i', 4, 1, True),
				('rmin', 'i', 4, 1, True),
				('rmax', 'i', 4, 1, True),
				('score', 'f', 4, 1, True),
			)),
			PacketType.ASYNC_STATUS: (self.generic_decoder, (
				('timestamp', 'Q', 8, 1, True),
				('status', 'i', 4, 1, True),
				('error_count', 'i', 4, 1, True),
			)),
			PacketType.RESP_GET_SENSORS: (self.get_connected_sensors, None),
			PacketType.RESP_GET_SENSOR_PARAM: (self.generic_decoder, (
				('sensor_id', 'L', 4, 1, True),
				('op_freq_hz', 'L', 4, 1, True),
				('nb_samples', 'L', 4, 1, True),
				('odr_ms', 'L', 4, 1, True),
				('range_mm', 'L', 4, 1, True),
				('pulse_length', 'L', 4, 1, True),
				('low_gain_rxlen', 'L', 4, 1, True),
			)),
			PacketType.RESP_GET_ALGO_CONFIG: (self.generic_decoder, (
				('sensor_id', 'L', 4, 1, True),
				('odr_us', 'L', 4, 1, True),
				('nb_samples', 'L', 4, 1, True),
				('max_range', 'L', 4, 1, True),
				('min_range', 'L', 4, 1, True),
				('op_freq_hz', 'L', 4, 1, True),
				('sensitivity', 'L', 4, 1, True),
				('range_offset', 'L', 4, 1, True),
				('range_nb_zones', 'L', 4, 1, True),
				('range_hysteresis', 'L', 4, 1, True),
				('range_interval', 'L', 4, 1, True),
			)),
			PacketType.RESP_GET_STATUS: (self.array_decoder, (
				('status', 'B', 1, 1, True),
			)),
			PacketType.ASYNC_GRV_DATA: [self.generic_decoder, (
				('timestamp', 'Q', 8, 1, True),
				('grv_w', 'i', 4, 1, True),
				('grv_x', 'i', 4, 1, True),
				('grv_y', 'i', 4, 1, True),
				('grv_z', 'i', 4, 1, True),
			)],
			PacketType.RESP_SOFT_RESET: (self.simple_acknowledge, None ),
			PacketType.RESP_RESET_SENSOR: (self.simple_acknowledge, None ),
			PacketType.RESP_RESET_PDALGO: (self.simple_acknowledge, None ),
			PacketType.RESP_ENABLE_PDALGO: (self.simple_acknowledge, None ),
			PacketType.RESP_ENABLE_IQSTREAM: (self.simple_acknowledge, None ),
			PacketType.RESP_ENABLE_IMU: (self.simple_acknowledge, None ),
			PacketType.RESP_ENABLE_ASIC_DATA: (self.simple_acknowledge, None ),
			PacketType.RESP_START: (self.simple_acknowledge, None ),
			PacketType.RESP_STOP: (self.simple_acknowledge, None ),
			PacketType.RESP_SET_ALGO_CONFIG: (self.simple_acknowledge, None ),
			PacketType.RESP_SET_ODR: (self.simple_acknowledge, None ),
			PacketType.RESP_SET_RANGE_MM: (self.simple_acknowledge, None ),
			PacketType.RESP_SET_SAMPLE_RANGE: (self.simple_acknowledge, None ),
			PacketType.RESP_SET_PULSE_LENGTH: (self.simple_acknowledge, None ),
			PacketType.RESP_SET_LOW_GAIN_RXLEN: (self.simple_acknowledge, None ),
		}

	@property
	def rsize(self):
		return (self.size - self.head)

	def update(self, buffer_in, pkt_handler):
		if self.rsize:
			buffer = self.buffer[self.head:] + buffer_in
		else:
			buffer = buffer_in

		#print('Redswallow:RX buffer> ', ' '.join(['{:02x}'.format(b) for b in buffer]))

		self.buffer = buffer
		self.head = 0
		self.size = len(self.buffer)

		while self.parseInputBuffer(pkt_handler) > 0:
			pass

	def reset(self):
		self.head = 0
		self.size = 0

	def parseInputBuffer(self, pkt_handler):
		FRAME_MIN_SIZE = 2+2+2  # Header, size, footer
		head0 = self.head

		if self.rsize >= FRAME_MIN_SIZE:
			if (tuple(self.buffer[self.head+0:self.head+2]) != self.HEADER):
				print("Invalid input frame. HEADER missmatch.")
				while(self.head != self.size):
					if(tuple(self.buffer[self.head+0:self.head+2]) != self.HEADER):
						self.head += 1
					else:
						break
				if(self.head == self.size):
					return -1

			frameLen = self.buffer[self.head+2] | (self.buffer[self.head+3] << 8)
			psize = self.head+4+frameLen

			if self.rsize >= (FRAME_MIN_SIZE+frameLen):
				if (tuple(self.buffer[self.head+frameLen+4:self.head+frameLen+6]) != self.FOOTER):
					print("Invalid input frame. FOOTER missmatch.")
					self.head = self.size
					return -1
				else:
					self.head += 4
					while (psize-self.head) > 0:
						if self.parseInputPacket(pkt_handler) <= 0:
							break
					self.head += 2

		return self.head - head0

	def parseInputPacket(self, pkt_handler):
		head0 = self.head
		if self.rsize <= 0:
			return 0

		command_code = self.buffer[self.head]
		self.head += 1

		decoder, args = self.decoders.get(command_code, (self.unknown_packet_decoder, command_code))
		ret_dict = decoder(args)

		pkt_handler(command_code, ret_dict)

		return self.head - head0

	def simple_acknowledge(self, *args):
		status = self.buffer[self.head]
		self.head += 1
		return { 'status': status }

	def generic_decoder(self, decoding_descriptor):
		ret_dict = {}
		for key, type, word_size, size, scalar in decoding_descriptor:
			value = [struct.unpack_from(
				'<' + type, self.buffer, offset=self.head + i * word_size)[0] for i in range(size)]
			self.head += size * word_size
			if key != '_':
				ret_dict[key] = value[0] if size == 1 and scalar else value
		return ret_dict

	def dmsg_decoder(self, decoding_descriptor):
		data_size = self.buffer[self.head]
		ret_dict = {'s': self.buffer[self.head+1:self.head+1+data_size]}
		self.head += 1 + data_size
		return ret_dict

	def array_decoder(self, decoding_descriptor):
		ret_dict = {}
		
		key, type, word_size, _, _ = decoding_descriptor[0]
		size = self.buffer[self.head]

		ret_dict[key] = [struct.unpack_from('<' + type, self.buffer, offset=self.head+1+i*word_size)[0] for i in range(size)]

		self.head += size + 1
		return ret_dict

	def chdata_decoder(self, asic_decoding_descriptor):
		rx_sensor_id = struct.unpack("<B", self.buffer[self.head+0: self.head+1])[0]
		tx_sensor_id = struct.unpack("<B", self.buffer[self.head+1: self.head+2])[0]
		timestamp = struct.unpack("<Q", self.buffer[self.head+2: self.head+10])[0]
		flags = struct.unpack("<B", self.buffer[self.head+10: self.head+11])[0]
		self.head += 11
		
		if flags & 0x01:
			data_size = struct.unpack("<h", self.buffer[self.head+0: self.head+2])[0]
			iq_bytes = self.buffer[self.head+2:self.head+2+data_size]
			qdata = [struct.unpack("<h", x[0:2])[0] for x in [iq_bytes[pos:pos + 4] for pos in range(0, len(iq_bytes), 4)]]
			idata = [struct.unpack("<h", x[2:4])[0] for x in [iq_bytes[pos:pos + 4] for pos in range(0, len(iq_bytes), 4)]]
			self.head += 2 + data_size
		else:
			qdata = []
			idata = []

		if flags & 0x02:
			ret_dict = self.generic_decoder(asic_decoding_descriptor)
		else:
			ret_dict = {key: -1 for key, _, _, _, _ in asic_decoding_descriptor if key != '_'}

		if ret_dict['target_detected'] == 0:
			ret_dict['range_cm'] = 0
			ret_dict['amplitude'] = 0
		else:
			ret_dict['range_cm'] = round(ret_dict['range_cm'] / 32 / 10, 1)

		ret_dict.update({
			'rx_sensor_id': rx_sensor_id,
			'tx_sensor_id': tx_sensor_id,
			'timestamp': timestamp,
			'idata': idata,
			'qdata': qdata,
		})

		return ret_dict

	def get_version_decoder(self, decoding_descriptor):
		nb_version, = struct.unpack_from('B', self.buffer, self.head)
		self.head += 1

		ret_dict = {}

		for i in range(nb_version):
			slabel, sversion = struct.unpack_from('<BB', self.buffer, self.head)
			self.head += 2
			label, version = struct.unpack_from('<{}sx{}sx'.format(slabel-1, sversion-1), self.buffer, self.head)
			self.head += slabel + sversion
			ret_dict[label.decode('ascii')] = version.decode('ascii')

		return ret_dict

	def get_connected_sensors(self, decoding_descriptor):
		nb_sensors, = struct.unpack_from('<B', self.buffer, self.head)
		ret_dict = {'nb_sensors': nb_sensors }
		ret_dict['sensor_ids'] = struct.unpack_from(
			'{}B'.format(nb_sensors), self.buffer, self.head+1)
		self.head += (nb_sensors + 1)
		return ret_dict

	def unknown_packet_decoder(self, command_code):
		print("Unknown packet", hex(command_code))
		self.head = self.size - 2


class PacketFormater:
	@staticmethod
	def format_cmd_packet(cmd, cmd_args_code="", *cmd_args):
		agrs_size = struct.calcsize("<B" + cmd_args_code)
		lcmd = [0x55, 0xaa, agrs_size, cmd ] + list(cmd_args) + [0xbe, 0xef]
		bcmd = struct.pack("<BBHB" + cmd_args_code + "BB", *lcmd)
		return bcmd
