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

from .packet import SUPPORTED_PROTOCOL_VERSION
from .packet import PacketType
from .packet import PacketDecoder
from .packet import PacketFormater

from .uart_interface import UartInterface
from .chsensors import *

from .constants import SPEED_OF_SOUND

import time
import threading
import sys
if sys.version_info >= (3, 0):
	import queue
	get_tick = time.monotonic
else:
	import Queue as queue
	get_tick = time.time

import copy
import functools
import struct

class PulseConverter:
	@staticmethod
	def samples2us(samples, fop):
		return samples * 1e6 / fop
	@staticmethod
	def us2samples(t_us, fop):
		return int(round(t_us / 1e6 * fop))
	@staticmethod
	def samples2cm(samples, fop):
		return samples / fop * SPEED_OF_SOUND * 100.0

class PollerThread(threading.Thread):
	def __init__(self, do_process, poll_interval=30.0):
		super(PollerThread, self).__init__(name="_DeviceControlPollerThread")
		self.exit_flag = threading.Event()
		self.poll_interval_sec = poll_interval / 1000.0
		self.do_process = do_process

	def run(self):
		while self.exit_flag.is_set() is False:
			start = get_tick()
			self.do_process()
			delta = self.poll_interval_sec - (get_tick() - start)
			if delta > 0:
				time.sleep(delta)

	def stop(self):
		self.exit_flag.set()


class DeviceControl(object):
	"""
	Public interface for controlling the device.
	Handlers can be set to process the data from the board.
	"""

	class EventHandler(object):
		"""
		Subclass it to catch event and from the device.
		"""

		def ch_data_handler(self, rx_sensor_id, tx_sensor_id, timestamp, idata, qdata, target_detected, range_cm, amplitude):
			pass

		def pd_data_handler(self, sensor_id, timestamp, presence, rmin, rmax, score):
			pass

		def imu_data_handler(self, timestamp, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z):
			pass

		def grv_data_handler(self, timestamp, grv_w, grv_x, grv_y, grv_z):
			pass

		def device_msg_handler(self, s):
			pass

		def status_handler(self, timestamp, status, error_count):
			pass

		def exception_handler(self, exception):
			pass

	def __init__(self, vid_pid=None, com_port='', baudrate=1000000, event_handler=EventHandler()):
		self.vid_pid = vid_pid
		self.com_port = com_port
		self.baudrate = baudrate
		self.serial = None
		self.openned = False
		self.packet_decoder = PacketDecoder()
		self.init_status = None
		self.init_done_event = threading.Event()
		self.event_handlers = {
			PacketType.ASYNC_CH_DATA: functools.partial(self._user_event_handler, event_handler.ch_data_handler),
			PacketType.ASYNC_PD_OUT: functools.partial(self._user_event_handler, event_handler.pd_data_handler),
			PacketType.ASYNC_IMU_DATA: functools.partial(self._user_event_handler, event_handler.imu_data_handler),
			PacketType.ASYNC_GRV_DATA: functools.partial(self._user_event_handler, event_handler.grv_data_handler),
			PacketType.ASYNC_STATUS: functools.partial(self._user_event_handler, event_handler.status_handler),
			PacketType.ASYNC_DEBUG_MESSAGE: functools.partial(self._user_event_handler, event_handler.device_msg_handler),
			PacketType.RESP_ENABLE_IQSTREAM: functools.partial(self._resp_handler, PacketType.RESP_ENABLE_IQSTREAM),
			PacketType.RESP_ENABLE_PDALGO: functools.partial(self._resp_handler, PacketType.RESP_ENABLE_PDALGO),
			PacketType.RESP_ENABLE_IMU: functools.partial(self._resp_handler, PacketType.RESP_ENABLE_IMU),
			PacketType.RESP_ENABLE_ASIC_DATA: functools.partial(self._resp_handler, PacketType.RESP_ENABLE_ASIC_DATA),
			PacketType.RESP_GET_ALGO_CONFIG: functools.partial(self._resp_handler, PacketType.RESP_GET_ALGO_CONFIG),
			PacketType.RESP_GET_PROTOCOL_VERSION: functools.partial(self._resp_handler, PacketType.RESP_GET_PROTOCOL_VERSION),
			PacketType.RESP_GET_SENSOR_PARAM: functools.partial(self._resp_handler, PacketType.RESP_GET_SENSOR_PARAM),
			PacketType.RESP_GET_SENSORS: functools.partial(self._resp_handler, PacketType.RESP_GET_SENSORS),
			PacketType.RESP_GET_VERSION: functools.partial(self._resp_handler, PacketType.RESP_GET_VERSION),
			PacketType.RESP_GET_STATUS: functools.partial(self._resp_handler, PacketType.RESP_GET_STATUS),
			PacketType.RESP_SOFT_RESET: functools.partial(self._resp_handler, PacketType.RESP_SOFT_RESET),
			PacketType.RESP_RESET_PDALGO: functools.partial(self._resp_handler, PacketType.RESP_RESET_PDALGO),
			PacketType.RESP_RESET_SENSOR: functools.partial(self._resp_handler, PacketType.RESP_RESET_SENSOR),
			PacketType.RESP_SET_ALGO_CONFIG: functools.partial(self._resp_handler, PacketType.RESP_SET_ALGO_CONFIG),
			PacketType.RESP_SET_ODR: functools.partial(self._resp_handler, PacketType.RESP_SET_ODR),
			PacketType.RESP_SET_RANGE_MM: functools.partial(self._resp_handler, PacketType.RESP_SET_RANGE_MM),
			PacketType.RESP_SET_SAMPLE_RANGE: functools.partial(self._resp_handler, PacketType.RESP_SET_SAMPLE_RANGE),
			PacketType.RESP_SET_PULSE_LENGTH: functools.partial(self._resp_handler, PacketType.RESP_SET_PULSE_LENGTH),
			PacketType.RESP_SET_LOW_GAIN_RXLEN: functools.partial(self._resp_handler, PacketType.RESP_SET_LOW_GAIN_RXLEN),
			PacketType.RESP_START: functools.partial(self._resp_handler, PacketType.RESP_START),
			PacketType.RESP_STOP: functools.partial(self._resp_handler, PacketType.RESP_STOP),
		}
		self.exception_handler = event_handler.exception_handler
		self.poller_thread = None
		self.worker_thread = None
		self.worker_queue = None
		self.response_event = threading.Event()
		self.response_pkt = None
		self.response_data = None

	def __del__(self):
		self.close()

	def open(self, **knargs):
		try:
			self._connect()
			self.checkProtocolVersion()
			self.reset()
			self.checkStatus()
			self.connected_sensors = self._sync_bin_command(PacketType.CMD_GET_SENSORS)
			self.openned = True
		except Exception as e:
			self._disconnect()
			raise e

	def close(self):
		try:
			self.reset()
		except:
			pass
		finally:
			self.openned = False
			self._disconnect()
	
	def reset(self):
		resp = self._sync_bin_command(PacketType.CMD_SOFT_RESET, timeout=6)
		if resp['status'] != 0:
			raise Exception("Failed to initialize the board after reset")


	def checkProtocolVersion(self):
		resp_data = self._sync_bin_command(PacketType.CMD_GET_PROTOCOL_VERSION)
		if resp_data['protocol_version'] != SUPPORTED_PROTOCOL_VERSION:
			raise Exception("Protocol version missmatch (board supports version {}, expected version {})".format(
					resp_data['protocol_version'], SUPPORTED_PROTOCOL_VERSION))

	def checkStatus(self):
		status = self._sync_bin_command(PacketType.CMD_GET_STATUS)['status']
		if status[0] != 0:
			#raise Exception(f'Failed to initialize the board, status=0x{status[0]:02x}')
			raise Exception('Failed to initialize the board, status=' + str(hex(status[0])))

	def enableIQStream(self):
		self._sync_bin_command(PacketType.CMD_ENABLE_IQSTREAM, "B", args_tuple = (1,))

	def disableIQStream(self):
		self._sync_bin_command(PacketType.CMD_ENABLE_IQSTREAM, "B", args_tuple = (0,))

	def enableImu(self):
		self._sync_bin_command(PacketType.CMD_ENABLE_IMU, "B", args_tuple = (1,))

	def disableImu(self):
		self._sync_bin_command(PacketType.CMD_ENABLE_IMU, "B", args_tuple = (0,))

	def enableAsicData(self):
		self._sync_bin_command(PacketType.CMD_ENABLE_ASIC_DATA, "B", args_tuple = (1,))

	def disableAsicData(self):
		self._sync_bin_command(PacketType.CMD_ENABLE_ASIC_DATA, "B", args_tuple = (0,))

	def getSensorConfig(self, sensor_id):
		return self._sync_bin_command(PacketType.CMD_GET_SENSOR_PARAM, 'B', args_tuple = (sensor_id,))

	def getAlgoConfig(self, sensor_id):
		return self._sync_bin_command(PacketType.CMD_GET_ALGO_CONFIG, "B", args_tuple = (sensor_id,))

	def setAlgoConfig(self, algo_config):
		resp = self._sync_bin_command(command=PacketType.CMD_SET_ALGO_CONFIG, code="I"*11, args_tuple=(int(algo_config['sensor_id']),
			int(algo_config['odr_us']), int(algo_config['nb_samples']), int(algo_config['max_range']), 
			int(algo_config['min_range']), int(algo_config['op_freq_hz']), int(algo_config['sensitivity']),
			int(algo_config['range_offset']), int(algo_config['range_nb_zones']), 
			int(algo_config['range_hysteresis']), int(algo_config['range_interval'])))
		#print("Redswallow:SetConfig> ", algo_config)
		if resp['status'] != 0:
			raise Exception("Failed to set algo config for sensor {}".format(algo_config['sensor_id']))

	def setRange(self, range_mm, sensor_id=None):
		sensor_ids = self.connected_sensors['sensor_ids'] if sensor_id is None else list(sensor_id)
		for sensor_id in sensor_ids:
			self._sync_bin_command(command=PacketType.CMD_SET_RANGE_MM, code="BI", args_tuple=(sensor_id, int(range_mm)))
			algo_config = self.getAlgoConfig(sensor_id)
			sensor_config = self.getSensorConfig(sensor_id)
			algo_config['nb_samples'] = sensor_config['nb_samples']
			self.setAlgoConfig(algo_config)

	def setSRange(self, nsamples, sensor_id=None):
		sensor_ids = self.connected_sensors['sensor_ids'] if sensor_id is None else list(sensor_id)
		for sensor_id in sensor_ids:
			self._sync_bin_command(command=PacketType.CMD_SET_SAMPLE_RANGE, code="BI", args_tuple=(sensor_id, int(nsamples)))
			algo_config = self.getAlgoConfig(sensor_id)
			sensor_config = self.getSensorConfig(sensor_id)
			algo_config['nb_samples'] = sensor_config['nb_samples']
			self.setAlgoConfig(algo_config)

	def startDeviceProcessing(self):
		self._sync_bin_command(PacketType.CMD_START)

	def stopDeviceProcessing(self):
		self._sync_bin_command(PacketType.CMD_STOP)

	def setODR(self, odr_ms):
		for sensor_id in self.connected_sensors['sensor_ids']:
			resp = self._sync_bin_command(PacketType.CMD_SET_ODR, "BI",  args_tuple=(sensor_id, int(odr_ms)))
			if resp['status'] != 0:
				raise Exception("Failed to set ODR for sensor {}".format(sensor_id))
			algo_config = self.getAlgoConfig(sensor_id)
			algo_config['odr_us'] = 1000 * odr_ms
			self.setAlgoConfig(algo_config)

	def getInfo(self):
		device_info = {
			'versions': {},
			'nb_sensors': 0,
			'sensors': {}
		}

		device_info['versions'] = self._sync_bin_command(PacketType.CMD_GET_VERSION)
		connected_sensors = self._sync_bin_command(PacketType.CMD_GET_SENSORS)
		device_info['nb_sensors'] = connected_sensors['nb_sensors']

		for sensor_id in connected_sensors['sensor_ids']:
			sensor_config = self.getSensorConfig(sensor_id)
			sensor_config['pulse_length_us'] = round(PulseConverter.samples2us(sensor_config['pulse_length'],
				sensor_config['op_freq_hz']), 2)
			device_info['sensors'][sensor_id] = {k: v for k, v in sensor_config.items() if k != 'sensor_id' }

		if 'ch101fw' in device_info['versions']:
			device_info['chsensor'] = Ch101Sensor()
		elif 'ch201fw' in device_info['versions']:
			device_info['chsensor'] = Ch201Sensor()
		else:
			device_info['chsensor'] = ChSensor()

		return device_info

	def _connect(self):
		self._disconnect()
		self.packet_decoder.reset()
		self.serial = UartInterface.setup(self.vid_pid, self.com_port, self.baudrate)
		self.worker_queue = queue.Queue()
		self.worker_thread = threading.Thread(target=self._user_event_worker, name="DeviceControlWorkerThread")
		self.worker_thread.start()
		self.poller_thread = PollerThread(do_process=self._read_from_serial, poll_interval=30)
		self.poller_thread.start()
		time.sleep(0.5) # Wait for FTDI setup

	def _disconnect(self):
		if self.poller_thread:
			self.poller_thread.stop()
			self.poller_thread.join()
			self.poller_thread = None

		if self.worker_queue:
			self.worker_queue.put(None)
			self.worker_queue.join()
			self.worker_queue = None

		if self.worker_thread:
			self.worker_thread.join()
			self.worker_thread = None

		try:
			self.serial.close()
			del self.serial
		except Exception:
			pass

	def _send_bin_command(self, command, code="", args_tuple = ()):
		bcmd = PacketFormater.format_cmd_packet(command, code, *args_tuple)
		#print('Redswallow:TX buffer> ', ' '.join(['{:02x}'.format(b) for b in bcmd]))
		return self.serial.write(bcmd)

	def _sync_bin_command(self, command, code="", timeout=2, nb_retry=0, args_tuple = ()):
		expected_pkt = command | 0x80

		while True:
			try:
				self.response_pkt = None
				self.response_event.clear()
				self.response_data = None
				self._send_bin_command(command, code, args_tuple)
				if not self.response_event.wait(timeout):
					raise Exception("Timeout after sending command 0x{:02x}".format(command))
				if expected_pkt and self.response_pkt != expected_pkt:
					raise Exception("Unexpected response. Receive 0x{:02x} instead of 0x{:02x}".format(expected_pkt, self.response_pkt))
			except Exception as e:
				if nb_retry == 0:
					raise e
				nb_retry -= 1
			else:
				break
		return copy.deepcopy(self.response_data)

	def _read_from_serial(self):
		try:
			self.serial.set_mode_nonblocking()
			payload = self.serial.read(4096)
			if len(payload) > 0:
				self.packet_decoder.update(payload, self._pkt_handler)
		except Exception as e:
			self.worker_queue.put(self.exception_handler, e)
			raise e

	def _pkt_handler(self, pkt_id, pkt_data):
		handler = self.event_handlers.get(pkt_id, None)
		if handler:
			handler(pkt_data)

	def _init_done_handler(self, event_data):
		if not self.init_done_event.is_set():
			self.init_status = event_data['status']
			self.init_done_event.set()
	
	def _resp_handler(self, event_id, event_data):
		if not self.response_event.is_set():
			if event_id >= 0x81 and event_id < 0xe0: #If it's a response packet (not an asynchrone one) 
				self.response_pkt = event_id
				self.response_data = event_data
				self.response_event.set()

	def _user_event_handler(self, user_event_cb, event_data):
		self.worker_queue.put((user_event_cb, event_data, ))

	def _user_event_worker(self):
		exit_loop = False
		while not exit_loop:
			item = self.worker_queue.get()
			if item is None:
				exit_loop = True
			else:
				try:
					item[0](**item[1])
				except Exception as e:
					self.exception_handler(e)
			self.worker_queue.task_done()

