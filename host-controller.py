import serial
import time
import datetime
import numpy as np

line_buffer = b''

base_quats = None
current_quats = None

get_base_at = datetime.datetime.now() + datetime.timedelta(seconds=10)

#rtscts opt here resets the esp on each connection
with serial.Serial('/dev/ttyUSB0', 115200, timeout=0, rtscts=True) as ser:
	
	while True:
		if ser.inWaiting():
			line_buffer += ser.read(ser.inWaiting())
		if b'\n' in line_buffer:
			lines = line_buffer.split(b'\n')
			if len(lines) > 2:
				line_buffer = lines[-1]

				most_recent_full_line = lines[-2].decode('ascii')


				most_recent_split = most_recent_full_line.strip(', \r\n').split(', ')

				if most_recent_full_line.count(',') < 12:
					print('skipping due to partial line:', most_recent_full_line)
					continue
				
				current_quats = np.asarray([float(x) for x in most_recent_split]).reshape([3,4])
				if base_quats is None and datetime.datetime.now() > get_base_at:
					base_quats = current_quats
				# print(current_quats)
				if not base_quats is None:
					dot_product = np.dot(base_quats[0], current_quats[0])
					print('leftarm angle moved since start:', np.arccos(np.clip(dot_product, -1.0, 1.0)) * 360 / (np.pi))
