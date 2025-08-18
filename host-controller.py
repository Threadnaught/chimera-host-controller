import serial
import time

line_buffer = b''

#rtscts opt here resets the esp on each connection
with serial.Serial('/dev/ttyUSB0', 115200, timeout=0, rtscts=True) as ser:
	while True:
		if ser.inWaiting():
			line_buffer += ser.read(ser.inWaiting())
		if b'\n' in line_buffer:
			lines = line_buffer.split(b'\n')
			if len(lines) > 2:
				line_buffer = lines[-1]
				print(lines[-2].decode('ascii'))