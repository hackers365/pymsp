import serial
import struct
import sys
import time
print sys.getdefaultencoding()


ser = serial.Serial('/dev/ttyUSB0', 115200)

print ser.write(struct.pack('<3c3B', '$', 'M', '<', 0, 105, 105))
ser.flush()
time.sleep(1)
print ser.inWaiting()
