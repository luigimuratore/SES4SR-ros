import serial, time
s = serial.Serial('/dev/ttyUSB1', 230400, timeout=1)
time.sleep(0.5)
s.write(b'\n')
print(s.read(200))
s.close()