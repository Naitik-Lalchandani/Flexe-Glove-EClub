import serial
import struct
import time
import sys

def read_exactly(ser, n):
    buf = b''
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise Exception("Timeout reading serial")
        buf += chunk
    return buf
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=5)
ser.flush()

try:
    while True:
        data = read_exactly(ser, 16)  # 4 floats * 4 bytes each = 16 bytes
        gx, gy, gz, marker = struct.unpack('ffff', data)
        if abs(marker - 1111.0) < 0.0001:
            output = f"Gyro: X={gx:.4f}, Y={gy:.4f}, Z={gz:.4f}"
        else:
            output = "Invalid packet marker, discarding data"

        # Clear previous line completely
        sys.stdout.write('\r' + ' ' * 80 + '\r')  # Clear and return to start
        sys.stdout.write(output)
        sys.stdout.flush()

        time.sleep(0.005)

except KeyboardInterrupt:
    ser.close()
