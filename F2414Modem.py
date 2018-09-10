import serial
import time

class Modem:
    def __init__(self, port, baudrate):
        ser = serial.Serial(port)
        ser.baudrate = baudrate
        ser.timeout = 10
        self.sport = ser

    def send(self, data):
        self.sport.write(b'don')
        time.sleep(2)
        self.sport.write(data)
        return self.sport.read(4096)
