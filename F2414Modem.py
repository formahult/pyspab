import serial
import time
import threading
import os

class F2414Modem:

    def send(self, data):
        self.sport.write(b'don')
        time.sleep(2)
        self.sport.write(data)
        return self.sport.read(4096)

    def subscribe(self, handler):
        self.handlers.append(handler)
        return self

    def unsubscribe(self, handler):
        self.handlers.remove(handler)
        return self

    def fire(self, sender, earg=None):
        for handler in self.handlers:
            handler(sender, earg)

    def monitor(self):
       while not self.die:
           data = self.sport.read(self.blocksize)
           if data is not b'':
               self.fire(data)

    def __init__(self, port, baudrate):
        ser = serial.Serial(port)
        self.die = False
        self.blocksize = 1024
        ser.baudrate = baudrate
        ser.timeout = 0.5
        self.sport = ser
        self.handlers = []
        self.thread = threading.Thread(group=None, target=self.monitor, name='mon', args=())
        self.thread.start()

    def __del__(self):
        self.die = True
        self.thread.join()

    __iadd__ = subscribe
    __isub__ = unsubscribe
    __call__ = fire
