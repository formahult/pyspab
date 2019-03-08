from picamera import PiCamera
import time
import base64


class Sensors:

    def __init__(self):
        self.camera = PiCamera()
        self.camera.resolution = (1024, 768)
        self.camera.start_preview()
        time.sleep(2)
        self.base64string = None

    def capture(self, filename):
        self.camera.capture(filename)

    def convert(self, filename):
        image = open(filename, 'rb')
        image_read = image.read()
        image_64_encode = base64.encodebytes(image_read)
        self.base64string = image_64_encode




