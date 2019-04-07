from ds18b20 import DS18B20
from time import sleep
# from picamera import PiCamera
import base64
import sched
import time
import SpabModel
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn


class SensorManager:
    def __init__(self, readings):
        self.readings = readings
        self.temp_sensor = DS18B20()
        self.temps = None
        self.ads = ADS.ADS1115
        self.chan = AnalogIn(self.ads, ADS.P0)
        self.conductivity_v = None
        self.conductivity = None

    def get_temp(self):
        #print("get_temp")
        self.temps = self.temp_sensor.get_temperature()

    def get_conductivity_v(self):
        volts = []
        for i in range(0, self.readings):
            volts.append(self.chan.voltage)
            time.sleep(0.025)
        volt_sum = 0
        for i in volts:
            volt_sum += i
        average = volt_sum / self.readings
        self.conductivity_v = average


def main():
    sensor_manager = SensorManager(4)
    while True:
        sensor_manager.get_conductivity_v()
        sensor_manager.get_temp()
        print("Temp: ", sensor_manager.temps, " V: ", sensor_manager.conductivity_v)
        time.sleep(0.5)


if __name__ == '__main__':
    main()

