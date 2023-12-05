# https://stackoverflow.com/questions/16077912/python-serial-how-to-use-the-read-or-readline-function-to-read-more-than-1-char

from mpf.core.mode import Mode
import serial
import time

ser = serial.Serial("/dev/ttyACM0", 115200)

class Base(Mode):
    def mode_init(self):
        print("[Serial Monitor] Base Mode custom python is initialized.")
        

    def mode_start(self, **kwargs):
        print("[Serial Monitor] Base Mode custom python is starting.")
        while True:
            if (ser.in_waiting() > 0):
                data_str = ser.read(ser.in_waiting()).decode('ascii')
                print(data_str, end='')
            time.sleep(.1)
            
        