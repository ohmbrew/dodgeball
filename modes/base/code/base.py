# https://stackoverflow.com/questions/16077912/python-serial-how-to-use-the-read-or-readline-function-to-read-more-than-1-char

from mpf.core.mode import Mode
import serial

ser = None

class Base(Mode):
    def mode_init(self):
        print("[Serial Monitor] Base Mode custom python is initialized.")
        ser = serial.Serial("/dev/ttyACM0", 115200)

    def mode_start(self, **kwargs):
        print("[Serial Monitor] Base Mode custom python is starting.")
        while True:
            cc = str(ser.realine())
            print(cc.decode('utf-8'))