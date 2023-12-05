# https://stackoverflow.com/questions/16077912/python-serial-how-to-use-the-read-or-readline-function-to-read-more-than-1-char

from mpf.core.mode import Mode
import serial
import time
import threading

ser = serial.Serial("/dev/ttyACM0", 115200)
String inSer

class Base(Mode):
    def listen(self):
        if (ser.in_waiting > 0):
            rec  = ser.read()
            inSer += rec
            if (rec == '\n'):
                print(inSer)
                inSer = ""
        if self.active:
            threading.Timer(1, listen).start()     # start new timer of 1 second

    def mode_start(self, **kwargs):
        print("[Serial Monitor] Base Mode custom python is starting.")
        self.active = True
        self.listen()

            
        