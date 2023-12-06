# https://stackoverflow.com/questions/16077912/python-serial-how-to-use-the-read-or-readline-function-to-read-more-than-1-char

from mpf.core.mode import Mode
import serial
import time
import threading


ser = serial.Serial("/dev/ttyACM0", 115200)
inData = ""

class Base(Mode):
    
    def listen(self):
        global inData

        while (ser.in_waiting > 0):
            rec  = ser.read()
            if (rec[0] != 0xd):
                if (rec[0] == 0xa):
                    print(inData)       # change this to fire an MPF event with data in its params
                    paddleParams = {'p1pos': '100'}
                    self.machine.events.post(event='paddle_update', m1pos=100, m1set=200)
                    inData = ""
                else:
                    inData += rec.decode('utf-8')
        if self.active:
            threading.Timer(1, self.listen).start()     # start new timer of 1 second

    def mode_start(self, **kwargs):
        print("[Serial Monitor] Base Mode custom python is starting.")
        inData = ""
        self.active = True
        self.listen()

            
        