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
                    # inData now has a full update. It may look like this: P,S,100,100,T,500,100
                    parsed = inData.split(sep=",")
                    if parsed[0] == "P":
                        # we have a paddle position update (only type of message right now!)
                        #print(inData)       # change this to fire an MPF event with data in its params
                        self.machine.events.post(event='paddle_update', p1=parsed[1], p1set=parsed[2], p1pos=parsed[3], p2=parsed[4], p2set=parsed[5], p2pos=parsed[6])
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

            
        