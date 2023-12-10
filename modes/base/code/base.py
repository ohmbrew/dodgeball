# This custom python module for Mission Pinball Framework links together the LattePanda 3 Delta SBC's on-board Ardino and MPF's framework
# The arduino is running its own real-time closed loop paddle positioning algorithm, and regularly sends data over its serial port, which
# is connected to the SBC and can be found at /dev/ttyACM0 (at least in my setup). This class opens up that serial port and reads characters
# until it sees a \n. It disregards \r, which can be apparently sent with Arduino's println() method. Once it receives a full message,
# it posts a custom "paddle_update" event in MPF, with each paddle's state (S=Stopped, T=Tracking) and positions (p1 set point, p1 position,
# p2 set point, p2 position).
# This can then be acted on by MPF:
# - track the paddle with a graphic
# - show where the set point is with a graphic
# TODO: Haven't figured out where the AI will be performed. I don't think it'll be MPF...so I'm not sure how I'm going to communicate back to Arduino
# when we want to move the AI Player 2 paddle. I think it may also lay in a custom python module for MPF...

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

            
        