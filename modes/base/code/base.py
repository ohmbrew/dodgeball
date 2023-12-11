# This custom python module for Mission Pinball Framework links together the LattePanda 3 Delta SBC's on-board Ardino and MPF's framework
# The arduino is running its own real-time closed loop paddle positioning algorithm, and regularly sends data over its serial port, which
# is connected to the SBC and can be found at /dev/ttyACM0 (at least in my setup). This class opens up that serial port and reads characters
# until it sees a \n. It disregards \r, which can be apparently sent with Arduino's println() method. Once it receives a full message,
# it posts a custom "paddle_update" event in MPF, with each paddle's state (S=Stopped, T=Tracking) and positions (p1 set point, p1 position,
# p2 set point, p2 position).

# This can then be acted on by MPF:
# - track the paddle with a graphic
# - draw fire trail on ball
# - make virtual games like Crossfire, etc.
# - show where the set point is with a graphic

# OR

# We run all game logic in here.
# We create a display, we can draw to the display.
# MPF can play audio based on events sent from here or we figure out how to play an audio file from here
# We can do the AI logic here
# We receive the messages from Arduino about paddle location here
# We can open the webcam here and use the test script I've been working on (10 Dec)
# We can send paddle location commanding (for AI player 2) from here

from mpf.core.mode import Mode
import serial
import time
import threading
import cv2

from utils import *

rate = 50     # how many milliseconds between calls to listen()
ser = serial.Serial("/dev/tty2", 115200)
inData = ""

class Base(Mode):
    # self-calling based on timer event firing every x milliseconds
    def listen(self):
        global inData

        while (ser.in_waiting > 0):
            rec  = ser.read()
            if (rec[0] != 0xd):
                if (rec[0] == 0xa):
                    # inData now has a full update. It may look like this: P,S,100,100,T,500,100
                    parsed = inData.split(sep=",")
                    print(inData)
                    if parsed[0] == "P":
                        # we have a paddle position update (only type of message right now!)
                        #print(inData)       # change this to fire an MPF event with data in its params
                        self.machine.events.post(event='paddle_update', p1_state=parsed[1], p1_set_point=parsed[2], p1pos=parsed[3], p2_state=parsed[4], p2_set_point=parsed[5], p2_pos=parsed[6])
                    inData = ""
                else:
                    inData += rec.decode('utf-8')
        if self.active:
            threading.Timer(1, self.listen).start()     # start new timer of 1 second

    def mode_start(self, **kwargs):
        print("[Serial Monitor] Base Mode custom python is starting.")
        inData = ""
        self.active = True

        # try CV2 stuff
        cap = cv2.VideoCapture(2)		# 2 = index of my USB webcam. May change in the future?
        if not cap.isOpened():
            print("Cannot open camera.")
            exit()
        else:
            print("Video capture initialized.")

        prev_time = 0
        new_time = 0
        fond = cv2.FONT_HERSHEY_DUPLEX
        playfield_corners = None
        prev_frame = None
        runs = 0



        self.listen()

            
        