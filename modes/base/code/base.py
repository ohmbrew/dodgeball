# https://stackoverflow.com/questions/16077912/python-serial-how-to-use-the-read-or-readline-function-to-read-more-than-1-char

from mpf.core.mode import Mode
import serial

ser = serial.Serial("/dev/ttyACM0", 115200)

class Base(Mode):
    # def listen(self):
    #     while True:
    #         if (ser.inWaiting() > 0):
    #             data_str = ser.read(ser.inWaitin()).decode('ascii')
    #         println(data_str, end='')
    def mode_init(self):
        print("[Serial Monitor] Base Mode custom python is initialized.")
        

    def mode_start(self, **kwargs):
        print("[Serial Monitor] Base Mode custom python is starting.")
        while True:
            if (ser.inWaiting() > 0):
                data_str = ser.read(ser.inWaiting()).decode('ascii')
            print(data_str, end='')
            
        