import RPi.GPIO as gpio
import time

'''
STEP = 20
FR = 26
ST = 19
gpio.setwarnings(False)
gpio.setmode(gpio.BCM)
gpio.setup(STEP,gpio.OUT)
gpio.setup(FR,gpio.OUT,initial = gpio.LOW)#HIGH zhangkai LOW close
gpio.setup(ST,gpio.OUT,initial = gpio.LOW)
PS = gpio.PWM(STEP,700)
PS.start(50)
'''
STEP = 24
FR = 22
ST = 19
    

class hand():
    
    def __init__(self):
        self.STEP = STEP
        self.FR = FR
        self.EN = ST
        self.count_z = 0
        self.count_n = 0
        gpio.setwarnings(False)
        gpio.setmode(gpio.BCM)
        #gpio.setup(self.EN,gpio.OUT,initial = gpio.LOW)
        gpio.setup(self.STEP,gpio.OUT)
        self.PS = gpio.PWM(self.STEP,900)
    
        
    def handopen(self):
        self.PS.start(70)
        gpio.setup(self.FR,gpio.OUT,initial = gpio.HIGH)
        time.sleep(2)
        self.PS.stop()
        
    def handclench(self):
        self.PS.start(70)
        gpio.setup(self.FR,gpio.OUT,initial = gpio.LOW)
        time.sleep(2)
        self.PS.stop()
    def handstop(self):
        self.PS.stop()
        gpio.setup(self.EN,gpio.OUT,initial = gpio.HIGH) 
