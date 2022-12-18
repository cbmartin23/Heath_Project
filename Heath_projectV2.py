

import RPi.GPIO as GPIO
import time
from time import sleep
import numpy as np
import _thread

smooth = .01
hurricane = .002

def stepper(speed,none):

    def stepper_H(state):

        if state:
            spacing = np.linspace(smooth, hurricane, 300)
            
        else:
            spacing = np.linspace(hurricane, smooth, 300)


        for i in spacing:
                
            for fullstep in range(4):
                GPIO.output(control_pins, fullstep_seq[fullstep])
                        
                sleep(i)
                
        print('exiting')
    

        
    GPIO.setmode(GPIO.BCM)
    control_pins=[17,22,27,23]
    
    GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    state = GPIO.input(25)
    prev_state = GPIO.input(25)

    for pin in control_pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, 0)
        
    if state:
        speed = hurricane

    else:
        speed = smooth
    
    fullstep_seq = [[1,0,0,1],[1,1,0,0],[0,1,1,0],[0,0,1,1]]
    fullstep_cc = [[1,0,0,1],[0,0,1,1],[0,1,1,0],[1,1,0,0]]
    
    try:
        while True:
            
            
            for fullstep in range(4):
                GPIO.output(control_pins,fullstep_seq[fullstep])
                
                if prev_state != state:
                    stepper_H(state)
                    
                    if state:
                        speed = hurricane
                        state = 1
                        prev_state = 1
                        
                    else:
                        speed = smooth
                        state = 0
                        prev_state = 0
                 
                sleep(speed)

            prev_state = state
            state = GPIO.input(25)
            #print('prev_state:', prev_state, '\nstate:', state)
            
    except KeyboardInterrupt:
        _thread.exit()
        GPIO.cleanup()


        
def servo_N(none,none1):
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(24, GPIO.OUT)

    servo = GPIO.PWM(24, 50)

    GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    state = GPIO.input(25)
    prev_state = GPIO.input(25)

    servo.start(0)
        
    try:
        
        while True:
                
            if state:
                
                print('In loop')
                servo.ChangeDutyCycle(7)

                sleep(2)

                servo.ChangeDutyCycle(9)

                sleep(2)

            if not state:
                
                servo.ChangeDutyCycle(2)

                sleep(2)

                servo.ChangeDutyCycle(6)

                sleep(2)

            state = GPIO.input(25)

    except KeyboardInterrupt:
        pass
    servo.stop()

    GPIO.cleanup()




def leds(none, none1):

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(5, GPIO.OUT)
    GPIO.setup(12, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)


    set1 = GPIO.PWM(5,50)
    set2 = GPIO.PWM(12,50)
    set3 = GPIO.PWM(13,50)

    GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    state = GPIO.input(25)

    try:
        
        while True:
                
            if not state:
                set1.start(1)



                for i in range(0,100,5):
                    
                    set1.ChangeDutyCycle(i)
                    sleep(.1)

                for i in range(100,0,-5):
                    
                    set1.ChangeDutyCycle(i)
                    sleep(.1)

                set1.stop(0)
                set2.start(1)

                for i in range(0,100,5):
                    
                    set2.ChangeDutyCycle(i)
                    sleep(.1)

                for i in range(100,0,-5):
                    
                    set2.ChangeDutyCycle(i)
                    sleep(.1)

                set2.stop(0)
                set3.start(1)

                for i in range(0,100,5):
                    
                    set3.ChangeDutyCycle(i)
                    sleep(.1)

                for i in range(100,0,-5):
                    
                    set3.ChangeDutyCycle(i)
                    sleep(.1)

            if state:

                for sp in np.linspace(.1, .05, 50):

                    GPIO.output(5,GPIO.HIGH)
                    sleep(sp)
                    GPIO.output(5,GPIO.LOW)
                    
                    GPIO.output(12,GPIO.HIGH)
                    sleep(sp)
                    GPIO.output(12,GPIO.LOW)

                    GPIO.output(13,GPIO.HIGH)
                    sleep(sp)
                    GPIO.output(13,GPIO.LOW)
    

_thread.start_new_thread(stepper, (smooth,None))
_thread.start_new_thread(servo_N, (None, None))
_thread.start_new_thread(leds, (None, None))

    
GPIO.cleanup()
