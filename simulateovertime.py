import RPi.GPIO as GPIO
import numpy as np
import matplotlib.pyplot as plt
import time
import matplotlib.animation as animation
from matplotlib import style
from time import sleep
import random
from math import ceil
from qlearningmotor import *
from HMI import Ui_HMID

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
GPIO.setup(18, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
count = 0
simulationTime = 5
refperiod = 5
simulationTime = int(1000 / 24 * simulationTime)
refperiod = int(1000 / 24 * refperiod)
encoderCount = 60
samplingTime = 0.01
scale = 1 / samplingTime
setPoint = 500
pwmResolution = 4096 #Because of raspberry pi 3 has 12bit resolution
pwmOut = GPIO.PWM(18, 1 / samplingTime)


def counterup(channel): 
    global count
    if GPIO.input(channel) == 1:
        count += 1
    else:
        count += 0

def PID(initError):
    error = initError[0]
    esum = initError[2]
    edot = initError[1]
    kP = 2.4579477095186
    kI = 12.6225810958925
    kD = 0.011974174020197
    controlout = kP*error + kI * esum + kD * edot
    #print(controlout)
    #print()
    controlout= controlout / pwmResolution * 100
    if controlout > 100:
        controlout = 100
    elif controlout < 0:
        controlout = 0
    return round(controlout, 1)

def antiwindup(initError):
    error = initError[0]
    esum = initError[2]
    edot = initError[1]
    kP = 2.4579477095186
    kI = 12.6225810958925
    kD = 0.011974174020197
    # Anti windup logic
    if initError[0] <= 0.001: # when the error near to zero turn off the integral gain
        controlout = kP * error + kD * edot
        controlout= controlout / pwmResolution * 100
        if controlout > 100:
            controlout = 100
        elif controlout < 0:
            controlout = 0
    else: # else act as normal PID 
        controlout = PID(initError)
    return round(controlout, 2)
    
def motorDirection(direction):
    if direction == 'ccw': # Counter Clockwise Direction
        GPIO.output(27, 1)
        GPIO.output(22, 0)
    elif direction == 'cw': # Clockwise Direction
        GPIO.output(27, 0)
        GPIO.output(22, 1)
    else: # Stop the Motor
        GPIO.output(27, 0)
        GPIO.output(22, 0)

def directionUpdate(polarity):
    if np.sign(polarity) <  0:
        motorDirection('cw')
    elif np.sign(polarity) > 0:
        motorDirection('ccw')

def errorUpdate(lastError, ref):
    sensorRead = speedConvert() #random.randrange(1, 100)
    new_state = ceil(ref - sensorRead) # this is the error at instance t+1
    derror = np.divide((new_state - lastError[0]), samplingTime)
    sumerror = lastError[2] + lastError[0] * samplingTime
    lastError = np.array([new_state, derror, sumerror])
    lastError = lastError
    return lastError, sensorRead

def speedConvert():
    speedDC = count * 60 * scale / encoderCount
    return speedDC

def animate(i):
    ln1.set_data(x1, y1)
    ln2.set_data(x2, y2)
    return ln1, ln2

ref, t = env.refSignal(simulationTime, 'square', refperiod, 500)
directionUpdate(ref[0])
GPIO.add_event_detect(17, GPIO.RISING, callback = counterup)

initReading = speedConvert()

y = np.array([])
u = np.array([])

initError = np.array([setPoint - initReading, 0, 0]) # array include three types of error [currenterror, derivativerror, sum of error]
sensorRead = initReading
e = np.array([])

for ct,x in enumerate(t):
    scaleOut = antiwindup(initError)
    if np.sign(ref[ct]) < 0:
        y = np.append(y, -sensorRead)
    else:
        y = np.append(y, sensorRead)
    e = np.append(e, initError[0])
    u = np.append(u, scaleOut)
    #print(f'error = {e[ct]}')
    #print()
    pwmOut.start(scaleOut)
    sleep(samplingTime)
    if np.sign(ref[ct]) < 0:
        initError, sensorRead = errorUpdate(initError, -ref[ct])
    else:
        initError, sensorRead = errorUpdate(initError, ref[ct])
    directionUpdate(ref[ct])
    count = 0
    
pwmOut.start(0)

t = np.multiply(t, 24 / 1000)
plt.figure()
plt.title('DC Motor Speed Control')
plt.plot(t, y, label = 'actual output')
plt.plot(t, ref, 'r', label = 'reference signal')
plt.xlabel('Time(s)')
plt.ylabel('Speed (RPM)')
plt.legend(loc=0)
plt.figure()
plt.title('Error Overtime')
plt.plot(t, e, label = 'error signal')
plt.plot(t, u, label = 'control signal')
plt.legend(loc=0)
plt.xlabel('Time(s)')
plt.ylabel('Error')
plt.show()

motorDirection('stop')
GPIO.cleanup()