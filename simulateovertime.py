# import RPi.GPIO as GPIO
import numpy as np
import matplotlib.pyplot as plt
import time
import matplotlib.animation as animation
from matplotlib import style
from time import sleep
import random
from math import ceil
from scipy import signal
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
# GPIO.setup(18, GPIO.OUT)
# GPIO.setup(27, GPIO.OUT)
# GPIO.setup(22, GPIO.OUT)

simulationTime = 1000
encoderCount = 61
samplingTime = 0.01
setPoint = 500
pwmResolution = 4096 #Because of raspberry pi 3 has 12bit resolution

def my_callback(channel): 
	count += 1 
	return count

def PID(initError):
	error = initError[0]
	esum = initError[2]
	edot = initError[1]
	kP = 10
	kI = 0.2
	kD = 2
	controlout = kP*error + kI * esum + kD * edot
	controlout= controlout/ pwmResolution * 100
	if controlout > pwmResolution:
		controlout = 100
	elif controlout < 0:
		controlout = 0
	return round(controlout, 1)

def antiwindup(initError):
	error = initError[0]
	esum = initError[2]
	edot = initError[1]
	kP = 10
	kI = 0.2
	kD = 2
	# Anti windup logic
	if initError[0] <= 0.001: # when the error near to zero turn off the integral gain
		controlout = kP * error + kD * edot / samplingTime
		if controlout > pwmResolution:
			controlout = 100
		elif controlout < 0:
			controlout = 0
	else: # else act as normal PID 
		controlout = PID(initError)
	return round(controlout, 1)
    
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

def errorUpdate(lastError, ref):
	sensorRead = random.randrange(1, 100)#speedConvert()
	new_state = ceil(ref - sensorRead) # this is the error at instance t+1
	derror = np.divide((new_state - lastError[1]), samplingTime)
	sumerror = (lastError[0] + lastError[2]) * samplingTime
	lastError = np.array([new_state, derror, sumerror])
	lastError = lastError
	return lastError, sensorRead

def speedConvert():
	speedDC = count * 60 / encoderCount
	return speedDC

def animate(i):
    ln1.set_data(x1, y1)
    ln2.set_data(x2, y2)
    return ln1, ln2

# GPIO.add_event_detect(17, GPIO.RISING, callback = my_callback)

initReading = 490 #speedConvert()

t = np.linspace(0, simulationTime, simulationTime * 2)
ref = initReading * np.heaviside(t, 0)
y = np.array([])
u = np.array([])

initError = np.array([setPoint - initReading, 0, 0]) # array include three types of error [currenterror, derivativerror, sum of error]
sensorRead = initReading
e = np.array([])

for count,x in enumerate(t):
	scaleOut = antiwindup(initError)
	y = np.append(y, sensorRead)
	e = np.append(e, initError[0])
	u = np.append(u, scaleOut)
	print(f'control output = {scaleOut}')
	print()
    # pwmOut = GPIO.PWM(scaleOut, 1 / samplingTime)
	initError, sensorRead = errorUpdate(initError, ref[count])
	
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