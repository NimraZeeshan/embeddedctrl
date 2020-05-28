import numpy as np
import matplotlib.pyplot as plt 
import time
import random
# import Rpi.GPIO as GPIO

# Parameter Init
pinPWM = 17 # Pin Number you were using in Raspberry Pi for PWM
pinRotary = 0
episodeNum = 4_000
rotatePenalty = 1
destinationReward = 10
epsilon = 0.95
decayValue = epsilon / (episodeNum - 1)
showPlot = 2500 # Will show plot every 2500 episode
learningRate = 0.3
discountRate = 0.9

Q_table_start = np.zeros([128, 2]) # init the q table space, in this plant we have to action either to drive or brake

initReading = 0 # The first time your sensor read the speed 
setPoint = 32 # Choose your DC Motor Speed setpoint (in RPM)
controlalgorithm = 'qlearn'
initError = np.array([setPoint - initReading, 0, 0]) # array include three types of error [currenterror, derivativerror, sum of error]
timeSampling = 0.001 # Systems sampling time
samplingFreq = 1 / timeSampling # Systems sampling frequency 
pwmResolution = 4096 #Because of raspberry pi 3 has 12bit resolution
derivativeGain = 121
proportionalGain = 1000
integralGain = 0.02
period = 0.5
frequency = 1 / period
amplitude = 32 # The range is from -maximum motor speed to maximum motor speed
simulationTime = 360 # your simulation time
# Main Program
class env():

    def __init__(self, pinPWM, lastError, timeSampling):
        self.pinPWM = pinPWM
        self.pinRotary = pinRotary
        self.pwmResolution = pwmResolution
        self.setPoint = setPoint
        self.controlalgorithm = controlalgorithm
        self.timeSampling = timeSampling
        self.proportionalGain = proportionalGain
        self.derivativeGain = derivativeGain
        self.integralGain = integralGain
        self.learningRate = learningRate
        self.discountRate = discountRate
        self.epsilon = epsilon
        self.episodeDecay = decayValue
        self.episode = episodeNum
        self.frequency = frequency
        self.amplitude = amplitude
        self.simulationTime = simulationTime
        self.lastError = lastError

    def errorUpdate(self):
        sensorRead = sensorRead()
        new_state = ceil(sensorRead - self.setPoint) # this is the error at instance t+1
        derror = self.lastError[1]
        sumerror += self.lastError[2] * self.timeSampling
        lastError = np.array([new_state, derror, sumerror])
        self.lastError = lastError
        return lastError

    def sensorRead(self):
        newval = random.randint(1, 100)
        return newval

    def refSignal(simulationTime, reference, period, amplitude):
        frequency = 1 / period
        if (frequency > 0.5 * samplingFreq) & (reference != 'unit step'):
            raise Exception("Please use frequency at least less than half times of your sampling frequency to avoid aliasing")
        t = np.linspace(0, simulationTime, simulationTime * 2)
        # Signal Reference List     
        if reference == 'sinusoidal': # Sinusoidal
            y = amplitude * np.sin(2 * np.pi * frequency * t)
        elif reference == 'unit step': # Unit Step Signal
            y = amplitude * np.heaviside(t, 0)
        elif reference == 'square': # Square Wave Signal
            y = amplitude * np.sign(np.sin(2 * np.pi * frequency * t))
        elif reference == 'random':
            # Pseudo Random Binary Sequence (PRBS)
            extendTime = self.simulationTime * 10 # we multiple the simulation time by 10 to extend the PRBS signal time
            initSpace = np.zeros(extendTime)
            freqRange = np.random.randint(frequency, samplingFreq, extendTime)
            idxAmp = 0
            while idxAmp < extendTime:
                initSpace[idxAmp] = amplitude
                initSpace[idxAmp + 1] = -amplitude
                idxAmp += 2
            idxTime = 0
            y = np.zeros(extendTime)
            while freqRange[idxTime] < extendTime:
                idx = freqRange[idxTime]
                y[idx:] = initSpace[idxTime]
                idxTime += 1
        else: # Manual control
            y = None
        return y, t

    def PID(self, error, sampling):
        controlout = self.proportionalGain * self.lastError[0] + self.derivativeGain * self.lastError[1] / sampling + self.integralGain * self.lastError[2]
        controlout = controlout / self.pwmResolution * 100
        if controlout > self.pwmResolution:
            controlout = 100
        return controlout

    def antiwindup(self, error, sampling):
        # Anti windup logic
        if self.lastError[0] <= 0.001: # when the error near to zero turn off the integral gain
            controlout = self.proportionalGain * self.lastError[0] + self.derivativeGain * self.lastError[1] / sampling 
            controlout = controlout / self.pwmResolution * 100
        else: # else act as normal PID 
            controlout = self.PID(error, sampling)
        return controlout

    def qlearn(self, error):
        # The Q-Learning in this case will be act as a bang-bang controller (on-off control)
        Q_table = Q_table_start
        rowterm, colterm = 63, 1
        init_state = ceil(error[0])
        for x in range(self.episode):
            stop = False
            while not stop:
                if random.uniform(0, 1) < self.epsilon:
                    if random.uniform(0,1) > 0.5:
                        colstate = 1
                    else:
                        colstate = 0
                        controlout = Q_table[init_state, colstate]
                else:
                    colstate = np.argmax(Q_table[init_state, :])
                    controlout = Q_table[init_state, colstate]
                new_state = self.errorUpdate(self.sensorRead()) # this is the new state
                new_colstate = np.argmax(Q_table[new_state, :])
                if (init_state == rowterm and colstate == colterm):
                    reward = destinationReward
                    stop = True
                else:
                    reward = -rotatePenalty
                now_qvalue = Q_table[init_state, :]
                maxqnext = Q_table[new_state, new_colstate] 
                new_qvalue = now_qvalue * (1 - learningRate) + learningRate * (reward + self.discountRate * maxqnext)
                Q_table[init_state, colstate] = new_qvalue
                init_state, colstate = new_state, new_colstate
                if self.episodeNum >= episode >= 1:
                    self.epsilon -= decayValue

class control(env):
    
    def __init__(self):
        super().__init__(pinPWM, initError, timeSampling)
    
    def applications(self, controlalgorithm):
        if controlalgorithm == 'PID':
            controlout = super().PID(error = self.lastError, sampling = self.timeSampling)
        elif controlalgorithm == 'antiwindup':
            controlout = super().antiwindup(error = self.lastError, sampling = self.timeSampling)
        else:
            controlout = super().qlearn(error = self.lastError)
        return controlout

if __name__ == "__main__":
    env(pinPWM, initError, timeSampling)