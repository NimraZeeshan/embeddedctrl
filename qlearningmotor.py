import numpy as np
import matplotlib.pyplot as plt 
import time
import random
# import Rpi.GPIO as GPIO

# Parameter Init
pinPWM = 12 # Pin Number you were using in Raspberry Pi for PWM
pinRotary = 0
episodeNum = 4_000
rotatePenalty = 1
destinationReward = 10
epsilon = 0.95
decayValue = epsilon / (episodeNum - 1)
showPlot = 2500 # Will show plot every 2500 episode

Q_table_start = np.zeros([128, 2]) # init the q table space, in this plant we have to action either to drive or brake

learningRate = 0.3
discountRate = 0.9
setPoint = 32 # Choose your DC Motor Speed setpoint (in RPM)
controlalgorithm = 'qlearn'
currentError = setPoint
error = np.array([43, 21, 122]) # array include three types of error [currenterror, derivativerror, sum of error]
timeSampling = 0.001
pwmResolution = 4096 #Because of raspberry pi 3 has 12bit resolution
derivativeGain = 121
proportionalGain = 1000
integralGain = 0.02

# Main Program
class main:
	def __init__(self, pinPWM, setPoint, timeSampling):
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
		self.episodeDecay = episodeDecay
		self.episode = episodeNum
	def control(self, controlalgorithm):
		if controlalgorithm == 'PID':
			controlout = self.controller.PID(self, error = error, sampling = timeSampling)
		elif controlalgorithm == 'antiwindup':
			controlout = self.controller.antiwindup(self, error = error, sampling = timeSampling)
		else:
			controlout = self.controller.qlearn(self, error = error)
		return controlout
	def theAgent(self):
		return error 
	class controller(main):
		def PID(self, error, sampling):
			e = error[0]
			edot = error[1]
			esum = error[2]
			controlout = self.proportionalGain * e + self.derivativeGain * edot / sampling + self.integralGain * esum * sampling
			controlout = controlout / self.pwmResolution * 100
			if controlout > self.pwmResolution:
				controlout = 100
			return controlout
		def antiwindup(self, error, sampling):
			controlout = PID(error, sampling)
			e = error[0]
			edot = error[1]
			esum = error[2]
			# Anti windup logic
			if e <= 0.001: # when the error near to zero turn off the integral gain
				controlout = self.proportionalGain * e + self.derivativeGain * edot / sampling 
				controlout = controlout / self.pwmResolution * 100
			else: # else act as normal PID 
				controlout = self.controller.PID(error, sampling)
			return controlout
		def qlearn(self, error):
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
					sensorRead = random.sample(range(1,64), 1) # Replace this with your raspberry pi 3 reading value
					new_state = ceil(sensorRead - self.setPoint) # this is the new state
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

if __name__ == "__main__":
    main()
