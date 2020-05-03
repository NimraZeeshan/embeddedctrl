import numpy as np
import matplotlib.pyplot as plt 
import time
# import Rpi.GPIO as GPIO

# Parameter Init
pinPWM = 12 # Pin Number you were using in Raspberry Pi for PWM
pinRotary = 0
episodeNum = 25_000
rotatePenalty = 1
destinationReward = 10
epsilon = 0.95
episodeDecay = 0.5
showPlot = 2500 # Will show plot every 2500 episode

startQtable = None

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
	def currentState(self):
		return None 
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
			return controlout


