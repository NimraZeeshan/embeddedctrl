import numpy as np
import matplotlib.pyplot as plt 
import time
# import Rpi.GPIO as GPIO

# Parameter Init
pinPWM = 12 # Pin Number you were using in Raspberry Pi for PWM
pinRotary = []
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
error = {'present' : [], 
		 'past'    : [],
		 'sum'	   : [] }
timeSampling = 0.001
pwmResolution = 4096 #Because of raspberry pi 3 has 12bit resolution

class main:
	def __init__(self):
		self.pinPWM = pinPWM
		self.pinRotary = pinRotary
		self.pwmResolution = pwmResolution
		self.setPoint = setPoint
		self.controlalgorithm = controlalgorithm
	def control(self):
		if self.controlalgorithm == 'PID':
			controlout = controller.PID(self, error)
		elif self.controlalgorithm == 'antiwindup':
			controlout = controller.antiwindup(self, error)
		else:
			controlout = controller.qlearn(self, error)
		return controlout 
	class controller(main):
		def __init__(self):
			self.pinPWM = self.pinPWM()
			self.pinRotary = self.pinRotary()
			self.setPoint = self.setPoint()
			self.pwmResolution = self.pwmResolution()
			self.proportionalGain = proportionalGain
			self.derivativeGain = derivativeGain
			self.integralGain = integralGain
			self.learningRate = learningRate
			self.discountRate = discountRate
			self.epsilon = epsilon
			self.episodeDecay = episodeDecay
			self.episode = episode
		def PID(self, error, sampling):
			e = error['present']
			edot = error['past']
			esum = error['sum']
			controlout = self.proportionalGain * e + self.derivativeGain * edot / sampling + self.integralGain * esum * sampling
			controlout = controlout / self.pwmResolution * 100
			if controlout > self.pwmResolution:
				controlout = 100
			return controlout
		def antiwindup(self, error, sampling):
			controlout = PID(error, sampling)
			e = error['present']
			# Anti windup logic
			if e <= 0.001: # when the error near to zero turn off the integral gain
				controlout = self.proportionalGain * e + self.derivativeGain * edot / sampling 
			elif: # else act as normal PID 
				controlout = PID(sampling)
			return controlout
		def qlearn(self, error):
			return controlout
	class currentstate(main):
		def __init__(self, error):
			self.error = error


