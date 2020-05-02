import numpy as np
import matplotlib.pyplot as plt 
import time
# from matplotlib import style
# import Rpi.GPIO as GPIO

# Parameter Init
pinPWM = 12 # Pin Number you were using in Raspberry Pi for PWM
pinRotary = []
episodeNum = 25000
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
error = np.array([])

class main:
	def __init__(self):
		self.pinPWM = pinPWM
		self.pinRotary = pinRotary
		self.setPoint = setPoint
		self.controlalgorithm = controlalgorithm
	def control(self):
		if self.controlalgorithm == 'PID':
		elif self.controlalgorithm == 'antiwindup':
		else:

		return controlout 
	class controller:
		def __init__(self):
			self.pinPWM = self.pinPWM()
			self.pinRotary = self.pinRotary()
			self.setPoint = self.setPoint()
			self.proportionalGain = proportionalGain
			self.derivativeGain = derivativeGain
			self.integralGain = integralGain
			self.learningRate = learningRate
			self.discountRate = discountRate
			self.epsilon = epsilon
			self.episodeDecay = episodeDecay
			self.episode = episode
		def PID(self, error):

			return
	class currentstate:
		def __init__(self):
			self.error = 
