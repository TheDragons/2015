import wpilib as wp

class feedMe():
	def __init__(self, leftMotPort, rightMotPort, solenoidUpPort, solenoidDownPort, motGain = 1, intPos = 0):
		self._leftMot = wp.TalonSRX(leftMotPort)
		self._rightMot = wp.TalonSRX(rightMotPort)
		#self._feedNoid = 5 if (self._joystickObj.getAxisCount() == 0) else self._joystickObj.getAxisCount()
		self._feedNoid = wp.DoubleSolenoid(solenoidUpPort,solenoidDownPort)
		self._feedNoid.set(intPos)
		self._leftSpeed = 0
		self._rightSpeed = 0
		self._motGain = motGain
		
	def updateClause(self, holdValues = False):
		self._leftMot.set(self._leftSpeed * self._motGain)
		self._rightMot.set(self._rightSpeed * self._motGain)
        
		if(not holdValues):
				self.stopFeeders()
		
	def armUp(self):
		self._feedNoid.set(1)

	def armDown(self):
		self._feedNoid.set(2)
   
	def armHold(self):
		self._feedNoid.set(0)
   
	def isDown(self):
		return self._feedNoid.get() == 2
   
	def isUp(self):
		return self._feedNoid.get() == 1
   
	def isHold(self):
		return self._feedNoid.get() == 0
	
	def feedIn(self, motValue = 1):
		if motValue < 0:
			motValue = 0
		self._rightSpeed = motValue * -1
		self._leftSpeed = motValue
	
	def feedOut(self, motValue = 1):
		if motValue < 0:
			motValue = 0
		self._rightSpeed = motValue
		self._leftSpeed = motValue * -1
	
	def setRightSpeed(self, motValue = 0):
		self._rightSpeed = motValue
	
	def setLeftSpeed(self, motValue = 0):
		self._leftSpeed = motValue
	
	def stopFeeders(self):
		self._rightSpeed = 0
		self._leftSpeed = 0
		
	def setMotorGain(self, motGain):
		self._motGain = motGain
		
	def joyControl(self, xAxis, yAxis):
		self._rightSpeed = xAxis + yAxis
		self._leftSpeed = xAxis - yAxis