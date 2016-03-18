#!/usr/bin/env python3

import wpilib as wp
import time as tm
import elevator as ele
import wpiJoystickOverlay as joy
import feeders as feed
from config import *

#xIn is the x axis of the joystick
#yIn is the y axis of the joystick
#rot is the rotation of the joystick
#wheelArray is fed in and modified with 4 values that are fed into motor functions, -1 to 1
def mecanum(xIn, yIn, strafe, wheelArray, deadZone = 0.05):
	wheelArray[0] = xIn - yIn - strafe
	wheelArray[1] = -xIn - yIn + strafe
	wheelArray[2] = xIn - yIn + strafe
	wheelArray[3] = -xIn - yIn - strafe

#sets wheelarrays to the side they are on the robot for tank drive
#wheels 0 and 2 are left
#wheels 1 and 3 are right
def tank(leftSet, rightSet, wheelArray):
	wheelArray[0] = leftSet
	wheelArray[1] = rightSet
	wheelArray[2] = leftSet
	wheelArray[3] = rightSet
	
#xIn is the x axis of the joystick
#yIn is the y axis of the joystick
#move the robot in any derevtion like in a arcade	
def arcade(xAxis, yAxis, wheelArray):
	wheelArray[0] = yAxis - xAxis 
	wheelArray[1] = yAxis + xAxis
	wheelArray[2] = yAxis - xAxis
	wheelArray[3] = yAxis + xAxis
			
class MyRobot(wp.SampleRobot):
	
	def robotInit(self):
		'''Robot initialization function'''
		#positions = [0,-625,-1210,-1815,-2425,-2635]
		positions = [0, -325, -650, -975, -1300, -1395]
		scorePos = [0, 0, -275, -545, -805, -1075]
		
		self.motorFrontRight = wp.TalonSRX(frontRightPort)
		self.motorBackRight = wp.TalonSRX(backRightPort)
		self.motorFrontLeft = wp.TalonSRX(frontLeftPort)
		self.motorBackLeft = wp.TalonSRX(backLeftPort)	  
	
		self.stick = joy.happyHappyJoyJoy(leftDriveJoyPort)
		self.stick2 = joy.happyHappyJoyJoy(rightDriveJoyPort)
		self.cop = joy.happyHappyJoyJoy(coPilotJoyPort)
		self.smartDs = wp.SmartDashboard() #the smart dashboard communication
		self.accel = wp.BuiltInAccelerometer()
		
		self.driveEncd = wp.Encoder(driveEncdAPort,driveEncdBPort)
		
		self.feeders = feed.feedMe(feederLeftPort, feederRightPort, 
									feedUpChannel, feedDownChannel, 0.35)
		
		self.rightSwitch = wp.DigitalInput(rightToteDetPort)
		self.leftSwitch = wp.DigitalInput(leftToteDetPort)
		self.autoSwitch = wp.DigitalInput(autoSwitchPort)

		self.comp = wp.Compressor()
		self.Vator = ele.elevatorObj(liftMotPort,liftEncdAPort,
										liftEncdBPort,positions, scorePos,
											0.75, 30)

		try:
			self.camServ = wp.CameraServer()
			self.usbCam = wp.USBCamera()
			self.usbCam.setExposureManual(50)
			self.usbCam.setBrightness(80)
			self.usbCam.updateSettings() # force update before we start thread
			self.camServ.startAutomaticCapture(self.usbCam)
		except:
			pass
		
	def autonomous(self):
		begin = tm.clock()
		self.driveEncd.reset()
		motorSpeed = .25
		if(self.isAutonomous() and self.isEnabled()):
			if(self.autoSwitch.get()):
				while(abs(self.driveEncd.get()) < 750): #500
					self.motorFrontRight.set(-motorSpeed)
					self.motorBackRight.set(-motorSpeed)
					self.motorFrontLeft.set(motorSpeed)
					self.motorBackLeft.set(motorSpeed)
					
					if((tm.clock() >= (begin + 10)) or not self.isAutonomous()):
						break

				self.motorFrontRight.set(0)
				self.motorBackRight.set(0)
				self.motorFrontLeft.set(0)
				self.motorBackLeft.set(0)

			else:
				while(abs(self.driveEncd.get()) < 300):
					self.motorFrontRight.set(-motorSpeed)
					self.motorBackRight.set(-motorSpeed)
					self.motorFrontLeft.set(motorSpeed)
					self.motorBackLeft.set(motorSpeed)
					
					if((tm.clock() >= (begin + 10)) or not self.isAutonomous()):
						break

				self.motorFrontRight.set(0)
				self.motorBackRight.set(0)
				self.motorFrontLeft.set(0)
				self.motorBackLeft.set(0)
										
	def disabled(self):
		pass
	
	def operatorControl(self):
		digL = False
		digR = False
		
		self.comp.start()
		clock = -100 #timer attached to the score funct, needs to be very small
		feedtime = -100 #timer attached to the feeder, needs to be very small
		wheelSpeed = [0,0,0,0] #this is the variable to control motor voltage
		motorGain = 0.5 #this is a scaler to the drive motors

		feedInScoreSec = 1
		scoreWaitSec = 5 #This is how many seconds to wait while scoring
		timerTest = 0
		cycleTime = 0
		totalTime  = 0 
		i= 0
		chanceFailure = 0
		loopDelay = 0.01
		averageTime = 0
		while self.isOperatorControl() and self.isEnabled():
			timerTest = tm.clock()
			#swapping out what form of drive we are doing
			mecanum( self.stick2.getX(), 
						self.stick2.getY(), 
							self.stick.getX(), 
								wheelSpeed) 
		
			if(self.cop.getButton(2)):
				self.Vator.setHome()
				
			#indexing totes
			if (self.cop.getButtonRise(6)):
				self.Vator.indexUp()
			if (self.cop.getButtonRise(7)):
				self.Vator.indexDown()
				
			#feeder go home
			if(self.cop.getButton(9)):
				self.Vator.goHome()

			#go to tote scoring position
			if(self.cop.getButtonRise(8)):
				self.Vator.setPos(-1475)
				
			#feeder code
			if(self.cop.getButton(11)):
				self.feeders.feedIn()
			if(self.cop.getButton(10)):
				self.feeders.feedOut()

			#tote detection
			if(self.Vator.atPos() and (not self.leftSwitch.get()) and (not self.rightSwitch.get()) and self.feeders.isDown()):
				if((digL == False) or (digR == False)):
					self.Vator.indexUp()
					feedtime = (tm.clock() + feedInScoreSec)
												
			#Update the dig last value, do not move or else bad juju
			digL = not self.leftSwitch.get()
			digR = not self.rightSwitch.get()
						
			if(self.stick.getButton(6)):
				self.feeders.armUp()
			if(self.stick.getButton(7)):
				self.feeders.armDown()		
								
			#dump the totes to score
			if (self.cop.getButtonRise(1)):
				self.Vator.score()
				clock = tm.clock() + scoreWaitSec
		
			if(not self.cop.inDeadZoneY()):
				self.Vator.manualControl(self.cop.getY())
				
			if(self.cop.getEventY() and self.cop.inDeadZoneY()):
				self.Vator.holdPos()

			if(feedtime > tm.clock()):
				self.feeders.feedOut()
				
			#Manual Mode
			if (self.cop.getZ() > 0.5):
				self.Vator.manualControl(self.cop.getY())
				feedtime = tm.clock() - 1
				
			if((clock > tm.clock()) and self.Vator.atPos()):
				wheelSpeed[0] -= 0.3
				wheelSpeed[1] -= 0.3
				wheelSpeed[2] -= 0.3
				wheelSpeed[3] -= 0.3
				self.feeders.feedIn()

			if(not self.stick.inDeadZoneX()):
				 clock = -100
			if(not self.stick.inDeadZoneY()):
				 clock = -100
			if(not self.stick2.inDeadZoneX()):
				 clock = -100

			#output to dashboard
			
			self.smartDs.putString("DB/String 0", "Cycle Time: " + str(cycleTime)[0:4])			 
			self.smartDs.putString("DB/String 1", "Total Cycle Time: " + str(totalTime)[0:4])
			self.smartDs.putString("DB/String 2", "Average CycleTime: " + str(averageTime)[0:4])   
			self.smartDs.putString("DB/String 3", "Change Failure : " + str(chanceFailure) + "%")
			
			self.smartDs.putString("DB/String 4", "At Pos: " + str(self.Vator.atPos()))
			self.smartDs.putString("DB/String 5", "liftVolt: " + str(self.Vator.getSpeed())[0:5])
			self.smartDs.putString("DB/String 6", "currentPos: " + str(self.Vator._encd.get())[0:5])
			self.smartDs.putString("DB/String 7", "wantPos: " + str(self.Vator.getPos())[0:5])
			self.smartDs.putString("DB/String 8", "Drive Encd: " + str(self.driveEncd.get())[0:5])
			
			#give motors value
			self.motorBackLeft.set(wheelSpeed[0]*motorGain)
			self.motorFrontLeft.set(wheelSpeed[2]*motorGain)
			self.motorBackRight.set(-wheelSpeed[1]*motorGain)
			self.motorFrontRight.set(- wheelSpeed[3]*motorGain)
			
			self.Vator.updateClause()
			self.feeders.updateClause()
			
			#zero out value
			wheelSpeed = [0,0,0,0]
			
			cycleTime = (tm.clock() - timerTest)
			totalTime += cycleTime
			i += 1
			averageTime = totalTime/i
			chanceFailure = int(averageTime/(averageTime + loopDelay) * 100)
			
			#print("CycleTime: " + str(cycleTime) + "\n" + "averageTime: " + str(totalTime/i) + "\n" + "Change Failure : " + str(int(averageTime/(averageTime + .01) * 100)) + "%")
			wp.Timer.delay(loopDelay)   # Wait, if you don't button event code will have a major mess up

if __name__ == '__main__':
	wp.run(MyRobot)
