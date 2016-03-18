import wpilib as wp

class elevatorObj():
    def __init__(self, motorPort, encdA, encdB, positions, scorePos, motScaler = 1, gain = 35, intIndex = 0, manual = False):
        self._liftMot = wp.TalonSRX(motorPort)
        self._encd = wp.Encoder(encdA, encdB)
        self._encd.reset()
        self._gain = gain
        self._positions = positions
        self._index = intIndex
        self._position = self._positions[self._index]
        self._manual = manual
        self._motScaler = motScaler
        self._manSpeed = 0
        self._speed = 0
        self._setPosion = False
        self._scorePos = scorePos
    def _indexInbounds(self, value):
        if(value < 0): 
            return False
        if(value > (len(self._positions) - 1)):
            return False   
        return True
    
    def _encdLift(self, gain, wntd, cur):
        speed = ((wntd - cur) / gain)
        
        if (speed>1):
            speed = 1
        elif (speed<-1):
            speed = -1
        
        return speed

    def updateClause(self):
        
        if(self._setPosion):
            self._index = 0
            
            while(self._indexInbounds(self._index + 1) and (self._position < self._positions[self._index])):
                self._index += 1
                
            if(self._indexInbounds(self._index + 1)):
                self._index += 1
                
        if(not self._manual):
            self._speed = self._encdLift(self._gain, self._position, self._encd.get())
            self._speed = self._speed*self._motScaler
            self._speed *= -1 #invert 
            self._liftMot.set(self._speed)
        else:
            self._liftMot.set(self._manSpeed)
            self._manual = False
            
    def holdPos(self):
        self._position = self._encd.get()
        self._setPosion = True
		
    def score(self):
        self._position = self._scorePos[self._index]
        
    def indexUp(self, amount = 1):
        if(amount < 0): 
            amount = 0
        
        if((amount + self._index) > len(self._positions) - 1):
            self._index = len(self._positions) - 1
        else:
            self._index += amount
            
        self.setIndex(self._index)
    
    def indexDown(self, amount = 1):
        if(amount < 0): 
            amount = 0
        
        if((self._index - amount) < 0):
            self._index = 0
        else:
             self._index -= amount
       
        self.setIndex(self._index)
        
    def setHome(self):
        self._encd.reset()
        self.goHome()
        
    #turnOnManual: Boolean, whether or not to enable manual control
    #lifterValue: the voltage scaler to give to the lifter motor only if turnOnManual is true
    def manualControl(self, lifterValue):
        self._manual = True
        self._manSpeed = lifterValue
        self._setPosion = True
        
    def setPos(self, position):
        self._position = position
        self._setPosion = True
        
    def setGain(self, gain):
        self._gain = gain
    
    def setPositions(self, positions):
        self._positions = positions
        self._position = self._positions[self._index]

    def setMotorScaler(self, motScale):
        self._motScaler = motScale
                        
    def setIndex(self, index):
        if(not self._indexInbounds(index)): 
            return 0
        
        self._index = index
        self._position = self._positions[self._index]
        self._setPosion = False
        
    def getPos(self):
        return self._position
        
    def atPos(self, windowSize = 18):
        return ((self._position - windowSize) <= self._encd.get() <= (self._position + windowSize))
    
    def getGain(self):
        return self._gain
    
    def getIndexPos(self):
        return self._positions[self._index]
    
    def getSpeed(self):
        return self._speed
    
    def getIndex(self):
        return self._index
    
    def goHome(self):
        self.setIndex(0)
