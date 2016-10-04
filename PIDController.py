import collections, time, threading, numpy, queue
# calculate the sum of latest N items

class DataPoint:
    def __init__(self, data, duration):
        self.data = data
        self.duration = duration
    def getProduct(self):
        return self.data * self.duration
class LimitedAggregator:
    calibrateThreshold = 3000
    def __init__(self, maxNumItem):
        self.length = maxNumItem
        self.dataStorage = collections.deque(maxlen=self.length)
        self.sum = 0
        self.pushCount = 0
        for i in range(self.length):
            self.dataStorage.append( DataPoint(0,0) );
    # recalculate the sum in the old fashion way -- add all things together
    def calibrate(self):
        tempSum = 0
        for currVal in self.dataStorage:
            tempSum += currVal.getProduct()
        self.sum = tempSum
    # update the sum in the ring-buffer way
    def push(self, data, duration):
        if self.pushCount > LimitedAggregator.calibrateThreshold:
            self.calibrate()
            pushCount = 0
        oldestValue = self.dataStorage[0]
        latestValue = DataPoint(data, duration)
        self.dataStorage.append(latestValue)
        self.sum -= oldestValue.getProduct()
        self.sum += latestValue.getProduct()
        self.pushCount += 1
    def reset(self):
        for i in range(self.length):
            self.push(0,0)

    def getSum(self):
        return self.sum

    def getLatestValue(self):
        return self.dataStorage[self.length-1].data
    def getLatestDuration(self):
        ret = self.dataStorage[self.length-1].duration
        if ret == 0:
            return DiscretePIDController.defaultDuration;
        return ret
    def getDifferentBetweenLatestTwo(self):
        if self.length < 2:
            return 0
        return (self.getLatestValue() - self.dataStorage[self.length -2].data) / self.getLatestDuration()

class DiscretePIDController:
    defaultDuration = 0.25
    smoother = 0.1
    def __init__(self, Kp, Ki, Kd, minU, maxU, initU = 0.0):
        self.errorDataSet = LimitedAggregator(20);
        self.desiredValue = 0
        self.oldCtl = initU
        self.proportionalCoefficient = Kp
        self.integralCoefficient = Ki
        self.derivativeCoefficient = Kd
        self.shouldStop = False
        self.lowerBound = minU
        self.upperBound = maxU
        self.changeBound = abs(maxU - minU) * DiscretePIDController.smoother
        self.lastDataFeedTime = -1.0;

    def reInit(self):
        self.errorDataSet.reset()

    def newTarget(self, target):
        self.desiredValue = target

    def ctl(self, curVal, curTm):
        self.feedData( curVal, curTm)
        newCtl = self.getControllVariable()
        newCtl = numpy.clip( newCtl, self.oldCtl - self.changeBound, self.oldCtl + self.changeBound)
        self.oldCtl = newCtl
        return newCtl
    #observe the world
    def feedData(self, observedValue, curTm):
        # if the it's first to read in the data, we will just set the last data feed time so that the
        # duration is default duration
        if self.lastDataFeedTime == -1.0:
            self.lastDataFeedTime = curTm - DiscretePIDController.defaultDuration
        duration = curTm - self.lastDataFeedTime
        self.lastDataFeedTime = curTm
        self.errorDataSet.push( self.desiredValue - observedValue, duration)

    #how to control the rudder and stuff
    def getControllVariable(self):
        ret =  self.errorDataSet.getLatestValue() * self.proportionalCoefficient
        ret += self.errorDataSet.getDifferentBetweenLatestTwo() * self.derivativeCoefficient
        ret += self.errorDataSet.getSum() * self.integralCoefficient
        return numpy.clip(ret, self.lowerBound, self.upperBound)

class DiscretePIDControllerDriver:
    LookFunctionDecipher = {
            'throttle' : 0, #look()[0] is air speed
            'elevator' : 3, #look()[3] is is the pitch
            'aileron'  : 4, #look()[4] is the roll
            'time'     : 7, #look()[7] is the time
            }
    def __init__(self, FGOCommunicator, whatToControl,Kp, Ki, Kd, minU, maxU, initU = 0.0, ):
        self.FGOCommunicator = FGOCommunicator
        self.stabilizeTarget = whatToControl
        self.stablizer = DiscretePIDController(Kp, Ki, Kd, minU, maxU, initU)
        self.shouldStop = False
        self.controlVal = []
        self.targetVal = []

    def createPitchStablizer(FGOCommunicator, Kp, Ki, Kd):
        CD = DiscretePIDControllerDriver(FGOCommunicator, 'elevator', Kp, Ki, Kd, -1.0, 1.0, float(FGOCommunicator.FGOController.getFGProp("elevator")))
        CD.stablizer.newTarget(0)
        return CD
    
    def createAirSpeedStablizer(FGOCommunicator, Kp, Ki, Kd):
        CD = DiscretePIDControllerDriver(FGOCommunicator, 'throttle', Kp, Ki, Kd, 0, 1.0, float(FGOCommunicator.FGOController.getFGProp("throttle")))
        CD.stablizer.newTarget(90)
        return CD
    
    def createRollStablizer(FGOCommunicator, Kp, Ki, Kd):
        CD = DiscretePIDControllerDriver(FGOCommunicator, 'aileron', Kp, Ki, Kd, -1.0, 1.0, float(FGOCommunicator.FGOController.getFGProp("aileron")))
        CD.stablizer.newTarget(0)
        return CD

    def setNewPIDParameter(self, p = None, i = None, d = None, reInit = False):
        try:
            Kp = int(p)
        except (ValueError, TypeError):
            pass
        else:
            self.stablizer.proportionalCoefficient = Kp
        try:
            Ki = int(i)
        except (ValueError, TypeError):
            pass
        else:
            self.stablizer.integralCoefficient = Ki

        try:
            Kd = int(d)
        except (ValueError, TypeError):
            pass
        else:
            self.stablizer.derivativeCoefficient = Kd

        if reInit:
            self.stopStablizing()
            self.stablizer.reInit()
            self.stablize()
    def setNewTarget(self, t):
        try:
            target = int(t)
        except (ValueError, TypeError):
            pass
        else:
            self.stablizer.newTarget(target)

    def __extractDataFormLookFunction(self):
        raw = self.FGOCommunicator.getCachedLook()
        time = raw[DiscretePIDControllerDriver.LookFunctionDecipher['time']]
        data = raw[DiscretePIDControllerDriver.LookFunctionDecipher[self.stabilizeTarget]]
        return [data, time]
    def stablize(self):
        t = threading.Thread(target=DiscretePIDControllerDriver.__stablize, args=(self,))
        t.start()
    def __stablize(self):
        self.shouldStop = False
        while not self.shouldStop:
            parsedLook = self.__extractDataFormLookFunction()
            controlValue = self.stablizer.ctl(parsedLook[0], parsedLook[1])
            self.targetVal.append(parsedLook[0])
            self.controlVal.append(controlValue)
            self.FGOCommunicator.submitCommand(self.stabilizeTarget, controlValue)
            time.sleep(DiscretePIDController.defaultDuration)
    def stopStablizing(self):
        self.shouldStop = True

class FGCommunicator:
    def __init__(self, FGOController):
        self.FGOController = FGOController
        self.lookData = FGOController.look()
        self.commandQueue = queue.Queue()
        self.mutex = threading.Lock()
        self.shouldStop = True
    def __updateLook(self):
        while not self.shouldStop:
            self.mutex.acquire()
            self.lookData = self.FGOController.look()
            self.mutex.release()
            time.sleep(DiscretePIDController.defaultDuration)

    def __sendCommand(self):
        while not self.shouldStop:
            try:
                command = self.commandQueue.get(block=True, timeout = DiscretePIDController.defaultDuration)
            except queue.Empty:
                continue
            else:
                self.mutex.acquire()
                self.FGOController.setFGProp(command[0], command[1])
                self.mutex.release()

    def getCachedLook(self):
        return self.lookData
    def submitCommand(self, controller, value):
        self.commandQueue.put((controller, value))
    def communicate(self):
        if self.shouldStop:
            self.shouldStop = False;
            recvThread = threading.Thread(target=FGCommunicator.__updateLook, args=(self,))
            sendThread = threading.Thread(target=FGCommunicator.__sendCommand, args=(self,))
            recvThread.start()
            sendThread.start()
    def stopCommunicate(self):
        self.shouldStop = True
