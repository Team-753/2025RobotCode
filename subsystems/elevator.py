import rev,wpilib,commands2,wpimath,math
import wpimath.controller
from RobotConfig import elevator
def constrain(var,min,max):
    if var>max:
        return max
    elif var<min:
        return min
    else:
        return var
    
class elevatorSubSystem(commands2.Subsystem):
    def __init__(self):
        '''Chris, I think the way to get this to work is to use rev's built in PID controller
        Currently we are setting the desired position, but we might also be passing it as a speed.
        That will mess stuff up. the built in pid controller has a method that will make it a lot easier
        You can use different values in your PIDs fairly easily by using multiple slots - Sanika'''
        #gets ID
        lMotorID=elevator.leftMotorID
        rMotorID=elevator.rightMotorID
        #sets motor based off motorid
        self.lMotor=rev.SparkMax(lMotorID,rev.SparkMax.MotorType.kBrushless)
        self.rMotor=rev.SparkMax(rMotorID,rev.SparkMax.MotorType.kBrushless)
        #read it
        self.pid=self.lMotor.getClosedLoopController()
        self.encoder=self.lMotor.getEncoder()
        self.absEncoder=self.lMotor.getAbsoluteEncoder()
        configL=rev.SparkMaxConfig()       
        configR=rev.SparkMaxConfig()
        configR.follow(lMotorID)
        configL.closedLoop.pid(0.05,0.0000001,0.01,slot=rev.ClosedLoopSlot.kSlot0)
        configL.closedLoop.IMaxAccum(0.1,slot=rev.ClosedLoopSlot.kSlot0)
        configL.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        self.encoder.setPosition(0)
        self.rMotor.configure(configR, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.lMotor.configure(configL, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.lMotor.IdleMode(0)
        self.rMotor.IdleMode(0)
    def checkBottom(self):
        if self.encoder.getPosition()<0.7:
            self.lMotor.IdleMode(0)
            self.lMotor.set(0)
            #print("MOTOR IDLE")
    def setPosition(self,desiredPos):
        self.desiredPos=desiredPos
        self.pid.setReference(self.desiredPos,rev.SparkMax.ControlType.kPosition,rev.ClosedLoopSlot.kSlot0,1.2)
        print("ELEVATING")
    def goToZero(self):
        if self.encoder.getPosition()<0.7:
                self.lMotor.set(0)
                #print("MOTOR IDLE")
        elif self.encoder.getPosition()<4 and abs(self.encoder.getVelocity())<150:
            self.lMotor.set(0.023)
            #print("AT POS")
        else:
            self.pid.setReference(4,rev.SparkMax.ControlType.kPosition,rev.ClosedLoopSlot.kSlot0)
    def goToMax(self):
        if self.encoder.getPosition()>23:
                self.lMotor.set(0.04)
                #print("MOTOR IDLE")
        elif self.encoder.getPosition()>22 and abs(self.encoder.getVelocity())<150:
            self.lMotor.set(0.05)
            #print("AT POS")
        else:
            self.pid.setReference(23,rev.SparkMax.ControlType.kPosition,rev.ClosedLoopSlot.kSlot0)
    def goUp(self):
        #if not self.encoder.getPosition()>26:
        self.lMotor.set(0.15)
        print(self.encoder.getPosition())
    def goDown(self):
        self.lMotor.set(0.017)
        print(self.encoder.getPosition())
    def holdPos(self):
        self.desiredPos=self.encoder.getPosition()
        self.pid.setReference(self.desiredPos,rev.SparkMax.ControlType.kPosition,rev.ClosedLoopSlot.kSlot0)
    def constantUp(self):
        self.lMotor.IdleMode(0)
        self.lMotor.set(0.06)
    def getPosError(self):
        return (self.desiredPos()-self.encoder.getPosition())


 