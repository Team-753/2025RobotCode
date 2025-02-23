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
        configL.closedLoop.pid(0.1,0,0,slot=rev.ClosedLoopSlot.kSlot0)
        configL.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        self.rMotor.configure(configR, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.lMotor.configure(configL, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.lMotor.IdleMode(0)
        self.rMotor.IdleMode(0)
    
    def setPosition(self,desiredPos):
        self.desiredPos=desiredPos
        self.pid.setReference(self.desiredPos,rev.SparkMax.ControlType.kPosition,rev.ClosedLoopSlot.kSlot0)
        print(self.encoder.getPosition(),self.desiredPos)
    def goUp(self):
        self.lMotor.set(0.1)
    def goDown(self):
        self.lMotor.set(0)
    def holdPos(self):
        self.desiredPos=self.encoder.getPosition()
        self.pid.setReference(self.desiredPos,rev.SparkMax.ControlType.kPosition,rev.ClosedLoopSlot.kSlot0)
    def idle(self):
        self.lMotor.IdleMode(0)
        self.lMotor.set(0)


 