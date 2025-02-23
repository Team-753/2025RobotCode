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
        self.encoder=self.lMotor.getAbsoluteEncoder()
        self.processVariable=self.encoder.getPosition()
        configL=rev.SparkMaxConfig()        
        configR=rev.SparkMaxConfig()
        configR.follow(lMotorID)
        self.rMotor.configure(configR, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.lMotor.configure(configL, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.lMotor.IdleMode(1)
        self.rMotor.IdleMode(1)
        self.encoderPos=0
        self.encoderRotations=0
        self.realEncoderPos = 0
        self.encoderOffset = self.encoder.getPosition()     #read it
        self.desiredPos=0
    def setPosition(self,desiredPos):
        self.desiredPos=desiredPos
    def elevatorPid(self):
        #UPDATE ENCODER POS
        encoderPast=self.encoderPos
        self.encoderPos=(1-(self.encoder.getPosition()-self.encoderOffset))
        encoderDelta=float(self.encoderPos-encoderPast)
        self.realEncoderPos=self.encoderRotations+self.encoderPos
        if abs(encoderDelta)>0.7:
            self.encoderRotations+=1*(-encoderDelta/abs(encoderDelta))
        #RUN ELEVATOR PID
        myPid=wpimath.controller.PIDController(0.3,0.05,0.00,period=0.02)
        myPid.setIZone(0.2)
        self.desiredPos=constrain(self.desiredPos,0,3.9)
        myPid.setSetpoint(self.desiredPos)
        #DECELELELELRATE ELEVATOR ON DOWN
        pidOut=myPid.calculate(measurement=self.realEncoderPos)
        if (pidOut+0.08)<0:
            #pidOut=pidOut*constrain(abs(self.realEncoderPos-self.desiredPos),0.1,1)
            #myPid.setP(constrain(abs(self.realEncoderPos-self.desiredPos)*0.6,0.1,0.3))
            pass
        print("elevator position",self.realEncoderPos,pidOut,self.desiredPos)
        #pidOut=constrain(pidOut,-0.1,0.2)
        if self.desiredPos<0.1 and self.realEncoderPos<0.1:
            self.lMotor.IdleMode(rev.SparkMax.IdleMode.kCoast)
            #self.rMotor.IdleMode(rev.SparkMax.IdleMode.kCoast)
            self.lMotor.set(0)
            #self.rMotor.set(0)
        else: 
            self.lMotor.set(pidOut+0.08)
    def goUp(self):
        self.desiredPos=self.desiredPos+0.03
        print("Going up",self.desiredPos)
    def goDown(self):
        self.desiredPos=self.desiredPos-0.03
        print("going down")
    def idle(self):
        self.lMotor.IdleMode(0)
        #self.rMotor.IdleMode(1)
        
    def Brake(self):
        #print("Breaking")
        pass
        #Gets the joystick y input of the avux controller



 