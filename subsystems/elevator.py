import rev,wpilib,commands2,wpimath
import wpimath.controller
from RobotConfig import elevator

class elevatorSubSystem(commands2.Subsystem):
    def __init__(self, auxController: commands2.button.CommandXboxController):
        #gets ID
        lMotorID=elevator.leftMotorID
        rMotorID=elevator.rightMotorID
        #sets motor based off motorid
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
        self.lMotor.IdleMode(1)
        self.rMotor.IdleMode(1)
        configL=rev.SparkMaxConfig()        
        configR=rev.SparkMaxConfig()
        configR.follow(lMotorID)
        self.rMotor.configure(configR, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.lMotor.configure(configL, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.encoderOffset=1-self.encoder.getPosition()
        self.encoderPos=0
        self.encoderRotations=0
        self.controller = auxController
        self.realEncoderPos = 0
        #read it
    def goUp(self):
        self.lMotor.set(0.13)
        self.rMotor.set(0.13)
    def goDown(self):
        self.lMotor.set(0)
        self.rMotor.set(0)
    def idle(self):
        self.lMotor.IdleMode(1)
        self.rMotor.IdleMode(1)
        
    def Brake(self):
        self.lMotor.set(.06)
        self.rMotor.set(.06)
        print("Breaking")

        #Gets the joystick y input of the avux controller
    def GetJoystickInput(self):
        return(-self.controller.getLeftY())
    
    def ManualControl(self, controllerInput):
        self.joystick = controllerInput
        if(abs(self.joystick) > .5):
            if(self.joystick < 0):
                #Lowers the elevator motors
                self.goDown()
                print("elevators moving down based on joystick")
            elif(self.joystick > 0):
                #sets the motor to raise
                self.goUp()
                print("Elevator going up based on joystick")
        elif(self.realEncoderPos < 1):
            self.idle()
        else:
            self.Brake()
    def setPosition(self,desiredPos):
        #i have as much confidence this code works as i have confidence we arent secretly ruled over by reptillian overlords
        #self.pid.setReference(desiredPos,rev.SparkMax.ControlType.kPosition)
        #print(self.encoder.getPosition(),desiredPos)
        myPid=wpimath.controller.PIDController(0.3,0.05,0.0,period=0.02)
        myPid.setIZone(0.3)
        myPid.setSetpoint(desiredPos)
        pidOut=myPid.calculate(measurement=self.realEncoderPos)
        self.lMotor.set(pidOut+0.1)
        print(pidOut)
        pass
    def getPosition(self):
        encoderPast=self.encoderPos
        self.encoderPos=(1-self.encoder.getPosition())-self.encoderOffset
        encoderDelta=float(self.encoderPos-encoderPast)
        self.realEncoderPos=self.encoderRotations+self.encoderPos
        print("my position",self.realEncoderPos)
        if abs(encoderDelta)>0.7:
            self.encoderRotations+=1*(-encoderDelta/abs(encoderDelta))

 