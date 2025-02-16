import rev,wpilib,commands2
from RobotConfig import elevator

class elevatorSubSystem(commands2.Subsystem):
    def __init__(self):
        #gets ID
        lMotorID=elevator.leftMotorID
        rMotorID=elevator.rightMotorID
        #sets motor based off motorid
        self.lMotor=rev.SparkMax(lMotorID,rev.SparkMax.MotorType.kBrushless)
        self.rMotor=rev.SparkMax(rMotorID,rev.SparkMax.MotorType.kBrushless)
        config=rev.SparkMaxConfig()
        config.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        self.lMotor.configure(config,rev.SparkMax.ResetMode.kNoResetSafeParameters,rev.SparkMax.PersistMode.kNoPersistParameters)
        self.rMotor.configure(config,rev.SparkMax.ResetMode.kNoResetSafeParameters,rev.SparkMax.PersistMode.kNoPersistParameters)
        #read it
    def goUp(self):
        self.lMotor.set(0.1)
        self.rMotor.set(0.1)
    def goDown(self):
        self.lMotor.set(-0.1)
        self.rMotor.set(-0.1)
    def idle(self):
        self.lMotor.set(0)
        self.rMotor.set(0)
    def Brake(self):
        self.lMotor.IdleMode(1)
        self.rMotor.IdleMode(1)

        #Gets the joystick y input of the aux controller
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


class posElevatorSubsystem(commands2.Subsystem):
    def __init__(self):
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
        configL.setIdleMode(rev.SparkMax.IdleMode.kBrake)
        configL.closedLoop.positionWrappingEnabled(True)
        configL.closedLoop.P=0.1
        configL.closedLoop.I=0
        configL.closedLoop.IMaxAccum(0.5)
        configL.closedLoop.D=0
        configL.absoluteEncoder.positionConversionFactor(6)
        configR=rev.SparkMaxConfig()
        configR.follow(lMotorID)
        configR.setIdleMode(rev.SparkMax.IdleMode.kBrake)
        self.rMotor.configure(configR, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.lMotor.configure(configL, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.encoderOffset=self.encoder.getPosition()
        self.encoderPos=0
        self.encoderRotations=0
    def setPosition(self,desiredPos):
        #i have as much confidence this code works as i have confidence we arent secretly ruled over by reptillian overlords
        #self.pid.setReference(desiredPos,rev.SparkMax.ControlType.kPosition)
        print(self.encoder.getPosition(),desiredPos)
    def getPosition(self):
        encoderPast=self.encoderPos
        self.encoderPos=self.encoder.getPosition()-self.encoderOffset
        encoderDelta=self.encoderPos-encoderPast
        if ((encoderDelta^2)^(1/2))>0.7:
            print(encoderDelta,self.encoderRotations)
            self.encoderRotations+=1*(encoderDelta/abs(encoderDelta))

