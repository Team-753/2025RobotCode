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
        config=rev.SparkMaxConfig()
        config.follow(lMotorID)
        self.rMotor.configure(config, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
    def setPosition(self,desiredPos):
        #i have as much confidence this code works as i have confidence we arent secretly ruled over by reptillian overlords
        self.pid.setReference(desiredPos,rev.SparkMax.ControlType.kPosition)