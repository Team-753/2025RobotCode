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
        
        pidConfig=rev.ClosedLoopConfig()
        pidConfig.P=0.1
        pidConfig.I=0
        pidConfig.D=0
        
        configL=rev.SparkMaxConfig()
        configL.setIdleMode(rev.SparkMax.IdleMode.kBrake)
        
        configR=rev.SparkMaxConfig()
        configR.follow(lMotorID)
        configR.setIdleMode(rev.SparkMax.IdleMode.kBrake)
        self.lMotor.configure(configL, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.rMotor.configure(configR, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
    def setPosition(self,desiredPos):
        #i have as much confidence this code works as i have confidence we arent secretly ruled over by reptillian overlords
        self.pid.setReference(desiredPos,rev.SparkMax.ControlType.kPosition)