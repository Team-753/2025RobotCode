import rev,wpilib,commands2
from RobotConfig import elevator

class elevatorSubSystem(commands2.subsystem):
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


class advElevatorSubsystem(commands2.subsystem):
    def __init__(self,myJoystick:commands2.button.CommandXboxController):
        lMotorID=elevator.leftMotorID
        rMotorID=elevator.rightMotorID
        self.joystick=myJoystick
        self.lMotor=rev.SparkMax(lMotorID,rev.SparkMax.MotorType.kBrushless)
        self.rMotor=rev.SparkMax(rMotorID,rev.SparkMax.MotorType.kBrushless)
        self.rMotor.follow(self.lMotor,True)
    def setSpeed(self):
        self.lMotor.set(self.joystick.getLeftY())

class posElevatorSubsystem(commands2.subsystem):
    def __init__(self):
        #gets ID
        lMotorID=elevator.leftMotorID
        rMotorID=elevator.rightMotorID
        #sets motor based off motorid
        self.lMotor=rev.SparkMax(lMotorID,rev.SparkMax.MotorType.kBrushless)
        self.rMotor=rev.SparkMax(rMotorID,rev.SparkMax.MotorType.kBrushless)
             #read it
        self.lMotor.ControlType.kPosition
        self.pid=self.lMotor.getClosedLoopController()
        self.encoder=self.lMotor.getAbsoluteEncoder()
        self.processVariable=self.encoder.getPosition()
    def setPosition(self,desiredPos):
        #i have as much confidence this code works as i have confidence we arent secretly ruled over by reptillian overlords
        self.pid.setReference(desiredPos, rev.SparkMax.ControlType.kPosition,0)