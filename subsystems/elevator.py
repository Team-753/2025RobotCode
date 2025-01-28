import rev,wpilib,commands2
from RobotConfig import elevator

class elevatorSubSystem(commands2.subsystem):
    def __init__(self):
        #gets ID
        lMotorID=elevator.leftMotorID
        rMotorID=elevator.rightMotorID
        #sets motor based off motorid
        self.lMotor=rev.CANSparkMax(lMotorID,rev.CANSparkMax.MotorType.kBrushless)
        self.rMotor=rev.CANSparkMax(rMotorID,rev.CANSparkMax.MotorType.kBrushless)
        #read it
        self.rMotor.follow(self.lMotor,True)
    def goUp(self):
        self.lMotor.set(0.1)
    def goDown(self):
        self.lMotor.set(-0.1)
    def idle(self):
        self.lMotor.set(0)


class advElevatorSubsystem(commands2.subsystem):
    def __init__(self,myJoystick:commands2.button.CommandXboxController):
        lMotorID=elevator.leftMotorID
        rMotorID=elevator.rightMotorID
        self.joystick=myJoystick
        self.lMotor=rev.CANSparkMax(lMotorID,rev.CANSparkMax.MotorType.kBrushless)
        self.rMotor=rev.CANSparkMax(rMotorID,rev.CANSparkMax.MotorType.kBrushless)
        self.rMotor.follow(self.lMotor,True)
    def setSpeed(self):
        self.lMotor.set(self.joystick.getLeftY())

class posElevatorSubsystem(commands2.subsystem):
    def __init__(self):
        #gets ID
        lMotorID=elevator.leftMotorID
        rMotorID=elevator.rightMotorID
        encoderID=elevator.encoderID
        #sets motor based off motorid
        self.lMotor=rev.CANSparkMax(lMotorID,rev.CANSparkMax.MotorType.kBrushless)
        self.rMotor=rev.CANSparkMax(rMotorID,rev.CANSparkMax.MotorType.kBrushless)
        
        self.encoder=wpilib.Encoder(encoderID)#!!!!!!!!INEEDTOFINDTHEENCODERTYPEOROURROBOTWILLEXPLODEANDTHESEASONWILLBEOVER!!!!!!!!!!!!!!!!!!!!!
        self.lMotor.ControlType(rev.CANSparkMax.ControlType.kPosition)
        #read it
        self.rMotor.follow(self.lMotor,True)
    def setPosition(self,desiredPos):
        self.lMotor.set()#something)