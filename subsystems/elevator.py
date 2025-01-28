import rev,wpilib,commands2
from RobotConfig import elevator

class elevatorSubSystem(commands2.subsystem):
    def __init__(self):
        lMotorID=elevator.leftMotorID
        rMotorID=elevator.rightMotorID
        self.lMotor=rev.CANSparkMax(lMotorID,rev.CANSparkMax.MotorType.kBrushless)
        self.rMotor=rev.CANSparkMax(rMotorID,rev.CANSparkMax.MotorType.kBrushless)
        self.rMotor.follow(self.lMotor,True)
    def goUp(self):
        self.lMotor.set(0.1)
    def goDown(self):
        self.lMotor.set(-0.1)
    def idle(self):
        self.lMotor.set(0)