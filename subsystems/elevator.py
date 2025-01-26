import rev,wpilib,commands2
from RobotConfig import elevator

class elevatorSubsystem(commands2.subsystem):
    def __init__(self):
        lMotorID=elevator.leftMotorID
        rMotorID=elevator.rightMotorID
        self.lMotor=rev.CANSparkMax(lMotorID,rev.CANSparkMax.MotorType.kBrushless)
        self.rMotor=rev.CANSparkMax(rMotorID,rev.CANSparkMax.MotorType.kBrushless)
        self.lMotor.follow(self.rMotor,True)
    def axisControl(self,axisIn):
        self.rMotor.set(axisIn)