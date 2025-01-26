import rev,wpilib,commands2
from RobotConfig import elevator

class elevatorSubSystem(commands2.subsystem):
    def __init__(self):
        self.lMotorID=elevator.leftMotorID
        self.rMotorID=elevator.rightMotorID