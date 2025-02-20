import commands2
from subsystems.drivetrain import DriveTrainSubSystem
from wpimath import geometry
from wpilib import Timer

class superSimpleAuto(commands2.Command):
    def __init__(self, driveTrainSubSystem: DriveTrainSubSystem, directions: tuple, stopTime):
        super().__init__()
        self.addRequirements(driveTrainSubSystem)
        self.driveTrain = driveTrainSubSystem
        self.timer = Timer
        self.directions = directions
        self.endTime = stopTime
    
    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def execute(self):
        self.driveTrain.joystickDrive(self.directions)
    
    def isFinished(self):
        if self.timer.get() > self.endTime:
            return True
        
    def end(self, interrupted: bool):
        self.driveTrain.joystickDrive([0, 0, geometry.Rectangle2d()])
        self.timer.stop()

    
