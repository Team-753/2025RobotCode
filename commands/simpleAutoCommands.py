import commands2
from subsystems.drivetrain import DriveTrainSubSystem
from wpimath import geometry
from wpilib import Timer

class superSimpleAuto(commands2.Command):
    def __init__(self, driveTrainSubSystem: DriveTrainSubSystem, directions: tuple, stopTime):
        super().__init__()
        self.addRequirements(driveTrainSubSystem)
        self.driveTrain = driveTrainSubSystem
        self.timer = Timer()
        self.directions = directions
        self.endTime = stopTime
    
    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def execute(self):
        self.driveTrain.joystickDrive(self.directions)
        print("running auto")
    
    def isFinished(self):
        if self.timer.hasElapsed(self.endTime):
            return True
        self.timer.reset()
        
    def end(self, interrupted: bool):
        #self.driveTrain.joystickDrive([0, 0, 0])
        self.timer.stop()

    
