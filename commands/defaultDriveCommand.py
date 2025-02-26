import commands2
from subsystems.drivetrain import DriveTrainSubSystem

class defaultDriveCommand(commands2.Command):
    def __init__(self, driveTrainSubSystem: DriveTrainSubSystem):
        super().__init__()
        self.addRequirements(driveTrainSubSystem)
        self.driveTrain = driveTrainSubSystem
        #print("default drive command is running.")

    def execute(self):
        #print("ahhh")
        self.driveTrain.joystickDrive(self.driveTrain.getJoystickInput())
        #print("calling the joystick inputs")

class SlowDown(commands2.Command):
    def __init__(self, driveTrainSubSystem: DriveTrainSubSystem):
        super().__init__()
        self.addRequirements(driveTrainSubSystem)
        self.driveTrain = driveTrainSubSystem

    def execute(self):
        self.driveTrain.halfSpeed()
        self.driveTrain.joystickDrive(self.driveTrain.getJoystickInput())
    
    def end(self, interrupted):
        self.driveTrain.fullSpeed()