import commands2
from subsystems.drivetrain import DriveTrainSubSystem
from wpimath import geometry

class DefaultDriveCommand(commands2.Command):
    def __init__(self, driveTrainSubSystem: DriveTrainSubSystem):
        super().__init__()
        self.addRequirements(driveTrainSubSystem)
        self.driveTrain = driveTrainSubSystem
        #print("default drive command is running.")

    def execute(self):
        #print("ahhh")
        self.driveTrain.joystickDrive(self.driveTrain.getJoystickInput())
        #print("calling the joystick inputs")