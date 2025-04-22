import commands2
from subsystems.drivetrain import DriveTrainSubSystem
from wpimath import geometry

class DefaultDriveCommand(commands2.Command):
    """class that wraps our driving inputs into a command"""
    def __init__(self, driveTrainSubSystem: DriveTrainSubSystem):
        super().__init__()
        self.addRequirements(driveTrainSubSystem) #I dont actually know what this does, but the code breaks if we dont do it, so we have it 
        self.driveTrain = driveTrainSubSystem # gotta be able to reference the drive train other placees in this class
        #print("default drive command is running.")

    def execute(self):
        #print("ahhh")
        self.driveTrain.joystickDrive(self.driveTrain.getJoystickInputCurved()) #actually drives the robot using our more complicated proccessing function
        #print("calling the joystick inputs")

class SlowDown(commands2.Command):
    #this was an overcomplication, it should never run. IGNORE IT
    def __init__(self, driveTrainSubSystem: DriveTrainSubSystem):
        super().__init__()
        self.addRequirements(driveTrainSubSystem)
        self.driveTrain = driveTrainSubSystem

    def execute(self):
        self.driveTrain.halfSpeed()
        self.driveTrain.joystickDrive(self.driveTrain.getJoystickInput())
    
    def end(self, interrupted):
        self.driveTrain.fullSpeed()