import commands2
from subsystems.limelight_camera import LimelightCamera


class Lock(commands2.Command):
    def __init__(self, kLimeLight: LimelightCamera):
        super().__init__()
        self.addRequirements(kLimeLight)
        self.limelight = kLimeLight

    def execute(self):
        print("Commencing Locking Sequence")


    
    def end(self, interrupted: bool):
        pass


       



