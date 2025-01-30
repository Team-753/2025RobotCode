import commands2
from subsystems.cannon import CannonSubsystem
from wpilib import XboxController

class place(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        print("place command is running")

    def execute(self):
        self.cannon.place()
        print("placing")

    def end(self, interrupted: bool):
        self.cannon.idle()

class intake(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        print("intake command is running")

    def execute(self):
        self.cannon.intake()
        print("intaking")
        #add intake position function

    def end(self, interrupted: bool):
        self.cannon.idle()
        #add idle for position





    


