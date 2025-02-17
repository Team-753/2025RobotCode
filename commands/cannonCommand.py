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

class DefaultPivotCommand(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem

    def execute(self):
        self.cannon.ManualControl(self.cannon.GetJoystickInput())
    def end(self, interrupted):
        self.cannon.angleStop()
        return super().end(interrupted)
class cannonToPosition(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
    def execute(self):
        self.cannon.goToPos(0.5)
        return super().execute()


    


