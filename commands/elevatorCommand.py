import commands2,wpilib
from subsystems.elevator import elevatorSubSystem,posElevatorSubsystem
class elevatorUp(commands2.Command,):
    def __init__(self, kElevSub:elevatorSubSystem):
        super().__init__()
        self.addRequirements
        self.eSub=kElevSub
    def execute(self):
        self.eSub.goUp
        return super().execute()
    def end(self, interrupted:bool):
        self.eSub.idle
class elevatorDown(commands2.Command,):
    def __init__(self, kElevSub:elevatorSubSystem):
        super().__init__()
        self.addRequirements
        self.eSub=kElevSub
    def execute(self):
        self.eSub.goDown
        return super().execute()
    def end(self, interrupted:bool):
        self.eSub.idle
class elevatorToPos(commands2.Command,):
    def __init__(self, kElevSub:posElevatorSubsystem):
        super().__init__()
        self.addRequirements
        self.eSub=kElevSub
        self.xbox=wpilib.XboxController(1)
    def execute(self):
        self.eSub.setPosition(self.xbox.getLeftY())
        return super().execute()

