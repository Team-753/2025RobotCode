import commands2,wpilib
from subsystems.elevator import elevatorSubSystem
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


