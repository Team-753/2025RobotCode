import commands2,wpilib
from subsystems.elevator import elevatorSubSystem


class elevatorUp(commands2.Command):
    def __init__(self, kElevSub:elevatorSubSystem):
        super().__init__()
        self.addRequirements(kElevSub)
        self.eSub=kElevSub
    def execute(self):
        self.eSub.goUp()
        #print("Going down")
    def end(self, interrupted):
        #self.eSub.holdPos()
        self.eSub.constantUp()
        self.eSub.checkBottom()
        pass
    
    
class elevatorDown(commands2.Command):
    def __init__(self, kElevSub:elevatorSubSystem):
        super().__init__()
        self.addRequirements(kElevSub)
        self.eSub=kElevSub
    def execute(self):
        self.eSub.goDown()
        
        #print("Going up")
        
    def end(self, interrupted):
        #self.eSub.holdPos()
        self.eSub.constantUp()
        self.eSub.checkBottom()
        pass

class elevatorToPos(commands2.Command):
    def __init__(self, elevatorSubSystem: elevatorSubSystem, desPos):
        super().__init__()
        self.addRequirements(elevatorSubSystem)
        self.elevator = elevatorSubSystem
        self.desiredPos=desPos
    def execute(self):
        if self.desiredPos==0:
            self.elevator.goToZero()
            self.elevator.checkBottom()
        else:
            self.elevator.setPosition(self.desiredPos)