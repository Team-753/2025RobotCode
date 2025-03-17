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

'''class elevatorToPos(commands2.Command):
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
            self.elevator.setPosition(self.desiredPos)'''

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
        elif self.desiredPos == 22:
            self.elevator.goToMax()
        else:
            self.elevator.setPosition(self.desiredPos)
    def isFinished(self):
        if float(self.elevator.getPosError())<0.2 and self.desiredPos!=0:
            print("elevatorcommandfinished")
            return True
        elif self.desiredPos==0 and self.elevator.checkBottom():
            return True
        else:
            return False