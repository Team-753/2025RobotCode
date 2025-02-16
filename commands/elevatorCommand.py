import commands2 ,wpilib
from subsystems.elevator import elevatorSubSystem,posElevatorSubsystem


class elevatorUp(commands2.Command):
    def __init__(self, kElevSub:elevatorSubSystem):
        super().__init__()
        self.addRequirements(kElevSub)
        self.eSub=kElevSub
    def execute(self):
        self.eSub.goUp()
        print("Going down")
    def end(self, interrupted):
        self.eSub.idle()
    
    
class elevatorDown(commands2.Command):
    def __init__(self, kElevSub:elevatorSubSystem):
        super().__init__()
        self.addRequirements(kElevSub)
        self.eSub=kElevSub
    def execute(self):
        self.eSub.goDown()
        print("Going up")
        
    def end(self, interrupted):
        self.eSub.idle()
    

#maybe works
class elevatorToPos(commands2.Command):
    def __init__(self, kElevSub:posElevatorSubsystem,desiredPosition):
        super().__init__()
        self.addRequirements(kElevSub)
        self.eSub=kElevSub
        self.desPos=desiredPosition
    def execute(self):
        self.eSub.setPosition(self.desPos)
        self.eSub.getPosition()
        print(self.desPos)
        return super().execute()

class DefaultElevatorCommand(commands2.Command):
    def __init__(self, elevatorSubSystem: elevatorSubSystem):
        super().__init__()
        self.addRequirements(elevatorSubSystem)
        self.elevator = elevatorSubSystem
        #print("default elevator command Running.")

    def execute(self):
        #print("uh")
        self.elevator.ManualControl(self.elevator.GetJoystickInput())
        #print("manually driving elevator")
    def end(self, interrupted):
        self.elevator.Brake()
        return super().end(interrupted)
