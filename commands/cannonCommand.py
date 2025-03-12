import commands2
from subsystems.cannon import CannonSubsystem
from wpilib import XboxController, Timer

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
        #return super().end(interrupted)

class intake(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        #print("intake command is running")

    def execute(self):
        self.cannon.intake()
        print("intaking")
        #add intake position function

    def end(self, interrupted: bool):
        self.cannon.idle()
        
        #return super().end(interrupted)
        #add idle for position
        
class PivotUp(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        #print("intake command is running")

    def execute(self):
        self.cannon.spinup()
        #add intake position function

    def end(self, interrupted: bool):
        self.cannon.angleIdle()
        
        
class PivotDown(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        #print("intake command is running")

    def execute(self):
        self.cannon.spindown()
        #add intake position function

    def end(self, interrupted: bool):
        self.cannon.angleIdle()
        
        
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
    def __init__(self, cannonSubsystem: CannonSubsystem, desPos):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        self.desiredPosition = desPos

    def execute(self):
        self.cannon.goToPos(self.desiredPosition)
        print("stuff")
        return super().execute()
    
    def end(self, interrupted):
        self.cannon.angleIdle()

class TopAlgaeRemoval(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem

    def execute(self):
        self.cannon.topAlgaeRemoval()
        print("removing algae")
        #self.cannon.goToPos(0.2)

    def end(self, interrupted):
        self.cannon.idle
        #self.cannon.angleStop


class BottomAlgaeRemoval(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem

    def execute(self):
        self.cannon.bottomAlgaeRemoval()
        print("removing algae")
        #self.cannon.goToPos(0.13)

    def end(self, interrupted):
        self.cannon.idle
        #self.cannon.angleStop

class AutoCannonPosition(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem, stopTime):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        self.timer = Timer()
        self.endTime = stopTime

    def initialize(self):
        self.timer.reset()
        self.timer.start()
        
    def execute(self):
        self.cannon.goToPos(0.133)

    def isFinished(self):
        if self.timer.hasElapsed(self.endTime):
            return True
        self.timer.reset

    def end(self, interrupted: bool):
        self.timer.stop
        

class AutoPlace(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem, stopTime):
        super().__init__()
        self.addRequirements(CannonSubsystem)
        self.cannon = CannonSubsystem
        self.timer = Timer()
        self.endTime = stopTime

    def initialize(self):
        self.timer.reset
        self.timer.start

    def execute(self):
        self.cannon.place

    def isFinished(self):
        if self.timer.hasElapsed(self.endTime):
            return True
        self.timer.reset

    def end(self, interrupted: bool):
        self.timer.stop


class AutoIntake(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem, stopTime):
        super().__init__()
        self.addRequirements(CannonSubsystem)
        self.cannon = cannonSubsystem
        self.timer = Timer()
        self.endTime = stopTime

    def initialize(self):
        self.timer.reset
        self.timer.start

    def execute(self):
        self.cannon.intake

    def isFinished(self):
        if self.timer.hasElapsed(self.endTime):
            return True
        self.timer.reset

    def end(self, interrupted: bool):
        self.timer.stop


class AutoTroughPlace(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem, stopTime):
        super().__init__()
        self.addRequirements(CannonSubsystem)
        self.cannon = CannonSubsystem
        self.timer = Timer()
        self.endTime = stopTime

    def initialize(self):
        self.timer.reset
        self.timer.start

    def execute(self):
        self.cannon.slowPlace

    def isFinished(self):
        if self.timer.hasElapsed(self.endTime):
            return True
        self.timer.reset

    def end(self, interrupted: bool):
        self.timer.stop




    


