import commands2
from subsystems.Climber import ClimberSubsystem
import wpilib


#extends the pistons to climb the cage
class ExtendClimber(commands2.Command):
    def __init__(self, kclimberSubsystem: ClimberSubsystem):
        super.__init__()
        self.addRequirements(kclimberSubsystem)
        self.climber = kclimberSubsystem
        
    def initialize(self):
        self.climber.GoUp()
    
    def execute(self):
        pass
        
    def end(self, interrupted: bool):
        print("done extending")
        
        
        
#Releases the pistons from their extended position
class ReleaseClimber(commands2.Command):
    def __init__(self, kclimberSubsystem: ClimberSubsystem):
        super().__init__()
        self.climber = kclimberSubsystem
        
    def initialize(self):
        self.climber.GoDown()
        
    def execute(self):
        pass
        
    def end(self, interrupted: bool):
        print("done lowering")
        
        