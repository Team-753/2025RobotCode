import commands2
from subsystems.Climber import ClimberSubsystem
import wpilib
import RobotConfig


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


#------------------------------------------

#Flips the pistons on an on and off position 

class FlipClimber(commands2.Command):
    def __init__(self, kclimberSubsystem: ClimberSubsystem):
        super().__init__()

        #The Boolean flip on whether or not we have extended the pistons
        self.hasExtended = RobotConfig.Climber.pistonsHaveExtended

        self.climber = kclimberSubsystem
    def initialize(self):
        if(not self.hasExtended):
            self.climber.GoUp()
            self.hasExtended = True
        else:
            self.climber.GoDown()
            self.hasExtended = False
    def execute(self):
        pass
    def end(self, interrupted):
        print("Has moved pistons")
    
        
        