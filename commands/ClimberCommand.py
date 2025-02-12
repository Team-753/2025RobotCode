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
        self.addRequirements(kclimberSubsystem)
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
        self.addRequirements(kclimberSubsystem)
        #The Boolean flip on whether or not we have extended the pistons
        self.hasExtended = False
        self.climber = kclimberSubsystem
    def initialize(self):
        self.hasExtended = not self.hasExtended
        print(self.hasExtended)
        if(self.hasExtended):
            self.climber.GoUp()
            print("Climb")
        else:
            self.climber.GoDown()
            print("Unclimb")

    def execute(self):
        pass

    def end(self, interrupted):
        print("Has moved pistons")
    

class FlipCompressor(commands2.Command):
    def __init__(self, kclimberSubsystem: ClimberSubsystem):
        super().__init__()
        self.addRequirements(kclimberSubsystem)
        self.hasTurnedOn = False
        #The Boolean flip on whether or not we have turned on the compressor
        self.climber = kclimberSubsystem
    def initialize(self):
        self.hasTurnedOn = not self.hasTurnedOn
        print(self.hasTurnedOn)
        if(self.hasTurnedOn):
            self.climber.CompressorOn()
            print("Compressor On")
        else:
            self.climber.CompressorOff()
            print("Compressor Off")

    def execute(self):
        pass

    def end(self, interrupted):
        print("Has flipped compressor")
    

        