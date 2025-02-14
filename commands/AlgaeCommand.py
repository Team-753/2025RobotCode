import commands2
from subsystems.algae import AlgaeSquisher
from subsystems.Climber import ClimberSubsystem


class FlipAlgaeSquisher(commands2.Command):
    def __init__(self, kClimberSubsystem: ClimberSubsystem):
        super().__init__()
        self.addRequirements(kClimberSubsystem)
        #The Boolean flip on whether or not we have extended the pistons
        self.hasExtended = False
        self.climber = kClimberSubsystem
    def initialize(self):
        self.hasExtended = not self.hasExtended
        print(self.hasExtended)
        if(self.hasExtended):
            self.climber.GoOut()
            print("algae go out")
        else:
            self.climber.ComeBack()
            print("algae comes back")
            
    def end(self, interrupted):
        print("algae moved")
        return super().end(interrupted)

#spins the motors in to pull in the algae
class GrabAlgae(commands2.Command):
    def __init__(self, kalgaeSubsystem: AlgaeSquisher):
        super().__init__()
        self.addRequirements(kalgaeSubsystem)
        self.algaeGrabber = kalgaeSubsystem
    
    def execute(self):
        self.algaeGrabber.IntakeAlgae()
        print("Grabbing algae")


    def end(self, interrupted):
        self.algaeGrabber.stop()
        print("grabbed algae")


#spins the motors the other way to release the algae
class ReleaseAlgae(commands2.Command):
    def __init__(self, kalgaeSubsystem: AlgaeSquisher):
        super().__init__()
        self.addRequirements(kalgaeSubsystem)
        self.algaeGrabber = kalgaeSubsystem
    
    def execute(self):
        self.algaeGrabber.ReleaseAlgae()
        print("Releasing ALgae")
    def end(self, interrupted):
        self.algaeGrabber.stop()
        print("released algae")