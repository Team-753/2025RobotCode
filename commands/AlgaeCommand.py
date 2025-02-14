import commands2
from subsystems.algae import AlgaeSquisher


class FlipAlgaeSquisher(commands2.Command):
    def __init__(self, kalgaeSubsystem: AlgaeSquisher):
        super().__init__()
        self.addRequirements(kalgaeSubsystem)
        #The Boolean flip on whether or not we have extended the pistons
        self.hasExtended = False
        self.algae = kalgaeSubsystem
    def initialize(self):
        self.hasExtended = not self.hasExtended
        print(self.hasExtended)
        if(self.hasExtended):
            self.algae.ExtendPiston()
            print("algae go out")
        else:
            self.algae.PullPistonBack()
            print("algae comes back")
            
    def end(self, interrupted):
        print("algae moved")
        return super().end(interrupted)

#spins the motors in to pull in the algae
class GrabAlgae(commands2.Command):
    def __init__(self, kalgaeSubsystem: AlgaeSquisher):
        super.__init__()
        self.addRequirements(kalgaeSubsystem)
        self.algaeGrabber = kalgaeSubsystem
    
    def initialize(self):
        self.algaeGrabber.GrabAlgae()
    
    def execute(self):
        pass
    def end(self, interrupted):
        print("Grabbing algae")


#spins the motors the other way to release the algae
class ReleaseAlgae(commands2.Command):
    def __init__(self, kalgaeSubsystem: AlgaeSquisher):
        super.__init__()
        self.addRequirements(kalgaeSubsystem)
        self.algaeGrabber = kalgaeSubsystem
    
    def initialize(self):
        self.algaeGrabber.ReleaseAlgae()
    
    def execute(self):
        pass
    def end(self, interrupted):
        print("Releasing algae")