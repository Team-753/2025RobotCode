import commands2
from subsystems.algae import AlgaeSquisher

#extends the position of the piston
class ExtendPiston(commands2.Command):  
    def __init__(self, kalgaeSubsystem: AlgaeSquisher):
        super.__init__()
        self.addRequirements(kalgaeSubsystem)
        self.algaeGrabber = kalgaeSubsystem
    
    def initialize(self):
        self.algaeGrabber.ExtendPiston()
    
    def execute(self):
        pass
    def end(self, interrupted):
        print("Extending piston")

#retracts the piston

class RetractPiston(commands2.Command):  
    def __init__(self, kalgaeSubsystem: AlgaeSquisher):
        super.__init__()
        self.addRequirements(kalgaeSubsystem)
        self.algaeGrabber = kalgaeSubsystem
    
    def initialize(self):
        self.algaeGrabber.PullPistonBack()
    
    def execute(self):
        pass
    def end(self, interrupted):
        print("Piston retracted")


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