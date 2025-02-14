import wpilib
import commands2
import RobotConfig as rc

class ClimberSubsystem(commands2.Subsystem):
    def __init__(self):

        #assigns the double solenoids
        self.pneumaticsHub = wpilib.PneumaticHub(rc.Climber.pneumaticsHubID)
        self.climber = self.pneumaticsHub.makeDoubleSolenoid(rc.Climber.solenoidForward, rc.Climber.solenoidReverse)

        self.algaeSquisher = self.pneumaticsHub.makeDoubleSolenoid(rc.algaeSquisher.squisherPistonForward, rc.algaeSquisher.squisherPistonReverse)
      
        #Assigns the compressor 
        self.compressor = self.pneumaticsHub.makeCompressor()

    def GoUp(self):
        #sets both solenoids to their extended position
        self.climber.set(wpilib.DoubleSolenoid.Value.kForward)


    
    def GoDown(self):
        #sets both solenoids to their lowered position
        self.climber.set(wpilib.DoubleSolenoid.Value.kReverse)
    
    def GoOut(self):
        self.algaeSquisher.set(wpilib.DoubleSolenoid.Value.kForward)
    def ComeBack(self):
        self.algaeSquisher.set(wpilib.DoubleSolenoid.Value.kReverse)


    def CompressorOn(self):
        self.compressor.enableDigital()
    
    def CompressorOff(self):
        self.compressor.disable()
