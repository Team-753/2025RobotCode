import wpilib
import commands2
import RobotConfig as rc

class ClimberSubSystem(commands2.Subsystem):
    def __init__(self):

        #assigns the double solenoids
        self.doubleSolenoid1 = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, rc.Climber.piston1Forward, rc.Climber.piston1Reverse)
        #self.doubleSolenoid1.set(wpilib.DoubleSolenoid.Value.kOff)

        self.doubleSolenoid2 = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, rc.Climber.piston2Forward, rc.Climber.piston2Reverse)
        #self.doubleSolenoid2.set(wpilib.DoubleSolenoid.Value.kOff)


        
        #Assigns the compmressor 
        self.compressor = wpilib.Compressor(wpilib.PneumaticsModuleType.CTREPCM)

    def GoUp(self):
        #sets both solenoids to their extended position
        self.doubleSolenoid1.set(wpilib.DoubleSolenoid.Value.kForward)
        self.doubleSolenoid2.set(wpilib.DoubleSolenoid.Value.kForward)


    
    def GoDown(self):
        #sets both solenoids to their lower position
        self.doubleSolenoid1.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.doubleSolenoid2.set(wpilib.DoubleSolenoid.Value.kReverse)

    def SwitchCompressor(self):
        #checks if the compressor is already on and turns it off
        self.isCompressorEnabled = self.compressor.isEnabled()
        if self.isCompressorEnabled:
            self.compressor.disable()
        else:
           #Turn on the compressor as long as it doesn't explode
           self.compressor.enableDigital()