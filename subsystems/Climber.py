import wpilib
import commands2
import RobotConfig as rc

class ClimberSubsystem(commands2.Subsystem):
    def __init__(self):

        #assigns the double solenoids
        self.doubleSolenoid1 = wpilib.DoubleSolenoid(
                wpilib.PneumaticsModuleType.REVPH, 
                rc.Climber.piston1Forward, 
                rc.Climber.piston1Reverse)
        
        self.doubleSolenoid2 = wpilib.DoubleSolenoid(
                wpilib.PneumaticsModuleType.REVPH, 
                rc.Climber.piston2Forward, 
                rc.Climber.piston2Reverse)
      
        #Assigns the compressor 
        self.compressor = wpilib.Compressor(wpilib.PneumaticsModuleType.REVPH)

    def GoUp(self):
        #sets both solenoids to their extended position
        self.doubleSolenoid1.set(wpilib.DoubleSolenoid.Value.kForward)
        self.doubleSolenoid2.set(wpilib.DoubleSolenoid.Value.kForward)


    
    def GoDown(self):
        #sets both solenoids to their lowered position
        self.doubleSolenoid1.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.doubleSolenoid2.set(wpilib.DoubleSolenoid.Value.kReverse)

    def SwitchCompressor(self):
        #checks if the compressor is already on and turns it off
        if self.compressor.isEnabled():
            self.compressor.disable()
            
        else:
           #Turn on the compressor until the system is full
           self.compressor.enableDigital()