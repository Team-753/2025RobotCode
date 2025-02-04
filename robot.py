import wpilib
class MyRobot(wpilib.TimedRobot):
    def __init__(self, period = .02):
        super().__init__(period)
    
    def robotInit(self):

        self.xbox=wpilib.XboxController(1) #ryan said plake holder ahhh
        self.pneumatic = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.REVPH,0,1)
        self.pistonExtended = False
        self.compressorOn = False
        self.compressor = wpilib.Compressor(wpilib.PneumaticsModuleType.REVPH)

        #self.motor2.
    def teleopPeriodic(self):
        
        if self.xbox.getAButtonPressed():
            self.pistonExtended = not self.pistonExtended
            print(self.pistonExtended)
            if (self.pistonExtended):
                print("go piston")
                self.pneumatic.set(wpilib.DoubleSolenoid.Value.kForward)
            else:
                print("come back piston")
                self.pneumatic.set(wpilib.DoubleSolenoid.Value.kReverse)

        if self.xbox.getYButtonPressed():
            self.compressorOn = not self.compressorOn
            print(self.compressorOn)
            if (self.compressorOn):
                print("ON COMPRESSOR")
                self.compressor.enableDigital()
            else:
                print("OFF COMPRESSOR")
                self.compressor.disable()
            
            