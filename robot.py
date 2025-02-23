import wpilib
import commands2
from robotContainer import RobotContainer
from subsystems.Climber import ClimberSubsystem
class MyRobot(commands2.TimedCommandRobot):
    def __init__(self, period = .02):
        super().__init__(period)
    
    def robotInit(self):
        self.robotContainer = RobotContainer()
        
    def teleopPeriodic(self):
        pass
    def teleopInit(self):
        pass

    def disabledInit(self):
        self.robotContainer.disabledInit()
            
            