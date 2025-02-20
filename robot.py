import wpilib
import commands2

from wpilib.cameraserver import CameraServer
from robotContainer import RobotContainer
from wpilib import SmartDashboard



class MyRobot(commands2.TimedCommandRobot):
    def __init__(self, period=.02):
        super().__init__(period)
        pass
        

    def robotInit(self):
        self.robotContainer = RobotContainer()
        SmartDashboard.setDefaultBoolean("Locked(1)", False)
        SmartDashboard.setDefaultBoolean("Locked(2)", False)

        

    def teleopPeriodic(self):

        pass

    def teleopInit(self):
        SmartDashboard.putBoolean("Locked(1)", True)
        SmartDashboard.putBoolean("Locked(2)", True)

    def disabledInit(self):
        SmartDashboard.putBoolean("Locked(1)", False)
        SmartDashboard.putBoolean("Locked(2)", False)
        return super().disabledInit()