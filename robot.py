import commands2
from robotContainer import RobotContainer
from wpilib import SmartDashboard



class MyRobot(commands2.TimedCommandRobot):
    def __init__(self, period=.02):
        super().__init__(period)
        

    def robotInit(self):
        self.robotContainer = RobotContainer()
        SmartDashboard.setDefaultBoolean("Locked(1)", False)
        SmartDashboard.setDefaultBoolean("Locked(2)", False)

        

    def teleopPeriodic(self):
        pass


    def teleopInit(self):
        SmartDashboard.putBoolean("Locked(1)", False)
        SmartDashboard.putBoolean("Locked(2)", False)

          

    def disabledInit(self):
        SmartDashboard.putBoolean("Locked(1)", False)
        SmartDashboard.putBoolean("Locked(2)", False)
        return super().disabledInit()
    
    def autonomousInit(self):
        self.robotContainer.autonomousInit()
        self.autoCommand = self.robotContainer.getAutonomousCommand()
        self.autoCommand.schedule()
    
