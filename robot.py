import commands2
from subsystems.limelight_camera import LimelightCamera
from robotContainer import RobotContainer
from wpilib import SmartDashboard



class MyRobot(commands2.TimedCommandRobot):
    def __init__(self, period=.02):
        super().__init__(period)

        

        self.camera = LimelightCamera("limelight-jamal")  # name of your camera goes in parentheses
        

    def robotInit(self):
        self.robotContainer = RobotContainer()
        SmartDashboard.setDefaultBoolean("Locked(1)", False)
        SmartDashboard.setDefaultBoolean("Locked(2)", False)

        

    def teleopPeriodic(self):

        if self.camera.hasDetection == True:
            SmartDashboard.putBoolean("Locked(1)", True)
            SmartDashboard.putBoolean("Locked(2)", True)
        else :
            SmartDashboard.putBoolean("Locked(1)", False)
            SmartDashboard.putBoolean("Locked(2)", False)


    def teleopInit(self):
        SmartDashboard.putBoolean("Locked(1)", False)
        SmartDashboard.putBoolean("Locked(2)", False)

          

    def disabledInit(self):
        SmartDashboard.putBoolean("Locked(1)", False)
        SmartDashboard.putBoolean("Locked(2)", False)
        return super().disabledInit()
    
    def autonomousInit(self):
        self.autoCommand = self.robotContainer.getAutonomousCommand()
        self.autoCommand.schedule()
    
