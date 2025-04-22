import commands2
from robotContainer import RobotContainer
from wpilib import SmartDashboard
from subsystems.limelight_camera import LimelightCamera



class MyRobot(commands2.TimedCommandRobot):
    def __init__(self, period=.02):
        super().__init__(period)
        
        #self.camera = LimelightCamera("limelight-jamal")  # name of your camera goes in parentheses

    def robotInit(self):
        """Things to do on code startup"""
        self.robotContainer = RobotContainer() #creates the robot container class
        #SmartDashboard.setDefaultBoolean("Locked(1)", False)
        #SmartDashboard.setDefaultBoolean("Locked(2)", False)

        

    def teleopPeriodic(self):
        
        '''if self.camera.hasDetection() == True:
            SmartDashboard.putBoolean("Locked(1)", True)
            SmartDashboard.putBoolean("Locked(2)", True)
        else :
            SmartDashboard.putBoolean("Locked(1)", False)
            SmartDashboard.putBoolean("Locked(2)", False)'''


    def teleopInit(self):
        """Things to do at the beginning of the teleop period"""
        self.robotContainer.teleopInit() #does whatever we told it to at the beginning of teleoop in the robot container

          

    def disabledInit(self):
        '''SmartDashboard.putBoolean("Locked(1)", False)
        SmartDashboard.putBoolean("Locked(2)", False)'''
        self.robotContainer.disabledInit()
    
    def autonomousInit(self):
        """Code run at the start of auto. This currently figures out what the robot is doing for auto"""
        '''if self.camera.hasDetection() == True:
            SmartDashboard.putBoolean("Locked(1)", True)
            SmartDashboard.putBoolean("Locked(2)", True)
        else :
            SmartDashboard.putBoolean("Locked(1)", False)
            SmartDashboard.putBoolean("Locked(2)", False)'''
        
        
        self.robotContainer.autonomousInit() #run the code in the robot container outlining what to do
        self.autoCommand = self.robotContainer.getAutonomousCommand() #Get the command to run from shuffleboard
        self.autoCommand.schedule() #runs the command
    
