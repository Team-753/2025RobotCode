from subsystems.drivetrain import DriveTrainSubSystem
from commands.defaultDriveCommand import DefaultDriveCommand
import commands2
import wpilib
import os
from wpimath import geometry, kinematics, estimator
import RobotConfig as config

class RobotContainer:
    def __init__(self) -> None:
        #declaring the subsystems and setting up the drivetrain control
        self.joystick = commands2.button.CommandJoystick(config.driveConstants.joystickConstants.USB_ID)
        self.driveTrain = DriveTrainSubSystem(self.joystick)
        #self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.driveTrain))

    def checkJoystickInput(self, kInput: float):
        #getting values out of the joystick to do things
        if kInput < 0.1:
            kInput = 0
            print ("input = 0")
        else:
            kInput = kInput/2
            print("input = " + kInput)
        return kInput
    
    def disabledInit(self):
        pass

    def autonomousInit(self):
        pass

    def autonousPeriodic(self):
        pass

    def teleopInit(self):
        #self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.driveTrain))
        pass

    def teleopPeriodic(self):
        self.checkJoystickInput(0.0)

    def testInit(self):
        pass

    def testPeriodic(self):
        pass
