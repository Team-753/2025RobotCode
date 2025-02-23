from subsystems.drivetrain import DriveTrainSubSystem
from commands.defaultDriveCommand import DefaultDriveCommand, SlowDown
import commands2
import wpilib
import os
from wpimath import geometry, kinematics, estimator
import RobotConfig as config
import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine


from commands.cannonCommand import place, intake, DefaultPivotCommand,cannonToPosition, PivotDown, PivotUp
from commands.AlgaeCommand import GrabAlgae,ReleaseAlgae, FlipAlgaeSquisher
from commands.elevatorCommand import elevatorUp,elevatorDown,elevatorToPos
from commands.ClimberCommand import FlipClimber, FlipCompressor

from commands.simpleAutoCommands import *


from subsystems.cannon import CannonSubsystem
from subsystems.algae import AlgaeSquisher
from subsystems.elevator import elevatorSubSystem
from subsystems.Climber import ClimberSubsystem

#from pathplannerlib.auto import AutoBuilder
#from phoenix6 import swerve, hardware
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
#from wpimath.units import rotationsToRadians

class RobotContainer():
    _max_speed = 1
    _max_angular_rate = 1
    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    def __init__(self) -> None:
        #declaring the subsystems and setting up the drivetrain control
        self.joystick = commands2.button.CommandJoystick(0)
        self.AuxController = commands2.button.CommandXboxController(1)

        self.driveTrain = DriveTrainSubSystem(self.joystick)
        self.elevator = elevatorSubSystem()
        
        self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.driveTrain))
        
        self.scheduler = commands2.CommandScheduler()

        self.algae = AlgaeSquisher()
        self.cannon = CannonSubsystem()
        self.climber = ClimberSubsystem()
        
        
        #sets the climber down off rip
        self.climber.GoDown()
        self.climber.ComeBack()

        # Path follower
        self._auto_chooser = wpilib.SendableChooser()
        self._auto_chooser.setDefaultOption("forward", superSimpleAuto(self.driveTrain, [0, 1, 0], 2))
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Configure the button bindings
        self.configureButtonBindings()


    def configureButtonBindings(self) -> None:
        self.AuxController.rightTrigger(0.5).whileTrue(place(self.cannon)) 
        self.AuxController.leftTrigger(0.5).whileTrue(intake(self.cannon))
        
        
        self.AuxController.rightBumper().whileTrue(FlipClimber(self.climber))
        self.AuxController.leftBumper().whileTrue(FlipAlgaeSquisher(self.climber))
        
        
        self.AuxController.a().onTrue(elevatorToPos(self.elevator,0.4))
        self.AuxController.b().onTrue(elevatorToPos(self.elevator,1.5))
        self.AuxController.y().onTrue(elevatorToPos(self.elevator,4))
        self.AuxController.x().onTrue(elevatorToPos(self.elevator,0.0))
        
        self.AuxController.axisGreaterThan(1,.5).onTrue(elevatorDown(self.elevator))
        self.AuxController.axisLessThan(1,-.5).onTrue(elevatorUp(self.elevator))
        
        self.AuxController.axisLessThan(5,-.5).whileTrue(PivotUp(self.cannon))
        self.AuxController.axisGreaterThan(5,.5).whileTrue(PivotDown(self.cannon))
        
        self.joystickButton4 = self.joystick.button(4)
        self.joystickButton4.whileTrue(SlowDown(self.driveTrain))
        
        
        
        #self.AuxController.start().onTrue(elevatorToPos(self.elevator,0.5))
        self.AuxController.rightStick().onTrue(elevatorToPos(self.elevator,0.0))
        self.AuxController.leftStick().onTrue(cannonToPosition(self.cannon,0.5))
        #self.AuxController.leftStick().whileTrue(elevatorUp(self.elevator))


        self.AuxController.pov(0).whileTrue(GrabAlgae(self.algae))
        self.AuxController.pov(180).whileTrue(ReleaseAlgae(self.algae))

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        
        path = self._auto_chooser.getSelected()
        return path
      

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
        #self.driveTrain.setSwerveStates(1.0, 1.0, 1.0)
        pass

    def teleopPeriodic(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass
