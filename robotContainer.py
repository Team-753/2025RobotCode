from subsystems.drivetrain import DriveTrainSubSystem
from commands.defaultDriveCommand import DefaultDriveCommand, SlowDown
import commands2
import wpilib
from wpimath import geometry, kinematics, estimator
import RobotConfig as config
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine

from commands.cannonCommand import place, intake, DefaultPivotCommand, cannonToPosition, PivotDown, PivotUp
from commands.AlgaeCommand import GrabAlgae, ReleaseAlgae, FlipAlgaeSquisher
from commands.elevatorCommand import elevatorUp, elevatorDown, elevatorToPos
from commands.ClimberCommand import FlipClimber, FlipCompressor

from commands.VisionCommand import Lock
from commands.simpleAutoCommands import *

from subsystems.limelight_camera import LimelightCamera
from subsystems.cannon import CannonSubsystem
from subsystems.algae import AlgaeSquisher
from subsystems.elevator import elevatorSubSystem
from subsystems.Climber import ClimberSubsystem

from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d

class RobotContainer():
    _max_speed = 1
    _max_angular_rate = 1
    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    
    def __init__(self) -> None:
        # Declaring subsystems and setting up drivetrain control.
        self.limelight = LimelightCamera("limelight-jamal")
        self.joystick = commands2.button.CommandJoystick(0)
        self.AuxController = commands2.button.CommandXboxController(1)
        self.driveTrain = DriveTrainSubSystem(self.joystick)
        self.elevator = elevatorSubSystem()
        self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.driveTrain))
        self.scheduler = commands2.CommandScheduler()
        self.algae = AlgaeSquisher()
        self.cannon = CannonSubsystem()
        self.climber = ClimberSubsystem()
        
        # Set the climber down immediately.
        self.climber.GoDown()
        self.climber.ComeBack()

        # Path follower auto-chooser.
        self._auto_chooser = wpilib.SendableChooser()
        self._auto_chooser.setDefaultOption("forward", superSimpleAuto(self.driveTrain, [0, 1, 0], 2))
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Configure button bindings.
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        self.AuxController.rightTrigger(0.5).whileTrue(place(self.cannon))
        self.AuxController.leftTrigger(0.5).whileTrue(intake(self.cannon))
        self.AuxController.rightBumper().whileTrue(FlipClimber(self.climber))
        self.AuxController.leftBumper().whileTrue(FlipAlgaeSquisher(self.climber))
        self.AuxController.a().onTrue(elevatorToPos(self.elevator, 6))
        self.AuxController.b().onTrue(elevatorToPos(self.elevator, 13))
        self.AuxController.y().onTrue(elevatorToPos(self.elevator, 22))
        self.AuxController.x().onTrue(elevatorToPos(self.elevator, 0))
        
        self.AuxController.a().onTrue(cannonToPosition(self.cannon, 0.108))
        self.AuxController.b().onTrue(cannonToPosition(self.cannon, 0.133))
        self.AuxController.y().onTrue(cannonToPosition(self.cannon, 0.))
        self.AuxController.x().onTrue(cannonToPosition(self.cannon, 0.297))
        
        self.AuxController.axisGreaterThan(1, 0.5).whileTrue(elevatorDown(self.elevator))
        self.AuxController.axisLessThan(1, -0.5).whileTrue(elevatorUp(self.elevator))
        self.AuxController.axisLessThan(5, -0.5).whileTrue(PivotUp(self.cannon))
        self.AuxController.axisGreaterThan(5, 0.5).whileTrue(PivotDown(self.cannon))
        
        self.joystickButton4 = self.joystick.button(4)
        self.joystickButton4.onTrue(SlowDown(self.driveTrain))
        
        # Bind the Lock command with both limelight and driveTrain.
        self.joystickButton2 = self.joystick.button(2)
        self.joystickButton2.whileTrue(Lock(self.limelight, self.driveTrain))
        
        self.AuxController.rightStick().onTrue(elevatorToPos(self.elevator, 0.0))
        self.AuxController.leftStick().onTrue(cannonToPosition(self.cannon, 0.5))
        self.AuxController.pov(0).whileTrue(GrabAlgae(self.algae))
        self.AuxController.pov(180).whileTrue(ReleaseAlgae(self.algae))

    def getAutonomousCommand(self) -> commands2.Command:
        return self._auto_chooser.getSelected()

    def checkJoystickInput(self, kInput: float):
        if kInput < 0.1:
            kInput = 0
            print("input = 0")
        else:
            kInput = kInput / 2
            print("input = " + str(kInput))
        return kInput
    
    def disabledInit(self):
        elevatorToPos(self.elevator, 0)

    def autonomousInit(self):
        pass

    def autonousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass
