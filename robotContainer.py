from subsystems.drivetrain import MyDriveTrain
from commands.defaultDriveCommand import DefaultDriveCommand
import commands2
import wpilib
import os
from wpimath import geometry, kinematics, estimator
import RobotConfig as config
import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine
from telemetry import Telemetry
from tuner_constants import TunerConstants
from subsystems.swerveModule import myModules

from commands.cannonCommand import place, intake
from commands.AlgaeCommand import GrabAlgae, ReleaseAlgae, ExtendPiston, RetractPiston
from commands.elevatorCommand import ElevatorJoystickCommand
from commands.ClimberCommand import ExtendClimber, ReleaseClimber


from subsystems.cannon import CannonSubsystem
from subsystems.algae import AlgaeSquisher
from subsystems.elevator import elevatorSubSystem
from subsystems.Climber import ClimberSubsystem

from pathplannerlib.auto import AutoBuilder
from phoenix6 import swerve, hardware
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

class RobotContainer:
    _max_speed = 1
    _max_angular_rate = 1
    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    def __init__(self) -> None:
        #declaring the subsystems and setting up the drivetrain control
        self.joystick = commands2.button.CommandJoystick(0)
        self.AuxController = commands2.button.CommandXboxController(1)
        self.kDriveConstants = swerve.SwerveDrivetrainConstants()
        self.kDriveConstants.can_bus_name = "rio"
        self.driveTrain = MyDriveTrain(hardware.TalonFX, hardware.TalonFX, hardware.CANcoder, self.kDriveConstants, myModules)
        self.drive = (swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE))
        #self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.driveTrain))
        self.scheduler = commands2.CommandScheduler()

        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE))
        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()
        self.cannon = CannonSubsystem()
        self.algae = AlgaeSquisher()
        self.elevator = elevatorSubSystem()
        self.climber = ClimberSubsystem()

        # Path follower
        """self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)"""

        # Configure the button bindings
        self.configureButtonBindings()


    def configureButtonBindings(self) -> None:
        self.AuxController.rightTrigger().whileTrue(place(self.cannon)) 
        self.AuxController.leftTrigger().whileTrue(intake(self.cannon))

        self.AuxController.pov(0).whileTrue(GrabAlgae(self.algae))
        self.AuxController.pov(180).whileTrue(ReleaseAlgae(self.algae))

        #also entirely chat gpt code uhhhhhhh sorry
        ElevatorJoystickCommand(self.elevator, self.AuxController.getLeftY())

        #might work for getting pistons to flip i dont know entirely The man ChatGPT Code
        self.algaePistonExtended = False

        self.AuxController.leftBumper().onTrue(
            commands2.cmd.runOnce(
                lambda: self.ToggleAlgaePiston(), self.algae
            )
        )

        self.climbersExtended = False

        self.AuxController.rightBumper().onTrue(
            commands2.cmd.runOnce(
                lambda: self.ToggleClimberPistons(), self.climber
            )
        )
        


        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self.drive.with_velocity_x(
                        self._joystick.getLeftY() * self._max_speed * -1
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        self._joystick.getLeftX() * self._max_speed * -1
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        self._joystick.getRightX() * self._max_angular_rate * -1
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        self._joystick.pov(0).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0)
            )
        )
        self._joystick.pov(180).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(-0.5).with_velocity_y(0)
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.back() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.back() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        '''self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )'''

    def ToggleAlgaePiston(self):
        if self.algaePistonExtended:
            ExtendPiston(self.algae)
        else:
            RetractPiston(self.algae)
        
        self.pistonExtended = not self.pistonExtended  # Toggle state

    def ToggleClimberPistons(self):
        if self.climbersExtended:
            ExtendClimber(self.climber)
        else:
            ReleaseClimber(self.climber)
        
        self.climbersExtended = not self.climbersExtended  # Toggle state

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        pass
        #return self._auto_chooser.getSelected()
    
      

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
        print(self._joystick.getLeftX())

    def testInit(self):
        pass

    def testPeriodic(self):
        pass
