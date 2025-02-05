from subsystems.drivetrain import DriveTrainSubSystem
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


from commands.cannonCommand import place, intake
from commands.AlgaeCommand import GrabAlgae, ReleaseAlgae, ExtendPiston, RetractPiston
#from commands.elevatorCommand import ElevatorJoystickCommand
from commands.ClimberCommand import ExtendClimber, ReleaseClimber


from subsystems.cannon import CannonSubsystem
from subsystems.algae import AlgaeSquisher
#from subsystems.elevator import elevatorSubSystem
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
        self.driveTrain = DriveTrainSubSystem(self.joystick)
        
        self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.driveTrain))
        self.scheduler = commands2.CommandScheduler()

        
          
    

        #self.cannon = CannonSubsystem()
        #self.algae = AlgaeSquisher()
        #self.elevator = elevatorSubSystem()
        #self.climber = ClimberSubsystem()

        # Path follower
        """self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)"""

        # Configure the button bindings
        self.configureButtonBindings()


    def configureButtonBindings(self) -> None:
        '''self.AuxController.rightTrigger().whileTrue(place(self.cannon)) 
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
        
        self.climbersExtended = not self.climbersExtended  # Toggle state'''
        pass

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
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass
