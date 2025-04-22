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


from commands.cannonCommand import *
from commands.AlgaeCommand import GrabAlgae,ReleaseAlgae, FlipAlgaeSquisher
from commands.elevatorCommand import elevatorUp,elevatorDown,elevatorToPos
from commands.ClimberCommand import FlipClimber, FlipCompressor



from commands.simpleAutoCommands import *
from commands.fancyAutoCommands import *


from subsystems.cannon import CannonSubsystem
from subsystems.algae import AlgaeSquisher
from subsystems.elevator import elevatorSubSystem
from subsystems.Climber import ClimberSubsystem

#from pathplannerlib.auto import AutoBuilder
#from phoenix6 import swerve, hardware
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from math import pi
#from wpimath.units import rotationsToRadians

class RobotContainer():
    #this does almost everything. handles all the places where subsystems interact and actually making stuff happen
    _max_speed = 1 #i think this is leftover from an old drivetrain thing
    _max_angular_rate = 1 #same as above
    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0) #not sure what this does or if it is used
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180) #same as above
    def __init__(self) -> None:
        #declaring the subsystems and setting up the drivetrain control

        self.joystick = commands2.button.CommandJoystick(0) #creating the joystick
        self.AuxController = commands2.button.CommandXboxController(1) #creating the xbox controller
        
        self.positionGetter = wpilib.SendableChooser() #creating a selector to allow input for our approximate starting location
        self.positionGetter.setDefaultOption("blue center", geometry.Pose2d(7.2702, 3.9942, geometry.Rotation2d(0))) #middle blue starting location
        self.positionGetter.addOption("blue right", geometry.Pose2d(1.8998, 3.9942, geometry.Rotation2d(0))) #right blue starting location (from the driverstation)
        self.positionGetter.addOption("blue left", geometry.Pose2d(6.0893, 3.9942, geometry.Rotation2d(0))) #left blue starting location, from driver station
        self.positionGetter.addOption("red center", geometry.Pose2d(7.2702, 8.2752, geometry.Rotation2d(0))) #middle red starting position
        self.positionGetter.addOption("red right", geometry.Pose2d(6.0893, 8.2752, geometry.Rotation2d(0))) # right red starting position from driver station
        self.positionGetter.addOption("red left", geometry.Pose2d(1.8998, 8.2752, geometry.Rotation2d(0))) #left red starting position from the driver station
        SmartDashboard.putData("position selector", self.positionGetter)

        self.startPos = self.positionGetter.getSelected() #saving the starting position to be used later.

        self.driveTrain = DriveTrainSubSystem(self.joystick, self.startPos) #creates the drive train and passes it the joystick
        self.elevator = elevatorSubSystem()
        self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.driveTrain)) #sets the default drive command to be the default command for the drive train. this means that it runs when nothing else is running
    
        self.scheduler = commands2.CommandScheduler() #ignore this, it doesnt do anything

        self.algae = AlgaeSquisher()
        self.cannon = CannonSubsystem()
        self.climber = ClimberSubsystem()
        
        
        #sets the climber down off rip
        self.climber.GoDown()
        self.climber.ComeBack()

        #build some complex commands. This should probably go somewhere else but... Im feeling lazy.
        self.level4Command = commands2.SequentialCommandGroup(elevatorToPos(self.elevator, 26).alongWith(AutoCannonPosition(self.cannon, 0.31, 1)).alongWith(AutoIntake(self.cannon, 1)), AutoCannonPosition(self.cannon, 0.175, 0.5), AutoPlace(self.cannon, 1), elevatorToPos(self.elevator, 0))

        # Path follower
        self._auto_chooser = wpilib.SendableChooser() #creates a selector to allow us to choose auto
        self._auto_chooser.addOption("a stop", superSimpleAuto(self.driveTrain, [-0.5, 0, 0], 1)) #an a stop auto that drive the robot forward at half speed
        self._auto_chooser.addOption("move", GoToPosition(geometry.Pose2d(-1.60, 0, pi), self.driveTrain)) #spins the robot around and drives it up to the reef
        self._auto_chooser.setDefaultOption("score center", commands2.SequentialCommandGroup(GoToPosition(geometry.Pose2d(0, 0, 0), self.driveTrain), GoToPosition(geometry.Pose2d(-1.30, 0, pi), self.driveTrain), elevatorToPos(self.elevator, 26).alongWith(AutoCannonPosition(self.cannon, 0.31, 1)).alongWith(AutoIntake(self.cannon, 1)), AutoCannonPosition(self.cannon, 0.175, 0.5), AutoPlace(self.cannon, 1), elevatorToPos(self.elevator, 0))) # turns the robot around, drives it to the reef, and when we get lucky scores on level 4 #GoToPosition(geometry.Pose2d(0, 0, 0), self.driveTrain))) #, AutoCannonPosition(self.cannon, 0.31, 1), GoToPosition(geometry.Pose2d(-1.60, 0, pi), self.driveTrain)))#, (AutoCannonPosition(self.cannon, 0.31, 1), GoToPosition(geometry.Pose2d(-1.65, 0, pi), self.driveTrain), AutoIntake(self.cannon, 1), AutoPlace(self.cannon, 1)))
        self._auto_chooser.addOption("score left", commands2.SequentialCommandGroup(GoToPosition(geometry.Pose2d(-1.79, 1.02, 2 * -pi/3),self.driveTrain), elevatorToPos(self.elevator, 26).alongWith(AutoCannonPosition(self.cannon, 0.31, 1)).alongWith(AutoIntake(self.cannon, 1)), AutoCannonPosition(self.cannon, 0.175, 0.5), AutoPlace(self.cannon, 1), elevatorToPos(self.elevator, 0))) #moves the robot from the left starting pose to the left side of the reef, aligns it and scores
        self._auto_chooser.addOption("score right", commands2.SequentialCommandGroup(GoToPosition(geometry.Pose2d(-1.82, -1.02, 2* pi/3), self.driveTrain), elevatorToPos(self.elevator, 26).alongWith(AutoCannonPosition(self.cannon, 0.31, 1)).alongWith(AutoIntake(self.cannon, 1)), AutoCannonPosition(self.cannon, 0.175, 0.5), AutoPlace(self.cannon, 1), elevatorToPos(self.elevator, 0))) #same as above but for the right side
        self._auto_chooser.addOption("center trough", commands2.SequentialCommandGroup(GoToPosition(geometry.Pose2d(-1.60, 0, 0), self.driveTrain), AutoCannonPosition(self.cannon, 0.31, 1), AutoTroughPlace(self.cannon, 1))) # turns the robot around, drives it to the reef, and scores in the trough
        SmartDashboard.putData("Auto Mode", self._auto_chooser) #displays the auto chooser

        # Configure the button bindings, which means we save all the setting and send them to the buttons
        self.configureButtonBindings()


    def configureButtonBindings(self) -> None:
        self.AuxController.rightTrigger(0.5).whileTrue(place(self.cannon)) 
        self.AuxController.leftTrigger(0.5).whileTrue(intake(self.cannon))
        
        
        self.AuxController.rightBumper().whileTrue(FlipClimber(self.climber))
        self.AuxController.leftBumper().whileTrue(FlipAlgaeSquisher(self.climber))
        
        
        self.AuxController.a().onTrue(elevatorToPos(self.elevator,6))
        self.AuxController.b().onTrue(elevatorToPos(self.elevator,13))
        self.AuxController.y().onTrue(elevatorToPos(self.elevator,26))
        self.AuxController.x().onTrue(elevatorToPos(self.elevator,0))
        #6/1

        self.AuxController.a().onTrue(cannonToPosition(self.cannon, 0.108))
        self.AuxController.b().onTrue(cannonToPosition(self.cannon, 0.133))
        self.AuxController.y().onTrue(cannonToPosition(self.cannon, 0.175))
        self.AuxController.x().onTrue(cannonToPosition(self.cannon, 0.31))

        self.AuxController.axisGreaterThan(1,.5).whileTrue(elevatorDown(self.elevator))
        self.AuxController.axisLessThan(1,-.5).whileTrue(elevatorUp(self.elevator))
        
        self.AuxController.axisLessThan(5,-.5).whileTrue(PivotUp(self.cannon))
        self.AuxController.axisGreaterThan(5,.5).whileTrue(PivotDown(self.cannon))
        
        #self.AuxController.leftStick().onTrue(elevatorToPos(self.elevator,6).andThen(AutoCannonPosition(self.cannon,0.2,0.1).andThen(AutoPlace(self.cannon,3))))#.andThen(AutoCannonPosition(self.cannon,desPos=0.2,stopTime=3).

        #self.joystickButton4 = self.joystick.button(4)
        #self.joystickButton4.onTrue(SlowDown(self.driveTrain))
        
    


        
        #self.AuxController.start().onTrue(elevatorToPos(self.elevator,0.5))
        self.AuxController.rightStick().onTrue(elevatorToPos(self.elevator,0.0))
        #self.AuxController.leftStick().whileTrue(elevatorUp(self.elevator))


        self.AuxController.pov(0).whileTrue(GrabAlgae(self.algae))
        self.AuxController.pov(180).whileTrue(ReleaseAlgae(self.algae))

        #these don't work at the same time right now, the elevator does stuff but the cannon doesn't.
        self.AuxController.pov(90).whileTrue(TopAlgaeRemoval(self.cannon).alongWith(elevatorToPos(self.elevator, 13)))
        #self.AuxController.pov(90).onTrue(elevatorToPos(self.elevator, 13))
        self.AuxController.pov(270).whileTrue(BottomAlgaeRemoval(self.cannon).alongWith(elevatorToPos(self.elevator, 6)))
        #self.AuxController.pov(270).onTrue(elevatorToPos(self.elevator, 6))
    

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        
        path = self._auto_chooser.getSelected() #the command to be run
        return path
    
    def getStartingPose(self) -> geometry.Pose2d:
        """use this to get provide an estimated starting pose
        
        :returns: the starting pose2d"""

        startPose = self.positionGetter.getSelected()
        return startPose
      

    def checkJoystickInput(self, kInput: float):
        #getting values out of the joystick to do things, this could definitely be deleted
        if kInput < 0.1:
            kInput = 0
            print ("input = 0")
        else:
            kInput = kInput/2
            print("input = " + kInput)
        return kInput
    
    def disabledInit(self):
        elevatorToPos(self.elevator, 0)

    def autonomousInit(self):
       pass

    def autonousPeriodic(self):
        #SmartDashboard.putString("command", str(self.)
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
