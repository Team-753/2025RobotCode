from wpimath import geometry, kinematics
import wpilib
import navx
from subsystems.swerveModule import SwerveModule
from wpilib import DriverStation
from wpimath import controller, trajectory, estimator
import wpimath
from math import hypot, radians, pi, atan2
from typing import List
import commands2
import RobotConfig as rc



class DriveTrainSubSystem(commands2.Subsystem):

    def __init__(self, joystick: commands2.button.CommandJoystick) -> None:

        #camera settings
        self.stateStdDevs = 0.1, 0.1, 0.1
        self.visionMeasurementsStdDevs = 0., 0.9, 0.9

        #set up the joystick and navx sensor
        self.joystick = joystick
        self.navx = navx.AHRS.create_spi()

        #getting some important constants about the robot declared
        self.kMaxSpeed = rc.driveConstants.RobotSpeeds.maxSpeed
        self.kMaxAngularVelocity = rc.driveConstants.RobotSpeeds.maxSpeed /hypot(rc.robotDimensions.trackWidth / 2, rc.robotDimensions.wheelBase / 2)
        self.wheelBase = rc.robotDimensions.wheelBase
        self.trackWidth = rc.robotDimensions.trackWidth

        #defining the location of each swerve module on the can chain
        self.frontLeft = SwerveModule(rc.SwerveModules.frontLeft.driveMotorID, rc.SwerveModules.frontLeft.turnMotorID, rc.SwerveModules.frontLeft.CANCoderID)
        self.frontRight = SwerveModule(rc.SwerveModules.frontRight.driveMotorID, rc.SwerveModules.frontRight.turnMotorID, rc.SwerveModules.frontRight.CANCoderID)
        self.rearLeft = SwerveModule(rc.SwerveModules.rearLeft.driveMotorID, rc.SwerveModules.rearLeft.turnMotorID, rc.SwerveModules.rearLeft.CANCoderID)
        self.rearRight = SwerveModule(rc.SwerveModules.rearRight.driveMotorID, rc.SwerveModules.rearRight.turnMotorID, rc.SwerveModules.rearRight.CANCoderID)

        #renaming some variables so they are easier to use
        teleopConstants = rc.driveConstants.poseConstants

        #setting up how we send info to the wheels about the position of the turn motor
        rotationConstants = rc.driveConstants.ThetaPIDConstants.translationPIDConstants
        self.rotationPID = controller.PIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, rotationConstants.period)
        self.rotationPID.enableContinuousInput(-pi, pi)

        #a pose helps the robot know where it is, the alliance thing is for autos
        self.poseTolerance = geometry.Pose2d(geometry.Translation2d(x=teleopConstants.xPoseToleranceMeters, y=teleopConstants.yPoseToleranceMeters), geometry.Rotation2d(teleopConstants.thetaPoseToleranceRadians))
        self.alliance = wpilib.DriverStation.Alliance.kBlue

        #classes to help proccess info about where the robot is and how fast and in what direction it is moving
        self.KINEMATICS = kinematics.SwerveDrive4Kinematics(geometry.Translation2d(float(self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(self.trackWidth / 2), float(-self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(-self.wheelBase / 2)))
        self.poseEstimatior = estimator.SwerveDrive4PoseEstimator(kinematics.SwerveDrive4Kinematics(geometry.Translation2d(float(self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(self.trackWidth / 2), float(-self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(-self.wheelBase / 2))), 
                                                                  self.getNavxRotation2d(), self.getSwerveModulePositions(), geometry.Pose2d(0, 0, geometry.Rotation2d()), self.stateStdDevs, self.visionMeasurementsStdDevs)
        
        self.field = wpilib.Field2d()
        #wpilib.SmartDashboard.putData("Field: ", self.field)
        
    def getNavxRotation2d(self)-> geometry.Rotation2d:
        #getting the direction the robot is facing relative to where we started for field orient
        return self.navx.getRotation2d()
    
    def getPose(self)-> geometry.Pose2d:
        #figuring out where the robot is relative to where we started
        return self.poseEstimatior.getEstimatedPosition()
    
    def getSwerveModulePositions(self):
        #figuring out where the wheels are relative to where they started
        return self.frontLeft.getPosition(), self.frontRight.getPosition(), self.rearLeft.getPosition(), self.rearRight.getPosition()
    
    def resetPose(self, poseToSet: geometry.Pose2d)-> None:
        #we broke our pose so we are resetting it with our current location as 0
        self.poseEstimatior.resetPosition(self.getNavxRotation2d(), self.getSwerveModulePositions(), poseToSet)

    def resetFieldOrient(self)-> None:
        #this doesnt do anything useful
        self.navx.reset()

    def getJoystickInput(self)-> tuple[float]:
        #getting input from the joysticks and changing it so that we can use it
        constants = rc.driveConstants.joystickConstants
        print('getting input.')
        return(-wpimath.applyDeadband(self.joystick.getY(), constants.yDeadband),
               -wpimath.applyDeadband(self.joystick.getX(), constants.xDeadband),
               -wpimath.applyDeadband(self.joystick.getZ(), constants.theataDeadband))
    
    def setSwerveStates(self, xSpeed: float, ySpeed: float, zSpeed: float, fieldOrient = True)-> None:
        #using the input from the get joystick input function to tell the wheels where to go
        if fieldOrient:
            SwerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, self.poseEstimatior.getEstimatedPosition().rotation()))
        else:
            SwerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds(xSpeed, ySpeed, zSpeed))
        print ('set swerve states (drivetrain is running)')
        self.frontLeft.setState(SwerveModuleStates[0])
        self.frontRight.setState(SwerveModuleStates[1])
        self.rearLeft.setState(SwerveModuleStates[2])
        self.rearRight.setState(SwerveModuleStates[3])
    
    def joystickDrive(self, inputs: tuple[float])-> None:
        #proccessing the joystick input values and sending them to the set swerve states function
        print ("joystick drive is running.")
        xSpeed, ySpeed, zSpeed, = (inputs[0] * self.kMaxSpeed,
                                   inputs[1] * self.kMaxSpeed,
                                   inputs[2] * self.kMaxAngularVelocity * rc.driveConstants.RobotSpeeds.manualRotationSpeedFactor)
        self.setSwerveStates(xSpeed, ySpeed, zSpeed, self.poseEstimatior.getEstimatedPosition())

    def stationary(self)-> None:
        #stop the robot by breaking all the motors
        self.frontLeft.stop()
        self.frontRight.stop()
        self.rearLeft.stop()
        self.rearRight.stop()

    def coast(self)-> None:
        # coast the robot
        self.frontLeft.setNuetral()
        self.frontRight.setNuetral()
        self.rearLeft.setNuetral()
        self.rearRight.setNuetral()

    def getRobotRelativeChassisSpeeds(self):
        #where are all the wheels relative to where they started, not really sure what this is for
        states = (self.frontLeft.getPosition(), self.frontRight.getPosition(), self.rearLeft.getPosition(), self.rearRight.getPosition())
        return self.KINEMATICS.toChassisSpeeds(states)
    
    def getCurrentPose(self)-> geometry.Pose2d:
        # updating the current position of the robot
        return self.poseEstimatior.getEstimatedPosition()
    
    def periodic(self) -> None:
        #do these things a bunch of times
        currentPose = self.poseEstimatior.update(self.getNavxRotation2d(), self.getSwerveModulePositions())
        self.field.setRobotPose(currentPose)

    

