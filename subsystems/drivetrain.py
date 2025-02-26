from wpimath import geometry, kinematics
import wpilib
import navx
from subsystems.swerveModule import SwerveModule
from wpilib import DriverStation
from wpimath import controller, trajectory, estimator
import wpimath
from math import hypot, pi
from typing import Tuple
import commands2
import RobotConfig as rc
import math

class DriveTrainSubSystem(commands2.Subsystem):
    def __init__(self, joystick: commands2.button.CommandJoystick) -> None:
        # Camera settings
        self.stateStdDevs = (0.0, 0.0, 0.0)
        self.visionMeasurementsStdDevs = (1.0, 1.0, 1.0)

        # Set up the joystick and navx sensor
        self.joystick = joystick
        self.navx = navx.AHRS.create_spi()

        # Getting important constants from RobotConfig
        self.kMaxSpeed = rc.driveConstants.RobotSpeeds.maxSpeed
        self.kMaxAngularVelocity = rc.driveConstants.RobotSpeeds.maxSpeed / hypot(rc.robotDimensions.trackWidth / 2, rc.robotDimensions.wheelBase / 2)
        self.wheelBase = rc.robotDimensions.wheelBase
        self.trackWidth = rc.robotDimensions.trackWidth
        self.speedMultiplier = 1

        # Define the location of each swerve module
        self.frontLeft = SwerveModule(rc.SwerveModules.frontLeft.driveMotorID,
                                      rc.SwerveModules.frontLeft.turnMotorID,
                                      rc.SwerveModules.frontLeft.CANCoderID,
                                      rc.SwerveModules.frontLeft.encoderOffset,
                                      rc.SwerveModules.frontLeft.isInverted)
        self.frontRight = SwerveModule(rc.SwerveModules.frontRight.driveMotorID,
                                       rc.SwerveModules.frontRight.turnMotorID,
                                       rc.SwerveModules.frontRight.CANCoderID,
                                       rc.SwerveModules.frontRight.encoderOffset,
                                       rc.SwerveModules.frontRight.isInverted)
        self.rearLeft = SwerveModule(rc.SwerveModules.rearLeft.driveMotorID,
                                     rc.SwerveModules.rearLeft.turnMotorID,
                                     rc.SwerveModules.rearLeft.CANCoderID,
                                     rc.SwerveModules.rearLeft.encoderOffset,
                                     rc.SwerveModules.rearLeft.isInverted)
        self.rearRight = SwerveModule(rc.SwerveModules.rearRight.driveMotorID,
                                      rc.SwerveModules.rearRight.turnMotorID,
                                      rc.SwerveModules.rearRight.CANCoderID,
                                      rc.SwerveModules.rearRight.encoderOffset,
                                      rc.SwerveModules.rearRight.isInverted)
        
        # Renaming some variables so they are easier to use
        teleopConstants = rc.driveConstants.poseConstants
        
        # Setup PID for rotation override
        rotationConstants = rc.driveConstants.ThetaPIDConstants.translationPIDConstants
        self.rotationPID = controller.PIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, rotationConstants.period)
        self.rotationPID.enableContinuousInput(-pi, pi)
        
        # Pose estimation and tolerance
        self.poseTolerance = geometry.Pose2d(
            geometry.Translation2d(x=teleopConstants.xPoseToleranceMeters, y=teleopConstants.yPoseToleranceMeters),
            geometry.Rotation2d(teleopConstants.thetaPoseToleranceRadians))
        self.alliance = wpilib.DriverStation.Alliance.kBlue
        
        # Kinematics and Pose Estimator
        self.KINEMATICS = kinematics.SwerveDrive4Kinematics(
            geometry.Translation2d(float(self.trackWidth / 2), float(self.wheelBase / 2)),
            geometry.Translation2d(float(self.trackWidth / 2), float(-self.wheelBase / 2)),
            geometry.Translation2d(float(-self.trackWidth / 2), float(self.wheelBase / 2)),
            geometry.Translation2d(float(-self.trackWidth / 2), float(-self.wheelBase / 2)))
        
        self.poseEstimatior = estimator.SwerveDrive4PoseEstimator(
            self.KINEMATICS,
            self.getNavxRotation2d(),
            self.getSwerveModulePositions(),
            geometry.Pose2d(geometry.Translation2d(), geometry.Rotation2d()),
            self.stateStdDevs,
            self.visionMeasurementsStdDevs)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field: ", self.field)

        # Joystick override variable
        self.joystickOverride: Tuple[float, float, float] = None

    def getNavxRotation2d(self) -> geometry.Rotation2d:
        # Get the robot's heading (inverted to account for sensor mounting)
        return geometry.Rotation2d(math.tau - self.navx.getRotation2d().radians())

    def getPose(self) -> geometry.Pose2d:
        # Return the current estimated pose
        return self.poseEstimatior.getEstimatedPosition()

    def getSwerveModulePositions(self):
        # Return the positions of each swerve module
        return (self.frontLeft.getPosition(), self.frontRight.getPosition(),
                self.rearLeft.getPosition(), self.rearRight.getPosition())
    
    def resetPose(self, poseToSet: geometry.Pose2d) -> None:
        # Reset the pose estimator with a new pose
        self.poseEstimatior.resetPosition(self.getNavxRotation2d(), self.getSwerveModulePositions(), poseToSet)
    
    def resetFieldOrient(self) -> None:
        self.navx.reset()
    
    def getJoystickInput(self) -> Tuple[float, float, float]:
        # If an override is set, return it; otherwise, process real joystick input.
        if self.joystickOverride is not None:
            return self.joystickOverride
        constants = rc.driveConstants.joystickConstants
        deadbandedY = -wpimath.applyDeadband(self.joystick.getY(), constants.yDeadband)
        deadbandedX = wpimath.applyDeadband(self.joystick.getX(), constants.xDeadband)
        deadbandedZ = -wpimath.applyDeadband(self.joystick.getZ(), constants.theataDeadband)
        return (deadbandedY, deadbandedX, deadbandedZ)
    
    def setSwerveStates(self, xSpeed: float, ySpeed: float, zSpeed: float, fieldOrient: bool = True) -> None:
        if fieldOrient:
            SwerveModuleStates = self.KINEMATICS.toSwerveModuleStates(
                kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, zSpeed, self.poseEstimatior.getEstimatedPosition().rotation()))
        else:
            SwerveModuleStates = self.KINEMATICS.toSwerveModuleStates(
                kinematics.ChassisSpeeds(xSpeed, ySpeed, zSpeed))
        self.frontLeft.setState(SwerveModuleStates[0])
        self.frontRight.setState(SwerveModuleStates[1])
        self.rearLeft.setState(SwerveModuleStates[2])
        self.rearRight.setState(SwerveModuleStates[3])
        wpilib.SmartDashboard.putNumber("current rotation", self.poseEstimatior.getEstimatedPosition().rotation().degrees())
        wpilib.SmartDashboard.putBoolean("have navx: ", self.navx.isConnected())
        wpilib.SmartDashboard.putNumber("last rotation: ", self.getCurrentPose().rotation().degrees())
        wpilib.SmartDashboard.putNumber("x distance: ", self.getCurrentPose().translation().X())
        wpilib.SmartDashboard.putNumber("y distance: ", self.getCurrentPose().translation().Y())
        wpilib.SmartDashboard.putNumber("navx position: ", self.getNavxRotation2d().degrees())

    def joystickDrive(self, inputs: Tuple[float, float, float]) -> None:
        # Process joystick inputs and drive the robot accordingly.
        xSpeed = inputs[0] * self.kMaxSpeed
        ySpeed = inputs[1] * self.kMaxSpeed
        zSpeed = inputs[2] * self.kMaxAngularVelocity * rc.driveConstants.RobotSpeeds.manualRotationSpeedFactor
        self.setSwerveStates(xSpeed, ySpeed, zSpeed, True)
        print("k", self.kMaxSpeed)

    def autoDrive(self, chassisSpeeds: kinematics.ChassisSpeeds, currentPose: geometry.Pose2d, fieldRelative: bool = True):
        if chassisSpeeds == kinematics.ChassisSpeeds(0, 0, 0):
            self.stationary()
        else:
            chassisSpeeds.omega = -chassisSpeeds.omega
            if fieldRelative:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(
                    kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vx, chassisSpeeds.vy, chassisSpeeds.omega, currentPose.rotation()))
            else:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(chassisSpeeds)
            self.frontLeft.setState(swerveModuleStates[0])
            self.frontRight.setState(swerveModuleStates[1])
            self.rearLeft.setState(swerveModuleStates[2])
            self.rearRight.setState(swerveModuleStates[3])
    
    def stationary(self) -> None:
        # Stop all drive motors.
        self.frontLeft.stop()
        self.frontRight.stop()
        self.rearLeft.stop()
        self.rearRight.stop()
    
    def coast(self) -> None:
        self.frontLeft.setNuetral()
        self.frontRight.setNuetral()
        self.rearLeft.setNuetral()
        self.rearRight.setNuetral()
    
    def getRobotRelativeChassisSpeeds(self):
        states = (self.frontLeft.getPosition(), self.frontRight.getPosition(),
                  self.rearLeft.getPosition(), self.rearRight.getPosition())
        return self.KINEMATICS.toChassisSpeeds(states)
    
    def getCurrentPose(self) -> geometry.Pose2d:
        return self.poseEstimatior.getEstimatedPosition()
    
    def halfSpeed(self):
        pass

    def fullSpeed(self):
        self.kMaxSpeed = rc.driveConstants.RobotSpeeds.maxSpeed
    
    def periodic(self):
        currentPose = self.poseEstimatior.update(self.getNavxRotation2d(), self.getSwerveModulePositions())
        self.field.setRobotPose(currentPose)
