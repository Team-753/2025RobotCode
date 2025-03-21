from wpimath import geometry, kinematics
import wpilib
import navx
from subsystems.swerveModule import SwerveModule
#from subsystems.limelight_camera import LimelightCamera
from wpilib import DriverStation
from wpimath import controller, trajectory, estimator
import wpimath
from math import hypot, radians, pi, atan2
from typing import List
import commands2
import RobotConfig as rc
import math,numpy
from subsystems import limelight_camera
from wpilib import Timer

from subsystems.limelight_camera import LimelightCamera


from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants

class DriveTrainSubSystem(commands2.Subsystem):
    def __init__(self, joystick: commands2.button.CommandJoystick) -> None:
        
        
        
        #camera settings
        self.stateStdDevs = 1, 1, 1
        self.visionMeasurementsStdDevs = 0.0, 0.0, 0.0

        
        self.limeLight = limelight_camera.LimelightCamera(rc.visionConstants.cameraName)
    
        

        #set up the joystick and navx sensor
        self.joystick = joystick
        #self.navx = navx.AHRS.create_spi()
        self.navx = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI, 55)

        #getting some important constants about the robot declared
        self.kMaxSpeed = rc.driveConstants.RobotSpeeds.maxSpeed
        self.kMaxAngularVelocity = rc.driveConstants.RobotSpeeds.maxSpeed /hypot(rc.robotDimensions.trackWidth / 2, rc.robotDimensions.wheelBase / 2)
        self.wheelBase = rc.robotDimensions.wheelBase
        self.trackWidth = rc.robotDimensions.trackWidth
        self.speedMultiplier = 1

        #defining the location of each swerve module on the can chain
        self.frontLeft = SwerveModule(rc.SwerveModules.frontLeft.driveMotorID, rc.SwerveModules.frontLeft.turnMotorID, rc.SwerveModules.frontLeft.CANCoderID, rc.SwerveModules.frontLeft.encoderOffset, rc.SwerveModules.frontLeft.isInverted)
        self.frontRight = SwerveModule(rc.SwerveModules.frontRight.driveMotorID, rc.SwerveModules.frontRight.turnMotorID, rc.SwerveModules.frontRight.CANCoderID, rc.SwerveModules.frontRight.encoderOffset, rc.SwerveModules.frontRight.isInverted)
        self.rearLeft = SwerveModule(rc.SwerveModules.rearLeft.driveMotorID, rc.SwerveModules.rearLeft.turnMotorID, rc.SwerveModules.rearLeft.CANCoderID, rc.SwerveModules.rearLeft.encoderOffset, rc.SwerveModules.rearLeft.isInverted)
        self.rearRight = SwerveModule(rc.SwerveModules.rearRight.driveMotorID, rc.SwerveModules.rearRight.turnMotorID, rc.SwerveModules.rearRight.CANCoderID, rc.SwerveModules.rearRight.encoderOffset, rc.SwerveModules.rearRight.isInverted)
        #self.limeLight = LimelightCamera("jamal")
        
        #renaming some variables so they are easier to use
        teleopConstants = rc.driveConstants.poseConstants
        
        #setting up how we send info to the wheels about the position of the turn motor, this is mostly for if we want a theta override function for auto place
        rotationConstants = rc.driveConstants.ThetaPIDConstants.translationPIDConstants
        self.rotationPID = controller.PIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, rotationConstants.period)
        self.rotationPID.enableContinuousInput(-pi, pi)
        
        #a pose helps the robot know where it is, the alliance thing is for autos
        self.poseTolerance = geometry.Pose2d(geometry.Translation2d(x=teleopConstants.xPoseToleranceMeters, y=teleopConstants.yPoseToleranceMeters), geometry.Rotation2d(teleopConstants.thetaPoseToleranceRadians))
        self.alliance = wpilib.DriverStation.Alliance.kBlue
        
        #classes to help proccess info about where the robot is and how fast and in what direction it is moving
        self.KINEMATICS = kinematics.SwerveDrive4Kinematics(geometry.Translation2d(float(self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(self.trackWidth / 2), float(-self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(-self.wheelBase / 2)))
        #self.poseEstimatior = estimator.SwerveDrive4PoseEstimator(kinematics.SwerveDrive4Kinematics(geometry.Translation2d(float(self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(self.trackWidth / 2), float(-self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(-self.wheelBase / 2))), 
                                                                  #self.getNavxRotation2d(), self.getSwerveModulePositions(), geometry.Pose2d(0, 0, geometry.Rotation2d()), self.stateStdDevs, self.visionMeasurementsStdDevs)
        self.poseEstimator = estimator.SwerveDrive4PoseEstimator(self.KINEMATICS, self.getNavxRotation2d(), self.getSwerveModulePositions(), geometry.Pose2d(geometry.Translation2d(), geometry.Rotation2d()), self.stateStdDevs, self.visionMeasurementsStdDevs)

        self.field = wpilib.Field2d()
        #self.field.
        wpilib.SmartDashboard.putData("Field: ", self.field)

        """Here lies fancy auto stuff get ready for buggy fun
        ppconfig = RobotConfig.fromGUISettings()
        AutoBuilder.configure(self.getPose, self.resetPose, self.getRobotRelativeChassisSpeeds,)"""

        #Welcome to the Ryan Zone 
        self.joystickOverride = None  # New override variable
        
        
    def getNavxRotation2d(self)-> geometry.Rotation2d:
        #getting the direction the robot is facing relative to where we started for field orient
        return geometry.Rotation2d(math.tau - self.navx.getRotation2d().radians()) #this inverts the reading from the navx which lets the robot work with the inverted cancoders. likely not neccesary next year.
    
    def getPose(self)-> geometry.Pose2d:
        #figuring out where the robot is relative to where we started
        return self.poseEstimator.getEstimatedPosition()
    
    def getSwerveModulePositions(self):
        #figuring out where the wheels are relative to where they started
        return self.frontLeft.getPosition(), self.frontRight.getPosition(), self.rearLeft.getPosition(), self.rearRight.getPosition()
    
    def resetPose(self, poseToSet: geometry.Pose2d)-> None:
        #we broke our pose so we are resetting it with our current location as 0
        self.poseEstimator.resetPosition(self.getNavxRotation2d(), self.getSwerveModulePositions(), poseToSet)
    
    def resetFieldOrient(self)-> None:
        #this doesnt do anything useful
        self.navx.reset()
    
    def getJoystickInput(self)-> tuple[float]: #getting input from the joysticks and changing it so that we can use it
        constants = rc.driveConstants.joystickConstants
        deadbandedY = -wpimath.applyDeadband(self.joystick.getY() * 10, constants.yDeadband)
        deadbandedX = wpimath.applyDeadband(self.joystick.getX() * 10, constants.xDeadband)
        deadbandedZ = -wpimath.applyDeadband(self.joystick.getZ(), constants.theataDeadband)
        return (deadbandedY, deadbandedX, deadbandedZ)
    def getJoystickInputCurved(self)-> tuple[float]: #getting input from the joysticks and changing it so that we can use it
        constants = rc.driveConstants.joystickConstants
        deadbandedY = -math.pow((abs(wpimath.applyDeadband(self.joystick.getY(), constants.yDeadband))),1.8)*numpy.sign(self.joystick.getY()) *5
        deadbandedX = math.pow((abs(wpimath.applyDeadband(self.joystick.getX(), constants.xDeadband))),1.8)*numpy.sign(self.joystick.getX()) *5
        deadbandedZ = -math.pow((abs(wpimath.applyDeadband(self.joystick.getZ(), constants.theataDeadband))),1.8)*numpy.sign(self.joystick.getZ())
        return (deadbandedY, deadbandedX, deadbandedZ)
        
    
    def setSwerveStates(self, xSpeed: float, ySpeed: float, zSpeed: float, fieldOrient = True)-> None:
        #using the input from the get joystick input function to tell the wheels where to go
        if fieldOrient:
            SwerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, self.poseEstimator.getEstimatedPosition().rotation()))

        else:
            SwerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds(xSpeed, ySpeed, zSpeed))
        self.frontLeft.setState(SwerveModuleStates[0])
        self.frontRight.setState(SwerveModuleStates[1])
        self.rearLeft.setState(SwerveModuleStates[2])
        self.rearRight.setState(SwerveModuleStates[3])
        wpilib.SmartDashboard.putNumber("current rotation", self.poseEstimator.getEstimatedPosition().rotation().degrees())


        
        #wpilib.SmartDashboard.putNumber("last rotation: ", self.getCurrentPose().rotation().degrees())
        wpilib.SmartDashboard.putNumber("x distance: ", self.getCurrentPose().translation().X())
        wpilib.SmartDashboard.putNumber("y distance: ", self.getCurrentPose().translation().Y())
        #wpilib.SmartDashboard.putNumber("navx position: ", self.getNavxRotation2d().degrees())


    
    def joystickDrive(self, inputs: tuple[float])-> None:
        #proccessing the joystick input values and sending them to the set swerve states function
        xSpeed, ySpeed, zSpeed, = (inputs[0] * self.kMaxSpeed,
                                   inputs[1] * self.kMaxSpeed,
                                   inputs[2] * self.kMaxAngularVelocity * rc.driveConstants.RobotSpeeds.manualRotationSpeedFactor)
        #print(self.navx.getAngle())

        if self.joystick.getHID().getRawButton(4):
            self.setSwerveStates(xSpeed * .5, ySpeed * .5, zSpeed* .75, True)

        else:
            self.setSwerveStates(xSpeed, ySpeed, zSpeed, True)

    def autoDrive(self, chasssisSpeeds: kinematics.ChassisSpeeds, currentPose: geometry.Pose2d, fieldRelative = True):
        if chasssisSpeeds == kinematics.ChassisSpeeds(0, 0, 0):
            self.stationary()
        else:
            chasssisSpeeds.omega = -chasssisSpeeds.omega
            if fieldRelative:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(chasssisSpeeds.vx, chasssisSpeeds.vy, chasssisSpeeds.omega, currentPose.rotation()))
            else:
                swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(chasssisSpeeds)
            
            self.frontLeft.setState(swerveModuleStates[0])
            self.frontRight.setState(swerveModuleStates[1])
            self.rearLeft.setState(swerveModuleStates[2])
            self.rearRight.setState(swerveModuleStates[3])
            
    
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
        return self.poseEstimator.getEstimatedPosition()
    
    def halfSpeed(self):
        #self.kMaxSpeed = 0.5 * (rc.driveConstants.RobotSpeeds.maxSpeed)
        pass
    def fullSpeed(self):
        self.kMaxSpeed = rc.driveConstants.RobotSpeeds.maxSpeed

    def getCurrentVel(self):
        #currently doesnt do anything aah
        pass
        
    
    def periodic(self):

        time = Timer.getFPGATimestamp()

        if  self.limeLight.hasDetection() == True:

            posedata,latency = self.limeLight.getPoseData()

            lockTime = time - (latency/1000)

            self.poseEstimator.addVisionMeasurement(posedata,lockTime)
            

        currentPose = self.poseEstimator.update(self.getNavxRotation2d(), self.getSwerveModulePositions())


        self.field.setRobotPose(currentPose)
        wpilib.SmartDashboard.putNumber("X position: ", currentPose.X())
        wpilib.SmartDashboard.putBoolean("have navx: ", self.navx.isConnected())
