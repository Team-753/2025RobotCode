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
    def __init__(self, joystick: commands2.button.CommandJoystick, startingPose = geometry.Pose2d()) -> None:
        
        
        
        #camera settings
        self.stateStdDevs = 1, 1, 1 # telling the robot how much to trust where the wheels and navx say it is
        self.visionMeasurementsStdDevs = 0.0, 0.0, 0.0 #telling the robot how much to trust where the camera says it is
        #right now it is set to ignore the camera and completely trust the wheels and navx
        
        self.limeLight = limelight_camera.LimelightCamera(rc.visionConstants.cameraName) #creating a limelight object ti guve us location info
    
        

        #set up the joystick and navx sensor
        self.joystick = joystick #declaring the joystick so that we can use it to drive in teleop
        #self.navx = navx.AHRS.create_spi() #this is the recommended nethod for declaring a navx, but it stopped working
        self.startingPose = startingPose #saving our starting pose to make it easy to give the robot a rough estimate of where it starts. this will help guide it at the start of auto, until it sees an april tag.
        self.navx = navx.AHRS(navx.AHRS.NavXComType.kUSB1, 200) #creating the navx using a usb port. this neans it has to be plugged into a usb port on the rio. theres also an update rate but i dont think that really matters, i just added it while trying to fix it

        #getting some important constants about the robot declared
        self.kMaxSpeed = rc.driveConstants.RobotSpeeds.maxSpeed # declaring a max speed. Its enforced because values coming out of the joysticks are -1 to 1, so multiplying by that wont give a value above the max speed
        self.kMaxAngularVelocity = rc.driveConstants.RobotSpeeds.maxSpeed /hypot(rc.robotDimensions.trackWidth / 2, rc.robotDimensions.wheelBase / 2) #pretty similar to the above, except for the spin rate of the robot.
        self.wheelBase = rc.robotDimensions.wheelBase #telling the robot the distance between the wheels in the x(?) dimension
        self.trackWidth = rc.robotDimensions.trackWidth #telling the robot the distance between the wheels in the y(?) dimension
        #the above two values dont really matter if the bot is a square, just keep them the same. if its not a square then you have to use the real values
        self.speedMultiplier = 1

        #defining the location of each swerve module on the can chain and creating all the modules
        self.frontLeft = SwerveModule(rc.SwerveModules.frontLeft.driveMotorID, rc.SwerveModules.frontLeft.turnMotorID, rc.SwerveModules.frontLeft.CANCoderID, rc.SwerveModules.frontLeft.encoderOffset, rc.SwerveModules.frontLeft.isInverted)
        self.frontRight = SwerveModule(rc.SwerveModules.frontRight.driveMotorID, rc.SwerveModules.frontRight.turnMotorID, rc.SwerveModules.frontRight.CANCoderID, rc.SwerveModules.frontRight.encoderOffset, rc.SwerveModules.frontRight.isInverted)
        self.rearLeft = SwerveModule(rc.SwerveModules.rearLeft.driveMotorID, rc.SwerveModules.rearLeft.turnMotorID, rc.SwerveModules.rearLeft.CANCoderID, rc.SwerveModules.rearLeft.encoderOffset, rc.SwerveModules.rearLeft.isInverted)
        self.rearRight = SwerveModule(rc.SwerveModules.rearRight.driveMotorID, rc.SwerveModules.rearRight.turnMotorID, rc.SwerveModules.rearRight.CANCoderID, rc.SwerveModules.rearRight.encoderOffset, rc.SwerveModules.rearRight.isInverted)
        #self.limeLight = LimelightCamera("jamal")
        
        #renaming some variables so they are easier to use
        #everything until the self.KINEMATICS is actually implemented in commands.fancyAutoCommands, this code is just leftover from my expirementation
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
        #Basically a class that knows that we have four swerves and where they are relative to each other
        #self.poseEstimatior = estimator.SwerveDrive4PoseEstimator(kinematics.SwerveDrive4Kinematics(geometry.Translation2d(float(self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(self.trackWidth / 2), float(-self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(-self.wheelBase / 2))), 
                                                                  #self.getNavxRotation2d(), self.getSwerveModulePositions(), geometry.Pose2d(0, 0, geometry.Rotation2d()), self.stateStdDevs, self.visionMeasurementsStdDevs)
        self.poseEstimator = estimator.SwerveDrive4PoseEstimator(self.KINEMATICS, self.getNavxRotation2d(), self.getSwerveModulePositions(), self.startingPose, self.stateStdDevs, self.visionMeasurementsStdDevs)
        #this is where all the driving math actually gets done. It processes information coming from the different sources and the spits out speeds for all the motors in the drivetrain

        self.field = wpilib.Field2d() #creates a field on shuffleboard, useful for debugging autos
        #self.field.
        wpilib.SmartDashboard.putData("Field: ", self.field) #actually displays the field on shuffleboard

        """Here lies fancy auto stuff get ready for buggy fun
        ppconfig = RobotConfig.fromGUISettings()
        AutoBuilder.configure(self.getPose, self.resetPose, self.getRobotRelativeChassisSpeeds,)"""

        #Welcome to the Ryan Zone 
        self.joystickOverride = None  # New override variable
        
        
    def getNavxRotation2d(self)-> geometry.Rotation2d:
        """Returns the current heading of the robot relative to the heading from the last code reset"""
        #getting the direction the robot is facing relative to where we started for field orient
        return geometry.Rotation2d(math.tau - self.navx.getRotation2d().radians()) #this inverts the reading from the navx which lets the robot work with the inverted cancoders. likely not neccesary next year.
    
    def getPose(self)-> geometry.Pose2d:
        """Returns the current estimated position of the robot on the field"""
        #figuring out where the robot is relative to where we started
        return self.poseEstimator.getEstimatedPosition()
    
    def getSwerveModulePositions(self):
        """Returns the current positions of the swerve modules relative to where they started. Likely primarily useful for pathplanner compatiblilty"""
        #figuring out where the wheels are relative to where they started
        return self.frontLeft.getPosition(), self.frontRight.getPosition(), self.rearLeft.getPosition(), self.rearRight.getPosition()
    
    def resetPose(self, poseToSet: geometry.Pose2d)-> None:
        """Resets the robots pose on the field to 0"""
        #we broke our pose so we are resetting it with our current location as 0
        self.poseEstimator.resetPosition(self.getNavxRotation2d(), self.getSwerveModulePositions(), poseToSet)
    
    def resetFieldOrient(self)-> None:
        """Resets, in theory, the heading of the navx, there are better way to fix most issues with getting accurate heading readings tho"""
        #this doesnt do anything useful
        self.navx.reset()
    
    def getJoystickInput(self)-> tuple[float]: #getting input from the joysticks and changing it so that we can use it
        """Proccesses data from the joystick and returns adjusted values"""
        constants = rc.driveConstants.joystickConstants #getting our deadbands from the config file
        deadbandedY = -wpimath.applyDeadband(self.joystick.getY() * 10, constants.yDeadband) #applying the deadband applying a scalar to the speed, also inverting the direction because for some reason joysticks are upside down
        deadbandedX = wpimath.applyDeadband(self.joystick.getX() * 10, constants.xDeadband) #same as above, not inverting here because it got inverted properly by the cancoders, i think
        deadbandedZ = -wpimath.applyDeadband(self.joystick.getZ(), constants.theataDeadband) #same again, joystick is still upside down so i invert
        return (deadbandedY, deadbandedX, deadbandedZ) #returning the new values
   
    def getJoystickInputCurved(self)-> tuple[float]: #getting input from the joysticks and changing it so that we can use it
        """More sophisticated proccesing of the data that lets us control the deadband and apply a curved speed multiplier, resulting (hopefully) in better control of the robot by the drivers"""
        constants = rc.driveConstants.joystickConstants #getting deadbands from robot config
        #not quite sure what the math is doing here, ask Chris (the tall one)
        deadbandedY = -math.pow((abs(wpimath.applyDeadband(self.joystick.getY(), constants.yDeadband))),1.8)*numpy.sign(self.joystick.getY()) *5
        deadbandedX = math.pow((abs(wpimath.applyDeadband(self.joystick.getX(), constants.xDeadband))),1.8)*numpy.sign(self.joystick.getX()) *5
        deadbandedZ = -math.pow((abs(wpimath.applyDeadband(self.joystick.getZ(), constants.theataDeadband))),2.3)*numpy.sign(self.joystick.getZ())*2
        return (deadbandedY, deadbandedX, deadbandedZ) #returning values
        
    
    def setSwerveStates(self, xSpeed: float, ySpeed: float, zSpeed: float, fieldOrient = True)-> None:
        """Function that sets the states of the wheels (turn motor position and drive motor speed)"""
        #using the input from the get joystick input function to tell the wheels where to go
        if fieldOrient: #checking to see if we have a working navx so that we can have one direction always be forward
            SwerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, self.poseEstimator.getEstimatedPosition().rotation()))

        else: #otherwise the front of the robot gets to be forward
            SwerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds(xSpeed, ySpeed, zSpeed))
        self.frontLeft.setState(SwerveModuleStates[0])
        self.frontRight.setState(SwerveModuleStates[1])
        self.rearLeft.setState(SwerveModuleStates[2])
        self.rearRight.setState(SwerveModuleStates[3])
        #actually sets the states of the wheels
        wpilib.SmartDashboard.putNumber("current rotation", self.poseEstimator.getEstimatedPosition().rotation().degrees()) #put the direction were facing on shuffleboard


        
        #wpilib.SmartDashboard.putNumber("last rotation: ", self.getCurrentPose().rotation().degrees())
        wpilib.SmartDashboard.putNumber("x distance: ", self.getCurrentPose().translation().X())
        wpilib.SmartDashboard.putNumber("y distance: ", self.getCurrentPose().translation().Y())
        # puts our x and y translations on shuffleboard so we know where we are relative to where we started
        #wpilib.SmartDashboard.putNumber("navx position: ", self.getNavxRotation2d().degrees())


    
    def joystickDrive(self, inputs: tuple[float])-> None:
        #proccessing the joystick input values and sending them to the set swerve states function also applying the max speeds to the values
        xSpeed, ySpeed, zSpeed, = (inputs[0] * self.kMaxSpeed,
                                   inputs[1] * self.kMaxSpeed,
                                   inputs[2] * self.kMaxAngularVelocity * rc.driveConstants.RobotSpeeds.manualRotationSpeedFactor)
        #print(self.navx.getAngle())

        if self.joystick.getHID().getRawButton(4):
            self.setSwerveStates(xSpeed * .5, ySpeed * .5, zSpeed* .75, True)

        else:
            self.setSwerveStates(xSpeed, ySpeed, zSpeed, True)
        #if one of the buttons on the joystick has been pressed halve the speed of the robot to make lining up easier

    def autoDrive(self, chasssisSpeeds: kinematics.ChassisSpeeds, currentPose: geometry.Pose2d, fieldRelative = True):
        #im pretty sure this is a pathplanner compatible drive function, but i have literally never used it
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
        #where are all the wheels relative to where they started, not really sure what this is for prolly pathplanner something
        states = (self.frontLeft.getPosition(), self.frontRight.getPosition(), self.rearLeft.getPosition(), self.rearRight.getPosition())
        return self.KINEMATICS.toChassisSpeeds(states)
    
    def getCurrentPose(self)-> geometry.Pose2d:
        # a duplicate of get pose
        return self.poseEstimator.getEstimatedPosition()
    
    def halfSpeed(self):
        #self.kMaxSpeed = 0.5 * (rc.driveConstants.RobotSpeeds.maxSpeed)
        pass
    def fullSpeed(self):
        self.kMaxSpeed = rc.driveConstants.RobotSpeeds.maxSpeed

    def getCurrentVel(self):
        #currently doesnt do anything aah
        pass

    #the above three functions are from when i made a half speed button way too complicated and dont actually do anything
        
    
    def periodic(self):
        """This code runs every robot cycle (20 ms), so anything that needs to be run regularly should go here. all subsystems have a periodic but we dont always use it"""
        time = Timer.getFPGATimestamp()

        if  self.limeLight.hasDetection() == True:
            #check to see if the robot can see an april tag

            posedata,latency = self.limeLight.getPoseData()

            lockTime = time - (latency/1000)

            self.poseEstimator.addVisionMeasurement(posedata,lockTime)

        #the above code is to adjust for the fact that the the limelight only updates every 5 or so robot cycles. in order to aviod giving the robot old data while saying that its current we check to see if it is new and then give the robot the data, along with an estimate of when we got it
            

        currentPose = self.poseEstimator.update(self.getNavxRotation2d(), self.getSwerveModulePositions())
        #update the pose estimator with our most up to date info on where the robot is from all the systems

        self.field.setRobotPose(currentPose) #update the position of the robot on the field in shuffleboard for debugging
        wpilib.SmartDashboard.putNumber("X position: ", currentPose.X()) # needed a real number to debug something, so i put it on the dashboard
        wpilib.SmartDashboard.putBoolean("have navx: ", self.navx.isConnected()) #checking to see every cycle if the navx is connected
