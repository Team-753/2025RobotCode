from wpimath import geometry

class robotDimensions:
    trackWidth = 0.7366 #pretty sure this is the width between swerve modules
    wheelBase = 0.7366 #prolly length between modules

class SwerveModules:
    #holds all the info that pertains to the swerve modules

    drivingGearRatio = 6.75 # find a real number, number of motor spins per wheel spin
    turningGearRatio = 13.3714 # find a real number, number of motor spins per wheel rotation relative to the chassis
    #turning gear notes from motor to output: 10: 30: 22/16 : 88
    #contiued: 1/3* 3/2 * 5/3 * 1/6

    class frontLeft:
        #stores constants for the front left module
        driveMotorID = 1 #can id for the drive motor of the front left module
        CANCoderID = 2 #can id for the turn motor of the front left module
        encoderOffset = -0.5380859375 #encoder offset for the front left module so that zero is where we want it, come from the pheonix tuner
        isInverted = False #value to store indiviual inversion of the can coder. all the can coders are inverted and this value is not used
        turnMotorID = 3 #can id of the turn motor for the front left module

    class frontRight:
        # same as above but for the front right module
        driveMotorID = 4
        CANCoderID = 5
        encoderOffset = -0.057373046875
        isInverted = False
        turnMotorID = 6
    
    class rearRight:
        # same as above but for the front right module
        driveMotorID = 7
        CANCoderID = 8
        encoderOffset = -0.2392578125
        isInverted = False
        turnMotorID = 9
    
    class rearLeft:
        # same as above but for the front right module
        driveMotorID = 10
        CANCoderID = 11
        encoderOffset = -0.955078125
        isInverted = False
        turnMotorID = 12
        
class coralCannon:
    TopMotorID = 13
    BottomMotorID = 14
    pivotMotorID = 15
    
class Climber:
    solenoidForward = 3
    solenoidReverse = 2

    pneumaticsHubID = 20

class AuxController:
    #constants related to the aux controller
    USB_ID = 1 #usb id of the aux controller

class algaeSquisher:
    squisherPistonForward = 1
    squisherPistonReverse = 0

    squisherMotorID = 18

class elevator:
    leftMotorID= 17
    rightMotorID= 16

    
class driveConstants:
    #in meters, wheels have 4 in diameters
    wheelDiameter = 0.1016

    class joystickConstants:
        # constants related to the joystick
        USB_ID = 0 #usb id of the joystick
        xDeadband = 0.1 #deadband in the x of the joystick
        yDeadband = 0.1 #deadband on the y of the joystick
        theataDeadband = 0.2 #deadband for theta of the joystick

    class RobotSpeeds:
        #scalars and maximums for driving the robot
        maxSpeed = 4.8 #maximum speed of the robot
        maxAcceleration = 3 #maximum acceleration of the robot, i dont believe this is actually used
        manualRotationSpeedFactor = 1.4 #not sure what this is or does

    class poseConstants:
        #constants relating to the position of the robot on the field
        #for all the pid controllers with values outlined here, you should only need to manipulate the p value
        class translationPIDConstants:
            #constants used in the translation (x and y) pid controllers
            kP = 5.0 #the proportion value
            kI = 0.0 #the integral value
            kD = 0.0 #the derivative value
            period = 0.025 # this is not used, i think

        class rotationPIDConstants:
            #constants related to the angle pid controller
            kP = 10 #the proportional value
            kI = 0.0 #the integral value
            kD = 0.0 #the derivative value
            period = 0.025 #also not used, i think

        xPoseToleranceMeters = 0.05 #amount we can be off from our setpoint, used in the fancy auto commands
        yPoseToleranceMeters = 0.05 #same as above
        thetaPoseToleranceRadians = 0.02 #same as above
        teleopVelLimit = 2 #Im not at all sure what this is for or if it is used
        teleopAccelLimit = 2 #same as above
        autoVelLimit = 4 #maximum speed in the fancy auto commands in m/s
        autoAccelLimit = 2 #maximum acceleration in the fancy auto commands in m/s^2

    class ThetaPIDConstants:
        #i dont think this is used. it seems like a duplicate of parts of the above
        autoVelLimit = 4
        autoAccelLimit = 2
        xPoseToleranceMeters = 0.03
        yPoseToleranceMeters = 0.03
        period = 0.05
        class translationPIDConstants:
            kP = 3.0
            kI = 0.0
            kD = 0.0
            period = 0.05


class visionConstants:

    cameraName = ("limelight-jamal")  # name of your camera goes in parentheses

    # The physical mounting offset of the camera relative to the robot's center (in degrees)
    x_offset = 1.5  # Adjust as needed

    # Acceptable error (in degrees) to consider the target centered
    x_tolerance = 2.0  

    # Tuning multipliers (tune these separately for lateral vs. rotational control)
    lateralCorrectionConstant = 0.005  
    rotationalCorrectionConstant = 0.007  
    
    # For forward speed based on the target's area (desired tag area when at the proper distance)
    forwardSpeedMultiplier = 0.01  
    desired_tag_area = 15.0


