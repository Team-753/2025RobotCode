from wpimath import geometry

class robotDimensions:
    trackWidth = 0.7366
    wheelBase = 0.7366

class SwerveModules:

    drivingGearRatio = 5.68 # find a real number
    turningGearRatio = 13.3714 # find a real number
    #turning gear notes from motor to output: 10: 30: 22/16 : 88
    #contiued: 1/3* 3/2 * 5/3 * 1/6

    class frontLeft:
        driveMotorID = 1
        CANCoderID = 2
        encoderOffset = -0.5380859375
        isInverted = False
        turnMotorID = 3

    class frontRight:
        driveMotorID = 4
        CANCoderID = 5
        encoderOffset = -0.057373046875
        isInverted = False
        turnMotorID = 6
    
    class rearRight:
        driveMotorID = 7
        CANCoderID = 8
        encoderOffset = -0.2392578125
        isInverted = False
        turnMotorID = 9
    
    class rearLeft:
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
    USB_ID = 1

class algaeSquisher:
    squisherPistonForward = 1
    squisherPistonReverse = 0

    squisherMotorID = 18

class elevator:
    leftMotorID= 17
    rightMotorID= 16

    
class driveConstants:
    wheelDiameter = 0.1016

    class joystickConstants:
        USB_ID = 0
        xDeadband = 0.1
        yDeadband = 0.1
        theataDeadband = 0.2

    class RobotSpeeds:
        maxSpeed = 4.8
        maxAcceleration = 3
        manualRotationSpeedFactor = 0.7

    class poseConstants:
        class translationPIDConstants:
            kP = 5.0
            kI = 0.0
            kD = 0.0
            period = 0.025

        class rotationPIDConstants:
            kP = 0.15
            kI = 0.0
            kD = 0.0
            period = 0.025

        xPoseToleranceMeters = 0.05
        yPoseToleranceMeters = 0.05
        thetaPoseToleranceRadians = 0.01745
        teleopVelLimit = 4.25
        teleopAccelLimit = 3
        autoVelLimit = 2
        autoAccelLimit = 1

    class ThetaPIDConstants:
        autoVelLimit = 6
        autoAccelLimit = 5
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


